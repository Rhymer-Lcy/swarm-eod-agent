#!/usr/bin/env python3
"""Run agent brains in the offline approximation (development only).

    python -m devtools.simlab.offline_eval --role blue --map Map01 \
        --layout _local/layouts/synthetic_Map01_uniform_n10_s1.json --seed 1
    python -m devtools.simlab.offline_eval --role red --map Map01 --drones 5 --seed 1
    python -m devtools.simlab.offline_eval --role blue --agent p1_ddc6897 ...

Prints one JSON summary: outcome counters, a proxy score computed with the
official formulas where the inputs exist offline (for red, the official map
scorer and wall-coverage code on the fixture truth), brain compute time per
drone-step, and the heading error of the brains' pose estimates against the
offline truth (overall and while carrying). ``--agent`` selects the current
agent package (default) or a frozen comparator from ``devtools/comparators``;
``--profile`` adds the top functions of a cProfile run of the brains. These
are development signals from an approximate simulator, not competition
results.
"""

from __future__ import annotations

import argparse
import cProfile
import importlib
import io
import json
import math
import pstats
import sys
import time
from pathlib import Path

import numpy as np

from .fixtures import load_fixture
from .offline_sim import OfflineWorld


def agent_package(agent: str) -> str:
    if agent == "current":
        return "swarm_rescue.solutions.eod"
    return f"devtools.comparators.{agent}"


def make_factory(role: str, fx: dict, n: int, timings: list, agent: str = "current",
                 errors: list = None, profiler=None):
    package = agent_package(agent)
    if role == "blue":
        RescueBrain = importlib.import_module(f"{package}.blue").RescueBrain
    else:
        PlaceBrain = importlib.import_module(f"{package}.red").PlaceBrain

    size = tuple(fx["size_area"])

    class Timed:
        world = None

        def __init__(self, brain, ident):
            self.brain = brain
            self.ident = ident

        def outgoing_message(self):
            return self.brain.outgoing_message()

        def step(self, obs):
            t0 = time.perf_counter()
            if profiler is not None:
                profiler.enable()
            cmd = self.brain.step(obs)
            if profiler is not None:
                profiler.disable()
            timings.append(time.perf_counter() - t0)
            if errors is not None and Timed.world is not None and self.brain.est.initialized:
                d = Timed.world.drones[self.ident]
                err = math.atan2(math.sin(self.brain.est.heading - d.heading),
                                 math.cos(self.brain.est.heading - d.heading))
                errors.append((abs(math.degrees(err)), d.carrying is not None))
            return cmd

        def exploration_map(self):
            return self.brain.exploration_map()

    def factory(i: int):
        if role == "blue":
            brain = RescueBrain(i, n, size, int(fx["max_timestep_limit"]))
        else:
            brain = PlaceBrain(i, n, size, int(fx["max_timestep_limit"]),
                               float(fx["max_walltime_limit"]))
        return Timed(brain, i)

    factory.timed_class = Timed
    return factory


def red_scores(fx: dict, world: OfflineWorld) -> dict:
    from swarm_rescue.simulation.reporting.place_score_manager import PlaceScoreManager
    from swarm_rescue.tools.map_exploration_scoring import scorer_for_walltime

    truth = fx["walls"]
    out = {}
    if world.submitted is not None:
        b = scorer_for_walltime(float(fx["max_walltime_limit"])).score(
            truth, world.submitted, 0.0)
        out.update({"map": round(100 * b.s_map, 2), "credit": round(b.credit, 4),
                    "penalty": round(b.penalty, 4),
                    "predicted_pixels": b.predicted_pixels,
                    "charged_pixels": b.charged_pixels})
    else:
        out["map"] = 0.0
    manager = PlaceScoreManager()
    expected = world.n * 2
    wall = manager.compute_wall_score([(bb.x, bb.y) for bb in world.bombs], truth,
                                      boundary_mask=fx["boundary"],
                                      expected_n_bombs=expected)
    out["wall"] = round(100 * wall.score, 2)
    limit = int(fx["max_timestep_limit"])
    t = world.submit_step or limit
    out["time"] = round(100 * max(0.0, min(1.0, (limit - t) / (0.5 * limit))), 2)
    out["partial"] = round(0.5 * out["map"] + 0.2 * out["wall"] + 0.1 * out["time"], 2)
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--role", choices=["blue", "red"], required=True)
    parser.add_argument("--map", required=True)
    parser.add_argument("--layout", default=None)
    parser.add_argument("--drones", type=int, default=None)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--agent", default="current",
                        help="'current' or a comparator name in devtools/comparators")
    parser.add_argument("--carry-ratio-sigma", type=float, default=0.0,
                        help="per-step spread of the carried rotation ratio (probe: 0.36)")
    parser.add_argument("--profile", action="store_true")
    args = parser.parse_args(argv)
    fx = load_fixture(args.map)
    n = args.drones or (10 if args.role == "blue" else 5)
    bombs = []
    if args.layout:
        doc = json.loads(Path(args.layout).read_text(encoding="utf-8"))
        bombs = [(b["x"], b["y"]) for b in doc["bombs"]]
    timings: list = []
    errors: list = []
    profiler = cProfile.Profile() if args.profile else None
    factory = make_factory(args.role, fx, n, timings, args.agent, errors, profiler)
    world = OfflineWorld(fx, "rescue" if args.role == "blue" else "place", n,
                         factory, seed=args.seed, bombs=bombs, max_steps=args.max_steps,
                         carry_ratio_sigma=args.carry_ratio_sigma)
    factory.timed_class.world = world
    t0 = time.perf_counter()
    summary = world.run()
    summary["agent"] = args.agent
    summary["wall_s"] = round(time.perf_counter() - t0, 1)
    if errors:
        err = np.array(errors)
        summary["heading_error_deg"] = {
            "all_p50": round(float(np.median(err[:, 0])), 2),
            "all_p95": round(float(np.percentile(err[:, 0], 95)), 2)}
        carry = err[err[:, 1] > 0, 0]
        if carry.size:
            summary["heading_error_deg"].update({
                "carry_n": int(carry.size), "carry_p50": round(float(np.median(carry)), 2),
                "carry_p95": round(float(np.percentile(carry, 95)), 2)})
    arr = np.array(timings) * 1000
    summary["brain_ms_per_drone_step"] = {"mean": round(float(arr.mean()), 3),
                                          "p99": round(float(np.percentile(arr, 99)), 2),
                                          "max": round(float(arr.max()), 1)}
    if args.role == "red":
        summary["scores"] = red_scores(fx, world)
    summary["diagnostics"] = {i: w.brain.diagnostics() for i, w in enumerate(world.brains)}
    if profiler is not None:
        buf = io.StringIO()
        pstats.Stats(profiler, stream=buf).sort_stats("cumulative").print_stats(35)
        summary["profile_cumulative"] = buf.getvalue().splitlines()[:80]
    print(json.dumps(summary, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
