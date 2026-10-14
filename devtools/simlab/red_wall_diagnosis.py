#!/usr/bin/env python3
"""Where the red wall-coverage score is lost (offline diagnosis only).

    python -m devtools.simlab.red_wall_diagnosis --map Map04 --seed 1
    python -m devtools.simlab.red_wall_diagnosis --map Map04 --seed 1 --agent p2_540054b

Runs a red team in the offline approximation and scores, with the official
``PlaceScoreManager.compute_wall_score`` against the walls-only truth
fixture (evaluation-only ground truth, never visible to agents):

* ``actual``: the bombs the team placed;
* ``greedy_team_map_at_place``: a greedy placement of the same number of
  bombs, without travel cost, on the team's merged wall map at the start of
  the placement window (what the agents knew when they chose), restricted to
  cells the agents' own maps show as free and reachable;
* ``greedy_team_map_at_end``: the same on the map known at submission;
* ``greedy_truth``: the same greedy on the truth walls over every free cell a
  drone can occupy (an upper reference near the official optimum).

It also reports how much of the weighted truth walls the team map contained
at those times (recall within 3 px), the redundancy of the actual blasts
(sum of individual weighted coverage over the union) and the placement
counters. ``actual`` well below ``greedy_team_map_at_place`` points at the
placement procedure; the latter well below ``greedy_truth`` points at what
was known at placement time. Development signal from an approximate
simulator, not a competition result.
"""

from __future__ import annotations

import argparse
import copy
import importlib
import json
import math
import sys

import cv2
import numpy as np

from .fixtures import load_fixture
from .offline_sim import OfflineWorld
from .stable_cv import distance_transform

SCALE = 4
BOUNDARY_WEIGHT = 0.5


def _team_map(brains, render, merge) -> np.ndarray:
    """Union of every drone's merged, rendered wall map (team knowledge)."""
    out = None
    for b in brains:
        m = render(merge(b.evidence, (v[1] for v in b.snapshots.values())),
                   dc_box=b.dc.bbox()) > 0
        out = m if out is None else (out | m)
    return out.astype(np.uint8)


def _agent_candidates(brains, shape) -> np.ndarray:
    """Pixels the drones' own grids show as free, off obstacles and reachable."""
    h, w = shape
    allowed = np.zeros((h, w), dtype=bool)
    for b in brains:
        g = b.grid
        free = g.free() & ~(cv2.dilate(g.occupied().astype(np.uint8),
                                       np.ones((5, 5), np.uint8)) > 0)
        field = b.planner.field([g.to_cell(b.est.x, b.est.y)], allow_unknown=True)
        free &= np.isfinite(field)
        up = cv2.resize(free.astype(np.uint8), (g.nx * int(g.cell), g.ny * int(g.cell)),
                        interpolation=cv2.INTER_NEAREST)[:h, :w] > 0
        allowed |= up
    return allowed


def _truth_candidates(fx) -> np.ndarray:
    walls = fx["walls"].astype(np.uint8)
    dist = distance_transform(1 - walls)
    return dist >= 17.0


def _solid_inference(brains, truth: np.ndarray):
    """Area the drones take as solid obstacle interior (R2 placement weights):
    its size, the share of it that is truth wall (precision) and the share of
    all truth wall it covers. ``None`` for agents without the inference or
    when nothing is taken as solid."""
    solid = np.zeros(truth.shape, dtype=bool)
    for b in brains:
        if not hasattr(b, "_solid_interiors"):
            return None
        s = b._solid_interiors()
        k = int(b.grid.cell)
        up = np.kron(s.astype(np.uint8), np.ones((k, k), np.uint8))[:truth.shape[0],
                                                                     :truth.shape[1]]
        solid[:up.shape[0], :up.shape[1]] |= up > 0
    if not solid.any():
        return None
    hit = int((solid & (truth > 0)).sum())
    return {"pixels": int(solid.sum()), "precision": round(hit / int(solid.sum()), 3),
            "truth_wall_share": round(hit / max(int((truth > 0).sum()), 1), 3)}


def greedy(weights_px: np.ndarray, allowed_px: np.ndarray, n: int, radius: int):
    """Greedy max-coverage centers (world coordinates) on a 1:4 grid."""
    h, w = weights_px.shape
    small = cv2.resize(weights_px.astype(np.float32), (w // SCALE, h // SCALE),
                       interpolation=cv2.INTER_AREA) * SCALE * SCALE
    ok = cv2.resize(allowed_px.astype(np.uint8), (w // SCALE, h // SCALE),
                    interpolation=cv2.INTER_NEAREST) > 0
    r = radius / SCALE
    k = int(math.ceil(r))
    disk = np.zeros((2 * k + 1, 2 * k + 1), np.float32)
    cv2.circle(disk, (k, k), int(round(r)), 1.0, -1)
    remaining = small.copy()
    centers = []
    for _ in range(n):
        cov = cv2.filter2D(remaining, -1, disk, borderType=cv2.BORDER_CONSTANT)
        cov[~ok] = -1.0
        i = int(np.argmax(cov))
        if cov.flat[i] <= 0:
            break
        rr, cc = divmod(i, cov.shape[1])
        mask = np.zeros_like(remaining, dtype=np.uint8)
        cv2.circle(mask, (cc, rr), int(round(r)), 1, -1)
        remaining[mask > 0] = 0.0
        centers.append(((cc + 0.5) * SCALE - w / 2.0, h / 2.0 - (rr + 0.5) * SCALE))
    return centers


def weighted(map_px: np.ndarray, boundary: np.ndarray) -> np.ndarray:
    return np.where(boundary > 0, BOUNDARY_WEIGHT, 1.0) * (map_px > 0)


def recall(team_map: np.ndarray, truth: np.ndarray, boundary: np.ndarray) -> float:
    near = cv2.dilate(team_map.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
    wt = weighted(truth, boundary)
    return float((wt * near).sum() / max(wt.sum(), 1e-9))


def run(map_name: str, seed: int, drones: int = 5, agent: str = "current") -> dict:
    package = "swarm_rescue.solutions.eod" if agent == "current" else f"devtools.comparators.{agent}"
    PlaceBrain = importlib.import_module(f"{package}.red").PlaceBrain
    wallmap = importlib.import_module(f"{package}.wallmap")
    from swarm_rescue.simulation.reporting.place_score_manager import PlaceScoreManager

    fx = load_fixture(map_name)
    size = tuple(fx["size_area"])
    brains = []

    def factory(i):
        brains.append(PlaceBrain(i, drones, size, int(fx["max_timestep_limit"]),
                                 float(fx["max_walltime_limit"])))
        return brains[-1]

    world = OfflineWorld(fx, "place", drones, factory, seed=seed, bombs_per_drone=2,
                         carry_ratio_sigma=0.36)
    t_place = brains[0].t_place
    at_place = cand_place = solid_place = None
    events = []        # one per placement: what the placer knew when it placed
    inventory = [d.inventory for d in world.drones]
    while True:
        before = len(world.bombs)
        knowledge = [(list(b.placed), {k: v[:2] for k, v in b.place_claims.items()},
                      b.place_decided) for b in brains]
        running = world.step()
        for i, d in enumerate(world.drones):
            if d.inventory < inventory[i]:
                placed, claims, decided = knowledge[i]
                bomb = world.bombs[before + sum(1 for e in events if e["step"] == world.step_index - 1)]
                events.append({"step": world.step_index - 1, "drone": i,
                               "xy": (round(bomb.x, 1), round(bomb.y, 1)),
                               "known_placed": placed, "known_claims": claims,
                               "target_age": world.step_index - 1 - decided,
                               "emergency": world.step_index - 1 >= brains[i].t_rally
                               - int(0.03 * brains[i].max_steps)})
            inventory[i] = d.inventory
        if world.step_index == t_place:
            # Measured on a copy: reading the live brains (the center box
            # refresh in ``bbox()``, planner caches) could change the run
            # being measured (Phase 4, Risk C).
            frozen = copy.deepcopy(brains)
            at_place = _team_map(frozen, wallmap.render, wallmap.merge)
            cand_place = _agent_candidates(frozen, fx["walls"].shape)
            solid_place = _solid_inference(frozen, fx["walls"])
        if not running:
            break
    at_end = _team_map(brains, wallmap.render, wallmap.merge)
    cand_end = _agent_candidates(brains, fx["walls"].shape)

    truth, boundary = fx["walls"], fx["boundary"]
    n = drones * 2
    mgr = PlaceScoreManager()
    radius = int(round(mgr.config.blast_radius * min(truth.shape)))

    def score(points):
        res = mgr.compute_wall_score(points, truth, boundary_mask=boundary, expected_n_bombs=n)
        return round(100.0 * res.score, 2)

    actual = [(b.x, b.y) for b in world.bombs]
    individual = sum(mgr.compute_wall_score([p], truth, boundary_mask=boundary,
                                            expected_n_bombs=n).breakdown.get(
                                                "weighted_destroyed", 0.0) for p in actual)
    union = mgr.compute_wall_score(actual, truth, boundary_mask=boundary,
                                   expected_n_bombs=n).breakdown.get("weighted_destroyed", 0.0)
    close_pairs = sum(1 for i, a in enumerate(actual) for b in actual[i + 1:]
                      if math.hypot(a[0] - b[0], a[1] - b[1]) < radius)
    # For every placement within one blast radius of an earlier one: did the
    # placer know the earlier bomb (or its claim) when it placed?
    overlaps = []
    for j, ev in enumerate(events):
        for prev in events[:j]:
            d = math.hypot(ev["xy"][0] - prev["xy"][0], ev["xy"][1] - prev["xy"][1])
            if d >= radius:
                continue
            knew = any(math.hypot(p[0] - prev["xy"][0], p[1] - prev["xy"][1]) < 40.0
                       for p in ev["known_placed"] + list(ev["known_claims"].values()))
            overlaps.append({"later": ev["drone"], "earlier": prev["drone"],
                             "distance": round(d, 1), "steps_apart": ev["step"] - prev["step"],
                             "later_knew_earlier": knew, "same_drone": ev["drone"] == prev["drone"],
                             "later_target_age": ev["target_age"],
                             "later_emergency": ev["emergency"]})
    solid_end = _solid_inference(brains, truth)
    from swarm_rescue.tools.map_exploration_scoring import scorer_for_walltime
    map_score = None
    if world.submitted is not None:
        b = scorer_for_walltime(float(fx["max_walltime_limit"])).score(truth, world.submitted, 0.0)
        map_score = round(100.0 * b.s_map, 2)
    out = {
        "map": map_name, "seed": seed, "agent": agent, "blast_radius_px": radius,
        "map_score_submitted": map_score,
        "recall_submitted": round(recall(world.submitted, truth, boundary), 3)
        if world.submitted is not None else None,
        "bombs_placed": len(actual), "submit_step": world.submit_step, "t_place": t_place,
        "actual": score(actual),
        "greedy_team_map_at_place": score(greedy(weighted(at_place, boundary), cand_place, n, radius))
        if at_place is not None else None,
        "greedy_team_map_at_end": score(greedy(weighted(at_end, boundary), cand_end, n, radius)),
        "greedy_truth": score(greedy(weighted(truth, boundary), _truth_candidates(fx), n, radius)),
        "recall_at_place": round(recall(at_place, truth, boundary), 3) if at_place is not None else None,
        "recall_at_end": round(recall(at_end, truth, boundary), 3),
        "redundancy": round(individual / union, 3) if union > 0 else None,
        "close_pairs": close_pairs,
        "solid_inference_at_place": solid_place,
        "solid_inference_at_end": solid_end,
        "overlaps": overlaps,
        "placements": [{k: e[k] for k in ("step", "drone", "xy", "target_age", "emergency")}
                       for e in events],
        "counters": {k: sum(int(b.diag.get(k, 0)) for b in brains)
                     for k in ("placed", "place_retries", "emergency_drops")},
        "bombs": [(round(x, 1), round(y, 1)) for x, y in actual],
    }
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", required=True)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--drones", type=int, default=5)
    parser.add_argument("--agent", default="current")
    args = parser.parse_args(argv)
    print(json.dumps(run(args.map, args.seed, args.drones, args.agent), sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
