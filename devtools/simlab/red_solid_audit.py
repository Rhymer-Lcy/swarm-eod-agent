#!/usr/bin/env python3
"""Phase 4, Risk B: R2's solid-interior inference against truth (offline only).

    python -m devtools.simlab.red_solid_audit --map P4RoomsV1 --seed 1
    python -m devtools.simlab.red_solid_audit --map Map04 --seed 1 --variant nosolid

R2 weighs, for placement only, every enclosed unknown region of a drone's
occupancy grid as solid wall (``red.PlaceBrain._solid_interiors``). This tool
runs a red team in the offline approximation and, at the start of the
placement window and at every placement decision, measures on a *deep copy*
of each brain (so the measurement cannot change the run; the earlier
diagnosis read the live brains, see Risk C in docs/PHASE4_REPORT.md):

* the inferred area, the part of it that is truth wall (true positive) and
  the part that is not (false positive), the latter split by the region it
  lies in using the scenario's annotations (sealed room, room with a narrow
  or normal door, disposal center, elsewhere);
* the placement counterfactual: the target the drone chooses with the
  inference and the target it would choose without it, each scored alone
  with the official wall scorer against the truth walls.

``--variant nosolid`` runs a diagnostic variant whose inference returns
nothing (a subclass defined here; it is not a candidate and has no agent
identifier), so that whole-run wall scores with and without the inference
can be compared on the same seed. Development signals only.
"""

from __future__ import annotations

import argparse
import copy
import importlib
import json
import sys
from typing import Any, Dict, List

import numpy as np

from .fixtures import load_fixture
from .offline_eval import zone_hooks
from .offline_sim import OfflineWorld


def _px_mask(fx: dict, rect) -> np.ndarray:
    h, w = fx["walls"].shape
    x0, y0, x1, y1 = rect
    m = np.zeros((h, w), bool)
    r0, r1 = int(max(h / 2 - y1, 0)), int(min(h / 2 - y0, h))
    c0, c1 = int(max(x0 + w / 2, 0)), int(min(x1 + w / 2, w))
    m[r0:r1, c0:c1] = True
    return m


def region_masks(fx: dict) -> Dict[str, np.ndarray]:
    """Truth regions for attributing false positives (evaluation only)."""
    h, w = fx["walls"].shape
    out = {k: np.zeros((h, w), bool) for k in
           ("sealed_room", "narrow_door_room", "door_room", "disposal_center")}
    for a in fx.get("annotations") or []:
        kind = a.get("kind")
        if kind in out and "rect" in a:
            out[kind] |= _px_mask(fx, a["rect"])
    dc = fx["disposal_center"]
    out["disposal_center"] |= _px_mask(fx, (dc["x"] - dc["width"] / 2 - 8, dc["y"] - dc["height"] / 2 - 8,
                                            dc["x"] + dc["width"] / 2 + 8, dc["y"] + dc["height"] / 2 + 8))
    return out


def solid_px(brain, shape) -> np.ndarray:
    """The brain's inferred solid cells as a pixel mask, computed on a copy."""
    b = copy.deepcopy(brain)
    s = b._solid_interiors()
    k = int(b.grid.cell)
    up = np.kron(s.astype(np.uint8), np.ones((k, k), np.uint8))[:shape[0], :shape[1]] > 0
    out = np.zeros(shape, bool)
    out[:up.shape[0], :up.shape[1]] = up
    return out


def attribute(inferred: np.ndarray, fx: dict, regions: Dict[str, np.ndarray]) -> Dict[str, Any]:
    truth = fx["walls"] > 0
    tp = inferred & truth
    fp = inferred & ~truth
    rest = fp.copy()
    by_region = {}
    for name, m in regions.items():
        by_region[name] = int((fp & m).sum())
        rest &= ~m
    by_region["elsewhere"] = int(rest.sum())
    n = int(inferred.sum())
    return {"inferred_px": n, "true_wall_px": int(tp.sum()), "false_px": int(fp.sum()),
            "precision": round(int(tp.sum()) / n, 3) if n else None,
            "false_px_by_region": by_region}


def counterfactual(brain, now: int, truth, boundary, n_bombs: int) -> Dict[str, Any]:
    """Target with and without the inference, on copies of one brain."""
    from swarm_rescue.simulation.reporting.place_score_manager import PlaceScoreManager
    mgr = PlaceScoreManager()
    with_solid = copy.deepcopy(brain)._choose_place_target(now)
    nos = copy.deepcopy(brain)
    nos._solid_interiors = lambda: np.zeros(nos.grid.logodds.shape, dtype=bool)
    without = nos._choose_place_target(now)

    def alone(p):
        if p is None:
            return None
        res = mgr.compute_wall_score([p], truth, boundary_mask=boundary, expected_n_bombs=n_bombs)
        return round(float(res.breakdown.get("weighted_destroyed", 0.0)), 1)

    differ = (with_solid is None) != (without is None) or (
        with_solid is not None and np.hypot(with_solid[0] - without[0],
                                            with_solid[1] - without[1]) > 8.0)
    return {"drone": brain.id, "step": now,
            "target": [round(v, 1) for v in with_solid] if with_solid else None,
            "target_without": [round(v, 1) for v in without] if without else None,
            "differs": bool(differ), "truth_cover": alone(with_solid),
            "truth_cover_without": alone(without)}


def run(map_name: str, seed: int, drones: int = 5, agent: str = "current",
        variant: str = "r2", decisions: bool = True) -> Dict[str, Any]:
    package = "swarm_rescue.solutions.eod" if agent == "current" else f"devtools.comparators.{agent}"
    PlaceBrain = importlib.import_module(f"{package}.red").PlaceBrain
    if variant == "nosolid":
        class NoSolid(PlaceBrain):            # diagnostic variant, not a candidate
            def _solid_interiors(self):
                return np.zeros(self.grid.logodds.shape, dtype=bool)
        PlaceBrain = NoSolid
    from swarm_rescue.simulation.reporting.place_score_manager import PlaceScoreManager
    from swarm_rescue.tools.map_exploration_scoring import scorer_for_walltime

    fx = load_fixture(map_name)
    size = tuple(fx["size_area"])
    truth, boundary = fx["walls"], fx["boundary"]
    regions = region_masks(fx)
    brains: List[Any] = []

    def factory(i):
        brains.append(PlaceBrain(i, drones, size, int(fx["max_timestep_limit"]),
                                 float(fx["max_walltime_limit"])))
        return brains[-1]

    holder: list = []
    world = OfflineWorld(fx, "place", drones, factory, seed=seed, bombs_per_drone=2,
                         carry_ratio_sigma=0.36, **zone_hooks(fx, holder))
    holder.append(world)
    n_bombs = drones * 2
    at_place = None
    decided = {b.id: b.place_decided for b in brains}
    events = []
    while True:
        running = world.step()
        now = world.step_index - 1
        if now == brains[0].t_place and variant == "r2":
            union = np.zeros(truth.shape, bool)
            per = []
            for b in brains:
                m = solid_px(b, truth.shape)
                union |= m
                per.append(attribute(m, fx, regions))
            at_place = {"team_union": attribute(union, fx, regions), "per_drone": per}
        if decisions and variant == "r2":
            for b in brains:
                if b.place_decided != decided[b.id] and b.place_decided == now:
                    # The decision just made used the brain's state before this
                    # step's updates; the copy re-decides on the state after
                    # them (one step later), which is close but not identical.
                    events.append(counterfactual(b, now, truth, boundary, n_bombs))
                decided[b.id] = b.place_decided
        if not running:
            break
    mgr = PlaceScoreManager()
    placed = [(bb.x, bb.y) for bb in world.bombs]
    wall = mgr.compute_wall_score(placed, truth, boundary_mask=boundary, expected_n_bombs=n_bombs)
    map_score = None
    if world.submitted is not None:
        s = scorer_for_walltime(float(fx["max_walltime_limit"])).score(truth, world.submitted, 0.0)
        map_score = round(100.0 * s.s_map, 2)
    end = None
    if variant == "r2":
        union = np.zeros(truth.shape, bool)
        for b in brains:
            union |= solid_px(b, truth.shape)
        end = attribute(union, fx, regions)
    differing = [e for e in events if e["differs"]]
    return {
        "map": map_name, "seed": seed, "agent": agent, "variant": variant,
        "fixture_sha256": fx.get("spec_sha256"),
        "wall": round(100.0 * wall.score, 2), "map_score": map_score,
        "placed": len(placed), "submit_step": world.submit_step,
        "solid_at_place": at_place, "solid_at_end": end,
        "decisions": len(events), "decisions_changed_by_inference": len(differing),
        "changed_truth_cover_sum": round(sum(e["truth_cover"] or 0 for e in differing), 1),
        "changed_truth_cover_without_sum": round(sum(e["truth_cover_without"] or 0
                                                     for e in differing), 1),
        "decision_events": events,
        "bombs": [(round(x, 1), round(y, 1)) for x, y in placed],
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", required=True)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--drones", type=int, default=5)
    parser.add_argument("--agent", default="current")
    parser.add_argument("--variant", choices=["r2", "nosolid"], default="r2")
    args = parser.parse_args(argv)
    print(json.dumps(run(args.map, args.seed, args.drones, args.agent, args.variant),
                     sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
