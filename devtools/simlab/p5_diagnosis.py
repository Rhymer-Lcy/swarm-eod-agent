#!/usr/bin/env python3
"""Phase 5 diagnosis of carrier behaviour near a walled disposal center.

Evaluation tooling only: it reads a round's truth log (``truth.npz``, written
by the instrumented runner) and the custom scenario's specification, both of
which are ground truth that agent code never sees.

    python -m devtools.simlab.p5_diagnosis round _local/runs/<run_id> --map P4PocketV2

For every carry interval of every drone, the carrying steps are classified by
the drone's true position: far from the center (more than ``--near`` px from
its box), near it with a true line of sight to some part of the center, or
near it without one (behind a wall). A per-drone time course in bins of
``--bin`` steps adds the agent's own state label. The line-of-sight test
samples straight segments from the drone centre to 64 points on the center's
outline against the specification's wall raster (walls, boxes and frame).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from . import p4_scenarios


def _box(spec: dict) -> Tuple[float, float, float, float]:
    dc = spec["disposal_center"]
    return (dc["x"] - dc["width"] / 2.0, dc["y"] - dc["height"] / 2.0,
            dc["x"] + dc["width"] / 2.0, dc["y"] + dc["height"] / 2.0)


def box_distance(x: float, y: float, box) -> float:
    dx = max(box[0] - x, 0.0, x - box[2])
    dy = max(box[1] - y, 0.0, y - box[3])
    return float(np.hypot(dx, dy))


def outline(box, n: int = 64) -> np.ndarray:
    """``n`` points spread over the center's outline, 1 px outside it."""
    x0, y0, x1, y1 = box[0] - 1, box[1] - 1, box[2] + 1, box[3] + 1
    per = 2 * (x1 - x0) + 2 * (y1 - y0)
    out = []
    for s in np.linspace(0.0, per, n, endpoint=False):
        if s < x1 - x0:
            out.append((x0 + s, y0))
        elif s < (x1 - x0) + (y1 - y0):
            out.append((x1, y0 + s - (x1 - x0)))
        elif s < 2 * (x1 - x0) + (y1 - y0):
            out.append((x1 - (s - (x1 - x0) - (y1 - y0)), y1))
        else:
            out.append((x0, y1 - (s - 2 * (x1 - x0) - (y1 - y0))))
    return np.array(out)


class Sight:
    """True line of sight from a point to the center (specification raster)."""

    def __init__(self, spec: dict) -> None:
        r = p4_scenarios.raster(spec)
        self.walls = r["walls"].astype(bool)
        self.h, self.w = self.walls.shape
        self.box = _box(spec)
        self.targets = outline(self.box)

    def visible(self, x: float, y: float) -> bool:
        tx, ty = self.targets[:, 0], self.targets[:, 1]
        n = int(max(np.max(np.abs(tx - x)), np.max(np.abs(ty - y)))) + 2
        t = np.linspace(0.0, 1.0, n)[None, :]
        px = x + t * (tx - x)[:, None]
        py = y + t * (ty - y)[:, None]
        rows = np.clip(np.round(self.h / 2.0 - py).astype(int), 0, self.h - 1)
        cols = np.clip(np.round(px + self.w / 2.0).astype(int), 0, self.w - 1)
        blocked = self.walls[rows, cols].any(axis=1)
        return bool((~blocked).any())


def round_report(run_dir: Path, map_name: str, near: float, bin_steps: int) -> Dict:
    spec = p4_scenarios.spec_by_name(map_name)
    sight = Sight(spec)
    box = sight.box
    data = np.load(run_dir / "truth.npz")
    drones, brain = data["drones"], data["brain"]
    f = {n: i for i, n in enumerate(str(v) for v in data["drone_fields"])}
    codes = {v: k for k, v in json.loads(str(data["state_codes"])).items()}
    out: Dict = {"run": run_dir.name, "map": map_name, "near_px": near,
                 "dc_box": [round(v, 1) for v in box], "drones": {}}
    for ident in np.unique(drones[:, f["drone"]]).astype(int):
        rows = drones[drones[:, f["drone"]] == ident]
        rows = rows[np.argsort(rows[:, f["step"]])]
        est = brain[brain[:, 1] == ident]
        state_by_step = {int(r[0]): codes.get(int(r[7]), "?") for r in est}
        held = rows[:, f["grasped"]] > 0
        steps = rows[:, f["step"]].astype(int)
        xs, ys = rows[:, f["x"]], rows[:, f["y"]]
        far = near_vis = near_blind = 0
        bins: List[Dict] = []
        cur = None
        for k in np.flatnonzero(held):
            d = box_distance(xs[k], ys[k], box)
            if d > near:
                cls = "far"
                far += 1
            elif sight.visible(xs[k], ys[k]):
                cls = "near_visible"
                near_vis += 1
            else:
                cls = "near_blind"
                near_blind += 1
            b = int(steps[k]) // bin_steps
            if cur is None or cur["bin"] != b:
                cur = {"bin": b, "far": 0, "near_visible": 0, "near_blind": 0,
                       "states": {}}
                bins.append(cur)
            cur[cls] += 1
            label = state_by_step.get(int(steps[k]), "?")
            cur["states"][label] = cur["states"].get(label, 0) + 1
        health = rows[:, f["health"]]
        out["drones"][int(ident)] = {
            "carry_steps": int(held.sum()), "far": far, "near_visible": near_vis,
            "near_blind": near_blind,
            "first_grasp_step": int(steps[held][0]) if held.any() else None,
            "health_start": float(health[0]), "health_end": float(health[-1]),
            "bins": [{"steps": f"{c['bin'] * bin_steps}-{(c['bin'] + 1) * bin_steps - 1}",
                      "far": c["far"], "near_visible": c["near_visible"],
                      "near_blind": c["near_blind"], "states": c["states"]} for c in bins],
        }
    return out


def track(run_dir: Path, map_name: str, idents: List[int], first: int, last: int,
          every: int) -> List[Dict]:
    """True and estimated pose, target, state and grasp of some drones, and
    the free or held bombs within 60 px of each, every ``every`` steps."""
    spec = p4_scenarios.spec_by_name(map_name)
    sight = Sight(spec)
    data = np.load(run_dir / "truth.npz")
    drones, brain, bombs = data["drones"], data["brain"], data["bombs"]
    f = {n: i for i, n in enumerate(str(v) for v in data["drone_fields"])}
    codes = {v: k for k, v in json.loads(str(data["state_codes"])).items()}
    out = []
    for step in range(first, last + 1, every):
        row = {"step": step, "drones": {}}
        bstep = bombs[bombs[:, 0] == (step // 5) * 5]
        for ident in idents:
            d = drones[(drones[:, f["drone"]] == ident) & (drones[:, f["step"]] == step)]
            b = brain[(brain[:, 1] == ident) & (brain[:, 0] == step)]
            if not len(d):
                continue
            x, y = float(d[0, f["x"]]), float(d[0, f["y"]])
            near = [(int(r[1]), round(float(r[2]), 1), round(float(r[3]), 1), int(r[4]))
                    for r in bstep if np.hypot(r[2] - x, r[3] - y) < 60.0]
            entry = {"x": round(x, 1), "y": round(y, 1),
                     "dc_dist": round(box_distance(x, y, sight.box), 1),
                     "dc_visible": sight.visible(x, y),
                     "grasped": int(d[0, f["grasped"]]), "health": int(d[0, f["health"]]),
                     "bombs_near": near}
            if len(b):
                entry.update({"est": [round(float(b[0, 2]), 1), round(float(b[0, 3]), 1)],
                              "target": [round(float(b[0, 5]), 1), round(float(b[0, 6]), 1)],
                              "state": codes.get(int(b[0, 7]), "?")})
            row["drones"][ident] = entry
        out.append(row)
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("round", help="classify carrying steps of an instrumented round")
    p.add_argument("run_dir", type=Path)
    p.add_argument("--map", required=True)
    p.add_argument("--near", type=float, default=100.0)
    p.add_argument("--bin", type=int, default=250)
    t = sub.add_parser("track", help="trajectories of some drones in a step range")
    t.add_argument("run_dir", type=Path)
    t.add_argument("--map", required=True)
    t.add_argument("--drones", required=True, help="comma-separated identifiers")
    t.add_argument("--first", type=int, required=True)
    t.add_argument("--last", type=int, required=True)
    t.add_argument("--every", type=int, default=25)
    b = sub.add_parser("bombs", help="true bomb positions and grasp flags at some steps")
    b.add_argument("run_dir", type=Path)
    b.add_argument("--steps", required=True, help="comma-separated steps (rounded down to the log period)")
    args = parser.parse_args(argv)
    if args.cmd == "bombs":
        data = np.load(args.run_dir / "truth.npz")
        bombs = data["bombs"]
        logged = np.unique(bombs[:, 0]).astype(int)
        for want in (int(v) for v in args.steps.split(",")):
            step = int(logged[logged <= want].max())
            rows = bombs[bombs[:, 0] == step]
            print(json.dumps({"step": step, "bombs": [
                [int(r[1]), round(float(r[2]), 1), round(float(r[3]), 1), int(r[4])]
                for r in rows]}, separators=(",", ":")))
        return 0
    if args.cmd == "round":
        print(json.dumps(round_report(args.run_dir, args.map, args.near, args.bin), indent=1))
    else:
        idents = [int(v) for v in args.drones.split(",")]
        for row in track(args.run_dir, args.map, idents, args.first, args.last, args.every):
            print(json.dumps(row, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
