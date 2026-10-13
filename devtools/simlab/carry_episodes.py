#!/usr/bin/env python3
"""Carry episodes of an instrumented blue round (diagnostics only).

    python -m devtools.simlab.carry_episodes _local/runs/<run_id> --map Map03

From the truth log: for every drone, each interval during which it held a
bomb, whether the release earned a disposal (reward increase), its length,
the drone's distance to the disposal-center box at the end, the health lost
during the episode and the heading error of the agent's estimate while
carrying. The disposal-center box comes from the walls-only fixture of the
map (evaluation-only ground truth, never used by agents).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from .fixtures import load_fixture


def box_distance(x: float, y: float, box) -> float:
    dx = max(box[0] - x, 0.0, x - box[2])
    dy = max(box[1] - y, 0.0, y - box[3])
    return float(np.hypot(dx, dy))


def episodes(run_dir: Path, map_name: str) -> dict:
    data = np.load(run_dir / "truth.npz")
    drones, brain = data["drones"], data["brain"]
    f = {n: i for i, n in enumerate(str(v) for v in data["drone_fields"])}
    fx = load_fixture(map_name)
    dc = fx["disposal_center"]
    box = (dc["x"] - dc["width"] / 2, dc["y"] - dc["height"] / 2,
           dc["x"] + dc["width"] / 2, dc["y"] + dc["height"] / 2)
    out = []
    for ident in np.unique(drones[:, f["drone"]]).astype(int):
        rows = drones[drones[:, f["drone"]] == ident]
        rows = rows[np.argsort(rows[:, f["step"]])]
        est = brain[brain[:, 1] == ident]
        est_by_step = {int(r[0]): r for r in est}
        held = rows[:, f["grasped"]] > 0
        starts = np.flatnonzero(held & ~np.r_[False, held[:-1]])
        for s in starts:
            e = s
            while e + 1 < len(rows) and held[e + 1]:
                e += 1
            released = e + 1 < len(rows)
            rewarded = released and rows[e + 1, f["reward"]] > rows[e, f["reward"]] + 1e-9
            errs = []
            for i in range(s, e + 1):
                b = est_by_step.get(int(rows[i, f["step"]]) + 1)
                if b is not None:
                    a = b[4] - rows[i, f["angle"]]
                    errs.append(abs(np.degrees(np.arctan2(np.sin(a), np.cos(a)))))
            last = rows[e]
            out.append({
                "drone": int(ident), "start": int(rows[s, f["step"]]), "end": int(last[f["step"]]),
                "length": int(e - s + 1),
                "outcome": "disposed" if rewarded else ("dropped" if released else "held_at_end"),
                "end_distance_to_center": round(box_distance(last[f["x"]], last[f["y"]], box), 1),
                "end_position": (round(float(last[f["x"]]), 1), round(float(last[f["y"]]), 1)),
                "health_lost": int(rows[s, f["health"]] - last[f["health"]]),
                "heading_err_p95": round(float(np.percentile(errs, 95)), 1) if errs else None,
            })
    summary = {"center_box": [round(v, 1) for v in box], "episodes": out,
               "counts": {k: sum(e["outcome"] == k for e in out)
                          for k in ("disposed", "dropped", "held_at_end")}}
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run", type=Path)
    parser.add_argument("--map", required=True)
    args = parser.parse_args(argv)
    if not (args.run / "truth.npz").exists():
        print(f"no truth log in {args.run}", file=sys.stderr)
        return 1
    print(json.dumps(episodes(args.run, args.map), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
