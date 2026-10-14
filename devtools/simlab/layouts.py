#!/usr/bin/env python3
"""Synthetic bomb layouts for blue-team tests (development only).

Layouts are sampled from the walls-only truth fixtures: bomb centres lie in
free space at least ``clearance`` px from any wall, outside the disposal
center and the start area, inside the free region connected to the start
area, and at least ``spacing`` px apart. Profiles:

* ``uniform``: uniform over the admissible region;
* ``far``: probability proportional to distance from the disposal center,
  a proxy for layouts that are slow to clear;
* ``wall`` (Phase 4): probability decaying with distance from the nearest
  wall, as a red team that maximizes wall coverage leaves its bombs;
* ``cluster`` (Phase 4): bombs within 130 px of three random centres
  (spacing 45 px), as several red drones placing in one area leave them.

Fixtures of Phase 4 custom scenarios (``p4_scenarios``) are accepted by
name like the official maps.

Every file is labelled ``"synthetic": true`` with its generator settings.
Synthetic layouts are not competition layouts; results on them are reported
separately from runs on layouts produced by red agents.

    python -m devtools.simlab.layouts --map Map01 --bombs 10 --seed 1 --profile uniform
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

from .fixtures import load_fixture
from .stable_cv import distance_transform

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = REPO_ROOT / "_local" / "layouts"


def admissible_mask(fx: dict, clearance: float = 26.0) -> np.ndarray:
    walls = fx["walls"].astype(np.uint8)
    h, w = walls.shape
    dist = distance_transform((1 - walls).astype(np.uint8))
    ok = dist >= clearance
    dc = fx["disposal_center"]
    c0 = int(dc["x"] - dc["width"] / 2 + w / 2) - 45
    c1 = int(dc["x"] + dc["width"] / 2 + w / 2) + 45
    r0 = int(h / 2 - dc["y"] - dc["height"] / 2) - 45
    r1 = int(h / 2 - dc["y"] + dc["height"] / 2) + 45
    ok[max(r0, 0):r1, max(c0, 0):c1] = False
    for sx, sy in fx["starts"]:
        cv2.circle(ok.view(np.uint8), (int(sx + w / 2), int(h / 2 - sy)), 70, 0, -1)
    # Region reachable by a drone from the start area.
    passable = (dist >= 17).astype(np.uint8)
    n, labels = cv2.connectedComponents(passable)
    sx, sy = fx["start_area"]
    seed_label = labels[int(h / 2 - sy), int(sx + w / 2)]
    return ok & (labels == seed_label)


def sample_layout(fx: dict, n_bombs: int, seed: int, profile: str = "uniform",
                  spacing: float = 60.0) -> dict:
    rng = np.random.default_rng(seed)
    mask = admissible_mask(fx)
    h, w = mask.shape
    rows, cols = np.nonzero(mask)
    xs = cols - w / 2.0
    ys = h / 2.0 - rows
    if profile == "far":
        dc = fx["disposal_center"]
        weight = np.hypot(xs - dc["x"], ys - dc["y"])
        weight = weight / weight.sum()
    elif profile == "wall":
        # Close to walls, as a red team maximizing wall coverage leaves them.
        dist = distance_transform((1 - fx["walls"].astype(np.uint8)).astype(np.uint8))
        weight = np.exp(-(dist[rows, cols] - 26.0) / 20.0)
        weight = weight / weight.sum()
    elif profile == "cluster":
        # Three cluster centres; bombs within 130 px of one of them.
        centres = [(float(xs[k]), float(ys[k])) for k in rng.choice(len(xs), 3, replace=False)]
        near = np.zeros(len(xs), dtype=bool)
        for cx, cy in centres:
            near |= np.hypot(xs - cx, ys - cy) < 130.0
        weight = near / near.sum()
        spacing = min(spacing, 45.0)
    else:
        weight = None
    chosen = []
    for _ in range(20000):
        if len(chosen) == n_bombs:
            break
        i = rng.choice(len(xs), p=weight)
        x, y = float(xs[i]), float(ys[i])
        if all(math.hypot(x - a, y - b) >= spacing for a, b in chosen):
            chosen.append((x, y))
    if len(chosen) < n_bombs:
        raise RuntimeError(f"could only place {len(chosen)} of {n_bombs} bombs")
    return {
        "map_name": fx["map"],
        "zones_config": [],
        "bombs": [{"x": round(x, 1), "y": round(y, 1), "theta": 0.0} for x, y in chosen],
        "synthetic": True,
        "generator": {"tool": "devtools.simlab.layouts", "seed": seed,
                      "profile": profile, "spacing": spacing, "n_bombs": n_bombs},
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", required=True)
    parser.add_argument("--bombs", type=int, default=10)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--profile", choices=["uniform", "far", "wall", "cluster"],
                        default="uniform")
    parser.add_argument("--out", default=None)
    args = parser.parse_args(argv)
    fx = load_fixture(args.map)
    doc = sample_layout(fx, args.bombs, args.seed, args.profile)
    out = Path(args.out) if args.out else DEFAULT_OUT / (
        f"synthetic_{args.map}_{args.profile}_n{args.bombs}_s{args.seed}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes((json.dumps(doc, indent=1) + "\n").encode("utf-8"))
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
