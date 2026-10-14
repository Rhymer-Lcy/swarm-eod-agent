#!/usr/bin/env python3
"""Synthetic delivery scenarios for the offline approximation (development only).

These fixtures reproduce the *mechanism* of the Phase 2 Map03 delivery
failure on invented geometry: a disposal center whose two faces lie a few
pixels from long walls, so that the space behind those walls is close to the
center in Euclidean distance but far from it along any free path. None of the
coordinates is taken from an official map; they are ground truth for the
offline simulator only and are never visible to agent code.

    python -m devtools.simlab.delivery_fixtures --agent current --scenario corner --seed 1
    python -m devtools.simlab.delivery_fixtures --agent p2_540054b --scenario corner --seed 1

Scenarios (world frame, origin at the arena center, y up, 700 x 500 px):

* ``open``: the center stands free; every face is accessible (control).
* ``pocket`` (Phase 4): walls 2 px from the west, south and east faces; only
  the north face is open.
* ``narrow`` (Phase 4): walls 2 px from the west and south faces; the north
  face lies behind a wall 30 px away with a 30 px opening (narrower than a
  carrying drone) in front of its middle; the east face is open but cannot
  be seen from the start area.
* ``corner``: a 6 px thick horizontal wall 2 px below the center's south
  face and a vertical one 2 px west of its west face (the gap on Map03 is
  about 1.5 px), both extended well beyond the center, so the south and west
  faces are unreachable. The space south of the horizontal wall and west of
  the vertical wall connects to the open north side only around the far
  ends of the walls.

Bomb layouts put bombs behind the walled faces (``corner``) or around the
center (``open``); drones start north of the center, as on the supplied maps.
"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from typing import Dict, List, Tuple

import numpy as np

W, H = 700, 500
DC = {"x": -60.0, "y": 0.0, "width": 160.0, "height": 80.0}   # x -140..20, y -40..40
WALL = 6

SCENARIOS = ("open", "corner", "pocket", "narrow")
NARROW_GAP = 30.0          # px; narrower than a carrying drone (Phase 4, Risk A)

BOMBS: Dict[str, List[Tuple[float, float]]] = {
    # Behind the south wall (below the center), behind the west wall, and
    # one north of the center as a reachable control.
    "corner": [(-60.0, -110.0), (0.0, -150.0), (-120.0, -160.0),
               (-215.0, 10.0), (-230.0, -80.0), (60.0, 150.0)],
    "open": [(-60.0, -110.0), (0.0, -150.0), (-120.0, -160.0),
             (-215.0, 10.0), (-230.0, -80.0), (60.0, 150.0)],
    # Phase 4 (Risk A): bombs south of the pocket and west of it.
    "pocket": [(-60.0, -130.0), (60.0, -150.0), (-220.0, -60.0), (120.0, 140.0)],
    "narrow": [(-60.0, -130.0), (120.0, -120.0), (-220.0, -60.0), (60.0, 160.0)],
}


def _px_row(y: float) -> int:
    return int(round(H / 2.0 - y))


def _px_col(x: float) -> int:
    return int(round(x + W / 2.0))


def _hwall(walls: np.ndarray, y: float, x0: float, x1: float) -> None:
    r = _px_row(y)
    walls[r - WALL // 2:r + WALL // 2, _px_col(x0):_px_col(x1)] = 1


def _vwall(walls: np.ndarray, x: float, y0: float, y1: float) -> None:
    c = _px_col(x)
    walls[_px_row(y1):_px_row(y0), c - WALL // 2:c + WALL // 2] = 1


def fixture(scenario: str, max_steps: int = 3000) -> dict:
    """Walls-only fixture in the format of ``devtools.simlab.fixtures``."""
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario {scenario!r}")
    walls = np.zeros((H, W), np.uint8)
    walls[:WALL, :] = walls[-WALL:, :] = 1
    walls[:, :WALL] = walls[:, -WALL:] = 1
    if scenario == "corner":
        x0 = DC["x"] - DC["width"] / 2      # west face, -140
        y0 = DC["y"] - DC["height"] / 2     # south face, -40
        _hwall(walls, y0 - 5.0, x0 - 5.0, 170.0)    # 2 px gap below the south face
        _vwall(walls, x0 - 5.0, y0 - 5.0, 170.0)    # 2 px gap west of the west face
    elif scenario in ("pocket", "narrow"):
        x0 = DC["x"] - DC["width"] / 2      # west face, -140
        x1 = DC["x"] + DC["width"] / 2      # east face, 20
        y0 = DC["y"] - DC["height"] / 2     # south face, -40
        y1 = DC["y"] + DC["height"] / 2     # north face, 40
        _vwall(walls, x0 - 5.0, y0 - 70.0, y1 + 30.0)   # west, 2 px gap
        _hwall(walls, y0 - 5.0, x0 - 70.0, x1 + 70.0)   # south, 2 px gap
        if scenario == "pocket":
            _vwall(walls, x1 + 5.0, y0 - 70.0, y1 + 30.0)   # east, 2 px gap: open north only
        else:
            # North closed 30 px above the face except a NARROW_GAP opening
            # in front of its middle; the east face stays open.
            yn = y1 + 30.0
            mid = DC["x"]
            _hwall(walls, yn, x0 - 8.0, mid - NARROW_GAP / 2 - WALL / 2)
            _hwall(walls, yn, mid + NARROW_GAP / 2 + WALL / 2, x1 + 30.0)
    boundary = np.zeros_like(walls)
    boundary[:WALL, :] = boundary[-WALL:, :] = 1
    boundary[:, :WALL] = boundary[:, -WALL:] = 1
    return {"map": f"Delivery-{scenario}", "walls": walls, "boundary": boundary,
            "obstacle_render": walls, "starts": np.zeros((0, 2)),
            "size_area": [W, H], "max_timestep_limit": max_steps,
            "max_walltime_limit": 600, "disposal_center": dict(DC),
            "start_area": [-60.0, 120.0], "default_drones": 4}


def brain_class(agent: str):
    package = "swarm_rescue.solutions.eod" if agent == "current" else \
        f"devtools.comparators.{agent}"
    return importlib.import_module(f"{package}.blue").RescueBrain


def run(agent: str, scenario: str, seed: int, drones: int = 4, max_steps: int = 3000,
        bombs=None) -> dict:
    """Run one offline delivery scenario; returns the world summary and brains."""
    from .offline_sim import OfflineWorld
    cls = brain_class(agent)
    fx = fixture(scenario, max_steps)
    layout = BOMBS[scenario] if bombs is None else bombs
    brains = []

    def factory(i: int):
        brains.append(cls(i, drones, (W, H), max_steps))
        return brains[-1]

    world = OfflineWorld(fx, "rescue", drones, factory, seed=seed, bombs=layout,
                         max_steps=max_steps, carry_ratio_sigma=0.36)
    out = world.run()
    out["carrying_at_end"] = sum(d.carrying is not None for d in world.drones)
    out["diag"] = {k: sum(int(b.diag.get(k, 0)) for b in brains)
                   for k in sorted(set().union(*(b.diag for b in brains)))}
    return {"summary": out, "world": world, "brains": brains}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--agent", default="current")
    parser.add_argument("--scenario", choices=SCENARIOS, default="corner")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--drones", type=int, default=4)
    parser.add_argument("--steps", type=int, default=3000)
    args = parser.parse_args(argv)
    res = run(args.agent, args.scenario, args.seed, args.drones, args.steps)
    out = dict(res["summary"])
    out.update({"agent": args.agent, "scenario": args.scenario, "seed": args.seed})
    print(json.dumps(out, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
