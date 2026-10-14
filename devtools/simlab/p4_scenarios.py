#!/usr/bin/env python3
"""Phase 4 custom-geometry scenarios (evaluation fixtures, development only).

Each scenario is a *specification*: wall segments, solid boxes, the disposal
center, the drone start area and optional sensor-disabler zones, generated
deterministically from a template family and a preregistered generation
seed (``docs/PHASE4_PLAN.md``). None of the coordinates is taken from an
official map. From a specification this module builds

* a map class made only of public framework elements (``NormalWall``,
  ``NormalBox``, ``DisposalCenter``, ``NoGpsZone``, ``NoComZone``,
  ``ClosedPlayground``), constructed exactly like the supplied map classes,
  so that a round runs through the official launcher, physics, sensors,
  communication and score managers (the runner registers the class under
  its name for one round; nothing under ``src/swarm_rescue`` changes);
* a GL-free raster of walls and boxes for validation tests.

These are synthetic development and validation fixtures, never official
competition maps; every run on them is labelled ``map_kind: custom`` in the
ledger. The specifications, including the region annotations used to score
solid-interior inference (``annotations``), are ground truth for evaluation
tooling only and are never visible to agent code.

    python -m devtools.simlab.p4_scenarios --list
    python -m devtools.simlab.p4_scenarios --show P4PocketV1
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import sys
from typing import Any, Dict, List, Tuple

import numpy as np

W, H = 1100, 750
EDGE_X, EDGE_Y = W / 2.0, H / 2.0
WALL = 6                 # NormalWall default thickness, px

# Template families: code (seed-sequence prefix), role and step limits. The
# step and wall-clock limits are those of the supplied 5000-step maps.
FAMILIES: Dict[str, Dict[str, Any]] = {
    "pocket": {"code": 401, "role": "rescue", "drones": 10},
    "corridor": {"code": 402, "role": "rescue", "drones": 10},
    "gpsdark": {"code": 403, "role": "rescue", "drones": 10},
    "comdark": {"code": 404, "role": "rescue", "drones": 10},
    "clutter": {"code": 405, "role": "rescue", "drones": 10},
    "rooms": {"code": 451, "role": "place", "drones": 5},
    "mixed": {"code": 452, "role": "place", "drones": 5},
}
MAX_STEPS = 5000
MAX_WALLTIME = 180

# Preregistered partitions (docs/PHASE4_PLAN.md, section 4). Generation seed
# 1 of every family is the probe partition (frozen-agent validation); seed 2
# is the final partition, untouched by any agent until its stage-2 use; seed
# 3 is a reserve that is generated but never run in Phase 4 unless an
# addendum written beforehand says so.
PARTITION_BY_SEED = {1: "C-probe", 2: "C-final", 3: "C-reserve"}


def scenario_name(family: str, seed: int) -> str:
    return f"P4{family.capitalize()}V{seed}"


def parse_name(name: str) -> Tuple[str, int]:
    if not name.startswith("P4") or "V" not in name[2:]:
        raise KeyError(f"not a Phase 4 scenario name: {name!r}")
    body, _, seed = name[2:].rpartition("V")
    family = body.lower()
    if family not in FAMILIES or not seed.isdigit():
        raise KeyError(f"unknown Phase 4 scenario: {name!r}")
    return family, int(seed)


# ----------------------------------------------------------------- helpers
class _Gen:
    """Seeded jitter helpers and the geometry being built (canonical frame)."""

    def __init__(self, family: str, seed: int) -> None:
        code = FAMILIES[family]["code"]
        self.rng = np.random.default_rng(np.random.SeedSequence([code, seed]))
        self.walls: List[List[float]] = []
        self.boxes: List[List[float]] = []
        self.zones: List[Dict[str, Any]] = []
        self.annotations: List[Dict[str, Any]] = []

    def j(self, a: float) -> float:
        return float(round(self.rng.uniform(-a, a)))

    def ri(self, lo: int, hi: int) -> int:
        return int(self.rng.integers(lo, hi + 1))

    def u(self, lo: float, hi: float) -> float:
        return float(round(self.rng.uniform(lo, hi), 2))

    def hw(self, y: float, x0: float, x1: float) -> None:
        self.walls.append([float(x0), float(y), float(x1), float(y)])

    def vw(self, x: float, y0: float, y1: float) -> None:
        self.walls.append([float(x), float(y0), float(x), float(y1)])

    def box(self, x0: float, y0: float, x1: float, y1: float, label: str = "box") -> None:
        self.boxes.append([float(x0), float(y0), float(x1), float(y1)])
        self.annotations.append({"kind": label, "rect": [float(x0), float(y0), float(x1), float(y1)]})

    def room(self, x0: float, y0: float, x1: float, y1: float, door=None,
             label: str = "sealed_room") -> None:
        """Four walls on the rectangle's edges; ``door`` = (side, offset, width)
        leaves a gap on one side (``n``, ``s``, ``e``, ``w``), centred at
        ``offset`` px from the side's midpoint."""
        sides = {"n": ("h", y1, x0, x1), "s": ("h", y0, x0, x1),
                 "w": ("v", x0, y0, y1), "e": ("v", x1, y0, y1)}
        for key, (orient, c, a, b) in sides.items():
            if door is not None and door[0] == key:
                mid = (a + b) / 2.0 + door[1]
                half = door[2] / 2.0 + WALL / 2.0
                parts = [(a, mid - half), (mid + half, b)]
            else:
                parts = [(a, b)]
            for p0, p1 in parts:
                if p1 - p0 < 1.0:
                    continue
                if orient == "h":
                    self.hw(c, p0, p1)
                else:
                    self.vw(c, p0, p1)
        self.annotations.append({"kind": label, "rect": [float(x0), float(y0), float(x1), float(y1)],
                                 "door": list(door) if door is not None else None})

    def zone(self, kind: str, cx: float, cy: float, w: float, h: float) -> None:
        self.zones.append({"type": kind, "x": float(cx), "y": float(cy),
                           "width": float(w), "height": float(h)})


def _dc(x: float, y: float, w: float, h: float) -> Dict[str, float]:
    return {"x": float(x), "y": float(y), "width": float(int(w)), "height": float(int(h))}


def _faces(dc: Dict[str, float]) -> Tuple[float, float, float, float]:
    return (dc["x"] - dc["width"] / 2.0, dc["y"] - dc["height"] / 2.0,
            dc["x"] + dc["width"] / 2.0, dc["y"] + dc["height"] / 2.0)


# --------------------------------------------------------------- templates
# Every template builds its geometry in a canonical frame; the generation
# seed then picks a mirror (x and/or y) and the jitter, so the two variants
# of a family differ in orientation, sizes and positions.

def _pocket(g: _Gen) -> Dict[str, Any]:
    """Disposal center walled on three sides (2-3 px gaps), the open face
    away from the start area, behind a long divider with two passages: the
    drones cannot see the center at the start and every delivery needs a
    long detour."""
    dc = _dc(230 + g.j(25), 20 + g.j(40), g.ri(170, 220), g.ri(90, 120))
    xw, ys, xe, yn = _faces(dc)
    gap = g.u(1.6, 3.0)
    o = gap + WALL / 2.0
    lip = g.ri(0, 25)
    back = xw - o
    g.hw(yn + o, back - 3, xe + lip)
    g.hw(ys - o, back - 3, xe + lip)
    g.vw(back, ys - o, yn + o)
    gap_n, gap_s = g.ri(72, 95), g.ri(110, 160)
    g.vw(back, yn + o, EDGE_Y - gap_n)
    g.vw(back, -EDGE_Y + gap_s, ys - o)
    g.annotations.append({"kind": "dc_pocket", "open": "e", "gap_px": gap})
    # West half: a room with a door, a box and a partition from the frame.
    g.room(-520 + g.j(10), 150 + g.j(15), -330 + g.j(15), 340, door=("e", g.j(30), 90),
           label="door_room")
    bx, by = -330 + g.j(25), -230 + g.j(25)
    g.box(bx, by, bx + g.ri(90, 120), by + g.ri(70, 100))
    g.hw(-40 + g.j(20), -EDGE_X, -390 + g.j(30))
    start = [back - 230 + g.j(20), dc["y"] + g.j(50)]
    return {"dc": dc, "start": start}


def _corridor(g: _Gen) -> Dict[str, Any]:
    """Disposal center walled on west, south and east (2-3 px gaps); its
    north face is reachable only through a narrow corridor (70-90 px) that
    covers part of the face. The start area lies in the opposite corner and
    a long wall forces a detour."""
    dc = _dc(-330 + g.j(30), -245 + g.j(20), g.ri(150, 190), g.ri(80, 100))
    xw, ys, xe, yn = _faces(dc)
    gap = g.u(1.6, 3.0)
    o = gap + WALL / 2.0
    g.vw(xw - o, ys - o - 3, yn + o)
    g.hw(ys - o, xw - o - 3, xe + o + 3)
    g.vw(xe + o, ys - o - 3, yn + o)
    cw = g.ri(70, 90)
    cx = min(max(dc["x"] + g.j(25), xw + 12 + cw / 2.0), xe - 12 - cw / 2.0)
    length = g.ri(200, 280)
    g.vw(cx - cw / 2.0 - WALL / 2.0, yn + o, yn + length)
    g.vw(cx + cw / 2.0 + WALL / 2.0, yn + o, yn + length)
    g.hw(yn + o, xw - o - 3, cx - cw / 2.0 - WALL / 2.0)
    g.hw(yn + o, cx + cw / 2.0 + WALL / 2.0, xe + o + 3)
    g.annotations.append({"kind": "dc_corridor", "open": "n", "gap_px": gap,
                          "corridor_width": cw, "corridor_length": length})
    g.hw(30 + g.j(30), -120 + g.j(40), EDGE_X)
    bx, by = 60 + g.j(30), -230 + g.j(30)
    g.box(bx, by, bx + g.ri(90, 130), by + g.ri(70, 100))
    start = [330 + g.j(30), 240 + g.j(30)]
    return {"dc": dc, "start": start}


def _rooms_layout(g: _Gen, west_partition: bool = True) -> None:
    """Shared interior of the localization and communication families: two
    rooms with doors and partitions, similar in density to the supplied maps."""
    g.room(-520 + g.j(10), 120 + g.j(20), -300 + g.j(20), 340, door=("e", g.j(30), 90),
           label="door_room")
    g.room(300 + g.j(20), 120 + g.j(20), 520 + g.j(10), 340, door=("w", g.j(30), 90),
           label="door_room")
    if west_partition:
        g.hw(-20 + g.j(20), -EDGE_X, -280 + g.j(30))
    g.hw(-20 + g.j(20), 280 + g.j(30), EDGE_X)


def _gpsdark(g: _Gen) -> Dict[str, Any]:
    """Supplied-map-like layout (center visible from the start) with GPS and
    compass disabled around the disposal center and in one room: carriers
    approach and deliver on odometry alone."""
    dc = _dc(g.j(60), -200 + g.j(30), g.ri(160, 210), g.ri(80, 110))
    _rooms_layout(g)
    start = [dc["x"] + g.j(30), dc["y"] + dc["height"] / 2.0 + 130 + g.j(15)]
    g.zone("no_gps", dc["x"], dc["y"], dc["width"] + 260, dc["height"] + 200)
    g.zone("no_gps", 410, 230, 260, 260)
    return {"dc": dc, "start": start}


def _comdark(g: _Gen) -> Dict[str, Any]:
    """Supplied-map-like layout with communication disabled over the eastern
    57 % of the arena, which holds most of the free space and the bombs."""
    dc = _dc(-300 + g.j(30), -220 + g.j(25), g.ri(160, 210), g.ri(80, 110))
    _rooms_layout(g, west_partition=False)
    start = [dc["x"] + g.j(30), dc["y"] + dc["height"] / 2.0 + 130 + g.j(15)]
    x0 = -80 + g.j(20)
    g.zone("no_com", (x0 + EDGE_X) / 2.0, 0.0, EDGE_X - x0, H)
    return {"dc": dc, "start": start}


def _clutter(g: _Gen) -> Dict[str, Any]:
    """Start in one corner, the disposal center in the opposite one with a
    solid box 2-3 px off its north face and a wall 2-3 px off its east face,
    and a field of boxes with 70-110 px passages in between."""
    dc = _dc(360 + g.j(20), -230 + g.j(20), g.ri(150, 190), g.ri(80, 100))
    xw, ys, xe, yn = _faces(dc)
    gap = g.u(1.6, 3.0)
    g.box(xw - 20, yn + gap, xe + 20, yn + gap + g.ri(60, 90), label="box_at_dc")
    g.vw(xe + gap + WALL / 2.0, ys - 60, yn + 100)
    g.annotations.append({"kind": "dc_two_sided", "gap_px": gap})
    # Box field in x -330..215, y -120..215: passages of 70-110 px between
    # columns and rows, clear of the start grid and of the center's open
    # west and south faces.
    x = -330.0
    for col in range(3):
        w = g.ri(80, 110)
        y = 200.0
        for row in range(2):
            h = g.ri(80, 110)
            g.box(x, y - h, x + w, y)
            y -= h + g.ri(70, 100)
        x += w + g.ri(70, 100)
    start = [-450 + g.j(15), 290 + g.j(15)]
    return {"dc": dc, "start": start}


def _rooms(g: _Gen) -> Dict[str, Any]:
    """Red: sealed empty rooms, a room with a narrow door, a room with a
    normal door and solid boxes of similar sizes (an outline-only map cannot
    tell a sealed room from a box)."""
    dc = _dc(-380 + g.j(20), 230 + g.j(20), g.ri(160, 200), g.ri(80, 100))
    start = [dc["x"] + g.j(20), dc["y"] - dc["height"] / 2.0 - 120 + g.j(10)]

    def r(x0, y0, x1, y1, s=18):
        return x0 + g.j(s), y0 + g.j(s), x1 + g.j(s), y1 + g.j(s)

    g.room(*r(-150, 110, 10, 250), label="sealed_room")
    g.room(*r(330, -330, 450, -130), label="sealed_room")
    g.room(*r(150, 140, 330, 290), door=("n", g.j(20), g.ri(36, 44)), label="narrow_door_room")
    g.room(*r(-500, -330, -300, -170), door=("n", g.j(20), 100), label="door_room")
    g.box(*r(-120, -300, 30, -170))
    g.box(*r(120, -120, 230, 60))
    g.vw(-230 + g.j(15), -60 + g.j(15), 60 + g.j(15))
    return {"dc": dc, "start": start}


def _mixed(g: _Gen) -> Dict[str, Any]:
    """Red: a box sharing a wall with a sealed room, a U-shaped (incomplete)
    outline, an L-shaped wall, a hollow room containing a box, a box against
    the frame, and a disposal center walled on three sides."""
    dc = _dc(380 + g.j(20), 210 + g.j(20), g.ri(150, 180), g.ri(80, 100))
    xw, ys, xe, yn = _faces(dc)
    gap = g.u(1.6, 3.0)
    o = gap + WALL / 2.0
    g.hw(yn + o, xw - 20, xe + o + 3)
    g.vw(xe + o, ys - o - 3, yn + o + 3)
    g.hw(ys - o, xw - 20, xe + o + 3)
    g.annotations.append({"kind": "dc_pocket", "open": "w", "gap_px": gap})
    start = [xw - 150 + g.j(15), dc["y"] + g.j(20)]
    bx, by = -200 + g.j(15), -300 + g.j(15)
    g.box(bx, by, bx + 120, by + 120, label="box_with_room")
    g.room(bx + 120, by, bx + 260, by + 120, label="sealed_room")
    ux, uy = 120 + g.j(20), -180 + g.j(20)
    g.vw(ux, uy, uy + 140)
    g.hw(uy + 140, ux, ux + 160)
    g.vw(ux + 160, uy, uy + 140)
    g.annotations.append({"kind": "u_shape", "rect": [ux, uy, ux + 160, uy + 140]})
    lx, ly = -460 + g.j(20), -60 + g.j(20)
    g.hw(ly, lx, lx + 180)
    g.vw(lx, ly, ly - 160)
    g.room(-520 + g.j(10), 130 + g.j(15), -280 + g.j(15), 330 + g.j(10),
           door=("s", g.j(30), 90), label="door_room")
    ix, iy = -440 + g.j(10), 190 + g.j(10)
    g.box(ix, iy, ix + 80, iy + 80, label="box_in_room")
    g.box(400 + g.j(15), -EDGE_Y, 520 + g.j(15), -260 + g.j(15), label="box_at_frame")
    return {"dc": dc, "start": start}


TEMPLATES = {"pocket": _pocket, "corridor": _corridor, "gpsdark": _gpsdark,
             "comdark": _comdark, "clutter": _clutter, "rooms": _rooms, "mixed": _mixed}


# ------------------------------------------------------------ specification
def _mirror(spec: Dict[str, Any], mx: int, my: int) -> None:
    for w in spec["walls"]:
        w[0], w[1], w[2], w[3] = mx * w[0], my * w[1], mx * w[2], my * w[3]

    def rect(r):
        x0, y0, x1, y1 = mx * r[0], my * r[1], mx * r[2], my * r[3]
        return [min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)]

    spec["boxes"] = [rect(b) for b in spec["boxes"]]
    flip = {"e": "w" if mx < 0 else "e", "w": "e" if mx < 0 else "w",
            "n": "s" if my < 0 else "n", "s": "n" if my < 0 else "s"}
    for a in spec["annotations"]:
        if "rect" in a:
            a["rect"] = rect(a["rect"])
        if a.get("open") in flip:
            a["open"] = flip[a["open"]]
        if a.get("door"):
            a["door"][0] = flip[a["door"][0]]
    dc = spec["disposal_center"]
    dc["x"], dc["y"] = mx * dc["x"], my * dc["y"]
    spec["start_area"] = [mx * spec["start_area"][0], my * spec["start_area"][1]]
    for z in spec["zones"]:
        z["x"], z["y"] = mx * z["x"], my * z["y"]
    spec["mirror"] = [mx, my]


def _clean(value: Any) -> Any:
    if isinstance(value, float):
        out = round(value, 2)
        return 0.0 if out == 0 else out
    if isinstance(value, list):
        return [_clean(v) for v in value]
    if isinstance(value, tuple):
        return [_clean(v) for v in value]
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()}
    return value


def build_spec(family: str, seed: int) -> Dict[str, Any]:
    """The scenario specification for ``family`` and generation ``seed``."""
    if family not in FAMILIES:
        raise KeyError(family)
    g = _Gen(family, seed)
    out = TEMPLATES[family](g)
    mx = 1 if g.rng.random() < 0.5 else -1
    my = 1 if g.rng.random() < 0.5 else -1
    meta = FAMILIES[family]
    spec = {
        "name": scenario_name(family, seed), "family": family, "generation_seed": seed,
        "partition": PARTITION_BY_SEED.get(seed, "unregistered"),
        "role": meta["role"], "drones": meta["drones"],
        "size_area": [W, H], "max_timestep_limit": MAX_STEPS,
        "max_walltime_limit": MAX_WALLTIME,
        "walls": g.walls, "boxes": g.boxes, "zones": g.zones,
        "disposal_center": out["dc"], "start_area": [float(v) for v in out["start"]],
        "annotations": g.annotations, "synthetic": True,
    }
    _mirror(spec, mx, my)
    return _clean(spec)


def spec_by_name(name: str) -> Dict[str, Any]:
    family, seed = parse_name(name)
    return build_spec(family, seed)


def spec_sha256(spec: Dict[str, Any]) -> str:
    blob = json.dumps(spec, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


def registered_names(seeds=(1, 2, 3)) -> List[str]:
    return [scenario_name(f, s) for f in FAMILIES for s in seeds]


# ------------------------------------------------------------- GL-free raster
def raster(spec: Dict[str, Any], include_frame: bool = True) -> Dict[str, np.ndarray]:
    """Pixel rasters (row = H/2 - y, col = x + W/2) of walls, boxes, frame and
    disposal center, for validation only (the official grids come from the
    framework's own rasterization, ``fixtures.extract_class``)."""
    w, h = spec["size_area"]
    walls = np.zeros((h, w), np.uint8)

    def fill(x0, y0, x1, y1, arr):
        c0, c1 = int(math.floor(x0 + w / 2.0)), int(math.ceil(x1 + w / 2.0))
        r0, r1 = int(math.floor(h / 2.0 - y1)), int(math.ceil(h / 2.0 - y0))
        arr[max(r0, 0):max(min(r1, h), 0), max(c0, 0):max(min(c1, w), 0)] = 1

    for x0, y0, x1, y1 in spec["walls"]:
        half = WALL / 2.0
        if abs(y0 - y1) < 1e-9:
            fill(min(x0, x1), y0 - half, max(x0, x1), y0 + half, walls)
        else:
            fill(x0 - half, min(y0, y1), x0 + half, max(y0, y1), walls)
    for x0, y0, x1, y1 in spec["boxes"]:
        fill(x0, y0, x1, y1, walls)
    frame = np.zeros_like(walls)
    if include_frame:
        frame[:WALL, :] = frame[-WALL:, :] = 1
        frame[:, :WALL] = frame[:, -WALL:] = 1
    dc = np.zeros_like(walls)
    x0, y0, x1, y1 = _faces(spec["disposal_center"])
    fill(x0, y0, x1, y1, dc)
    return {"walls": walls | frame, "frame": frame, "dc": dc}


def free_region(spec: Dict[str, Any], clearance: float = 17.0) -> np.ndarray:
    """Pixels a drone centre can occupy, connected to the start area."""
    import cv2
    r = raster(spec)
    solid = (r["walls"] | r["dc"]).astype(np.uint8)
    from .stable_cv import distance_transform
    dist = distance_transform(1 - solid)
    ok = (dist >= clearance).astype(np.uint8)
    _, labels = cv2.connectedComponents(ok)
    w, h = spec["size_area"]
    sx, sy = spec["start_area"]
    lab = labels[int(round(h / 2.0 - sy)), int(round(sx + w / 2.0))]
    if lab == 0:
        return np.zeros_like(ok, dtype=bool)
    return labels == lab


# ---------------------------------------------------------------- map class
def make_map_class(spec: Dict[str, Any]):
    """A map class for ``spec`` built like the supplied ``Map0x`` classes."""
    from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract  # noqa: F401
    from swarm_rescue.simulation.elements.disposal_center import DisposalCenter
    from swarm_rescue.simulation.elements.normal_wall import NormalBox, NormalWall
    from swarm_rescue.simulation.elements.sensor_disablers import NoComZone, NoGpsZone
    from swarm_rescue.simulation.gui_map.closed_playground import ClosedPlayground
    from swarm_rescue.simulation.gui_map.map_abstract import MapAbstract
    from swarm_rescue.simulation.utils.misc_data import MiscData

    frozen = json.loads(json.dumps(spec))

    class CustomMap(MapAbstract):
        SPEC = frozen

        def __init__(self, drone_type, zones_config=(), number_drones=None):
            super().__init__(drone_type, zones_config, number_drones=number_drones)
            s = self.SPEC
            self._max_timestep_limit = int(s["max_timestep_limit"])
            self._max_walltime_limit = int(s["max_walltime_limit"])
            self._size_area = (int(s["size_area"][0]), int(s["size_area"][1]))
            self._start_area_drones = tuple(s["start_area"])
            dc = s["disposal_center"]
            self._disposal_center = DisposalCenter(size=(int(dc["width"]), int(dc["height"])))
            self._disposal_center_pos = ((dc["x"], dc["y"]), 0)
            self._bombs_pos = []
            self._bombs_path = []
            self._number_bombs = 0
            self._bombs = []
            self._number_drones = (self._number_drones_override
                                   if self._number_drones_override is not None
                                   else int(s["drones"]))
            # Drone grid and random headings exactly as in the supplied maps.
            nb_per_side = math.ceil(math.sqrt(float(self._number_drones)))
            sx = self._start_area_drones[0] - (nb_per_side - 1) * 0.5 * 40.0
            sy = self._start_area_drones[1] - (nb_per_side - 1) * 0.5 * 40.0
            self._drones_pos = []
            for i in range(self._number_drones):
                x = sx + (float(i) % nb_per_side) * 40.0
                y = sy + math.floor(float(i) / nb_per_side) * 40.0
                self._drones_pos.append(((x, y), random.uniform(-math.pi, math.pi)))
            self._drones = []
            self._playground = ClosedPlayground(size=self._size_area)
            self._playground.add(self._disposal_center, self._disposal_center_pos)
            for x0, y0, x1, y1 in s["walls"]:
                wall = NormalWall(pos_start=(x0, y0), pos_end=(x1, y1))
                self._playground.add(wall, wall.wall_coordinates)
            for x0, y0, x1, y1 in s["boxes"]:
                box = NormalBox(up_left_point=(x0, y1), width=x1 - x0, height=y1 - y0)
                self._playground.add(box, box.wall_coordinates)
            for z in s["zones"]:
                cls = NoGpsZone if z["type"] == "no_gps" else NoComZone
                zone = cls(size=(int(z["width"]), int(z["height"])))
                self._playground.add(zone, ((z["x"], z["y"]), 0))
            self._explored_map.initialize_walls(self._playground)
            misc_data = MiscData(size_area=self._size_area,
                                 number_drones=self._number_drones,
                                 max_timestep_limit=self._max_timestep_limit,
                                 max_walltime_limit=self._max_walltime_limit)
            for i in range(self._number_drones):
                drone = drone_type(identifier=i, misc_data=misc_data)
                self._drones.append(drone)
                self._playground.add(drone, self._drones_pos[i])

    CustomMap.__name__ = spec["name"]
    CustomMap.__qualname__ = spec["name"]
    return CustomMap


def register(launcher_module, name: str):
    """Make ``name`` resolvable by the launcher for this process (harness only).

    The launcher looks map classes up by name in its module namespace; this
    adds one evaluation-only class there. Official map classes and every
    framework function are left untouched, and agent code never calls this.
    """
    if name in ("Map01", "Map02", "Map03", "Map04", "Map05"):
        raise ValueError("official map names cannot be registered as custom maps")
    spec = spec_by_name(name)
    cls = make_map_class(spec)
    setattr(launcher_module, name, cls)
    return cls, spec


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--show", default=None)
    args = parser.parse_args(argv)
    if args.show:
        spec = spec_by_name(args.show)
        print(json.dumps({"sha256": spec_sha256(spec), **spec}, indent=1))
        return 0
    for name in registered_names():
        spec = spec_by_name(name)
        print(f"{name}\t{spec['partition']}\t{spec['role']}\t{spec_sha256(spec)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
