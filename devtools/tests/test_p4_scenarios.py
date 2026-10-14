"""Phase 4 custom-geometry scenarios: specification checks (no OpenGL).

The specifications are generated from preregistered seeds; these tests pin
that generation is deterministic, that each scenario is runnable (drone
start positions clear of walls, the disposal center reachable from the
start area) and that each family has the property it is meant to exercise
(for example, a center that cannot be seen from the start area). The
framework's own rasterization of the same scenarios is compared with the
GL-free raster on the remote server (``test_p4_scenarios_gl.py``).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.simlab import p4_scenarios as S  # noqa: E402

NAMES = S.registered_names()


def _starts(spec):
    n = spec["drones"]
    side = math.ceil(math.sqrt(n))
    sx = spec["start_area"][0] - (side - 1) * 20.0
    sy = spec["start_area"][1] - (side - 1) * 20.0
    return [(sx + (i % side) * 40.0, sy + (i // side) * 40.0) for i in range(n)]


def _px(spec, x, y):
    w, h = spec["size_area"]
    return int(round(h / 2.0 - y)), int(round(x + w / 2.0))


def _blocked(raster, spec, x0, y0, x1, y1) -> bool:
    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 2
    for t in np.linspace(0.0, 1.0, n):
        r, c = _px(spec, x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        if raster[r, c]:
            return True
    return False


def _face_points(spec, step=10.0):
    dc = spec["disposal_center"]
    x0, x1 = dc["x"] - dc["width"] / 2 + 1, dc["x"] + dc["width"] / 2 - 1
    y0, y1 = dc["y"] - dc["height"] / 2 + 1, dc["y"] + dc["height"] / 2 - 1
    pts = [(x, y0) for x in np.arange(x0, x1, step)] + [(x, y1) for x in np.arange(x0, x1, step)]
    pts += [(x0, y) for y in np.arange(y0, y1, step)] + [(x1, y) for y in np.arange(y0, y1, step)]
    return pts


def test_names_parse_and_official_names_are_refused():
    assert S.parse_name("P4PocketV1") == ("pocket", 1)
    with pytest.raises(KeyError):
        S.parse_name("Map03")
    with pytest.raises(ValueError):
        S.register(object(), "Map03")


def test_generation_is_deterministic_and_variants_differ():
    for family in S.FAMILIES:
        a, b = S.build_spec(family, 1), S.build_spec(family, 1)
        assert S.spec_sha256(a) == S.spec_sha256(b)
        assert S.spec_sha256(S.build_spec(family, 2)) != S.spec_sha256(a)
    assert len({S.spec_sha256(S.spec_by_name(n)) for n in NAMES}) == len(NAMES)


@pytest.mark.parametrize("name", NAMES)
def test_scenario_is_runnable(name):
    spec = S.spec_by_name(name)
    w, h = spec["size_area"]
    r = S.raster(spec)
    solid = (r["walls"] | r["dc"]).astype(np.uint8)
    dist = cv2.distanceTransform(1 - solid, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    for x, y in _starts(spec):
        assert abs(x) < w / 2 - 20 and abs(y) < h / 2 - 20, (x, y)
        assert dist[_px(spec, x, y)] >= 20.0, (name, x, y)
    free = S.free_region(spec)
    assert free.sum() > 0.3 * w * h, name
    # The center can be reached: free, reachable pixels within 45 px of it.
    near = cv2.dilate(r["dc"], np.ones((91, 91), np.uint8)) > 0
    assert (free & near).sum() > 500, name
    for z in spec["zones"]:
        assert abs(z["x"]) + z["width"] / 2 <= w / 2 + 1
        assert abs(z["y"]) + z["height"] / 2 <= h / 2 + 1


@pytest.mark.parametrize("seed", [1, 2, 3])
@pytest.mark.parametrize("family", ["pocket", "corridor", "clutter"])
def test_center_is_hidden_from_the_start_area(family, seed):
    """These families start the drones out of sight of the center."""
    spec = S.build_spec(family, seed)
    walls = S.raster(spec)["walls"]
    for sx, sy in _starts(spec):
        for px, py in _face_points(spec):
            if math.hypot(px - sx, py - sy) <= 200.0:     # semantic range
                assert _blocked(walls, spec, sx, sy, px, py), (family, seed, sx, sy, px, py)


@pytest.mark.parametrize("seed", [1, 2, 3])
@pytest.mark.parametrize("family", ["gpsdark", "comdark", "rooms", "mixed"])
def test_center_is_visible_from_the_start_area(family, seed):
    spec = S.build_spec(family, seed)
    walls = S.raster(spec)["walls"]
    seen = 0
    for sx, sy in _starts(spec):
        for px, py in _face_points(spec):
            if math.hypot(px - sx, py - sy) <= 200.0 and not _blocked(walls, spec, sx, sy, px, py):
                seen += 1
                break
    assert seen >= spec["drones"] // 2, (family, seed, seen)


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_pocket_and_corridor_close_all_but_one_face(seed):
    for family in ("pocket", "corridor"):
        spec = S.build_spec(family, seed)
        walls = S.raster(spec)["walls"]
        dc = spec["disposal_center"]
        closed = 0
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            x = dc["x"] + dx * (dc["width"] / 2 + 5)
            y = dc["y"] + dy * (dc["height"] / 2 + 5)
            r, c = _px(spec, x, y)
            closed += bool(walls[r, c])
        # Three faces walled 2-3 px away; the corridor's opening lies in front
        # of the remaining face's middle, the pocket's face is open.
        assert closed == 3, (family, seed, closed)


def test_red_families_carry_the_truth_annotations():
    for seed in (1, 2):
        rooms = S.build_spec("rooms", seed)
        kinds = [a["kind"] for a in rooms["annotations"]]
        assert kinds.count("sealed_room") == 2 and kinds.count("box") == 2
        assert "narrow_door_room" in kinds and "door_room" in kinds
        mixed = S.build_spec("mixed", seed)
        kinds = [a["kind"] for a in mixed["annotations"]]
        assert {"box_with_room", "sealed_room", "u_shape", "box_in_room", "box_at_frame",
                "dc_pocket"} <= set(kinds)
