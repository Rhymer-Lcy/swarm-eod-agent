"""Phase 4, Risk B: R2's enclosed-unknown-as-solid inference (characterization).

R2 weighs, for placement only, every 4-connected region of unknown cells of
its own occupancy grid that touches no free cell, does not reach the grid
edge, is at most 70,000 px^2 and does not overlap the observed disposal
center. The submitted map is unchanged. These tests build occupancy grids
directly (cell level, 8 px cells) for the situations named in the Phase 4
brief and record what the frozen R2 infers. A region that is not wall but is
inferred is a false positive by construction; such cases document the
limitation, they do not change R2.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod import red as r2_red  # noqa: E402

SIZE = (800, 600)          # grid 100 x 75 cells


def _brain():
    brain = r2_red.PlaceBrain(0, 5, SIZE, 5000, 180.0)
    g = brain.grid
    g.logodds[:, :] = P.LOGODDS_MIN          # everything observed free
    g.observed[:, :] = True
    g.version += 1
    return brain


def _set(brain, r0, c0, r1, c1, value):
    g = brain.grid
    g.logodds[r0:r1, c0:c1] = value
    g.observed[r0:r1, c0:c1] = value != 0.0
    g.version += 1


def _outline(brain, r0, c0, r1, c1, door=None):
    """Occupied one-cell ring around rows r0..r1-1, cols c0..c1-1, interior
    unknown; ``door`` = (row or col slice on the top side) left free."""
    _set(brain, r0, c0, r1, c1, P.LOGODDS_MAX)
    _set(brain, r0 + 1, c0 + 1, r1 - 1, c1 - 1, 0.0)
    if door is not None:
        a, b = door
        _set(brain, r0, a, r0 + 1, b, door_value(brain))


def door_value(_brain):
    return P.LOGODDS_MIN


def _inferred(brain, r0, c0, r1, c1) -> int:
    return int(brain._solid_interiors()[r0:r1, c0:c1].sum())


def test_actual_solid_box_outline_is_inferred():
    b = _brain()
    _outline(b, 20, 20, 36, 40)                 # 160 x 128 px box seen from outside
    assert _inferred(b, 21, 21, 35, 39) == 14 * 18


def test_sealed_empty_room_is_inferred_like_a_box():
    """A sealed empty room maps exactly like a solid box (outline, unknown
    interior): it is inferred, a false positive the grid cannot avoid."""
    box, room = _brain(), _brain()
    _outline(box, 20, 20, 36, 40)
    _outline(room, 20, 20, 36, 40)
    assert np.array_equal(box._solid_interiors(), room._solid_interiors())
    assert _inferred(room, 21, 21, 35, 39) > 0


def test_room_seen_through_its_entrance_is_not_inferred():
    b = _brain()
    _outline(b, 20, 20, 36, 40, door=(28, 33))  # 40 px door on the top side
    _set(b, 21, 26, 27, 35, P.LOGODDS_MIN)      # lidar carved part of the inside
    assert _inferred(b, 21, 21, 35, 39) == 0


def test_unobserved_door_keeps_the_room_open():
    """The door cells themselves unknown: the unknown region reaches the free
    cells outside through the door, so it is not enclosed."""
    b = _brain()
    _outline(b, 20, 20, 36, 40)
    _set(b, 20, 28, 21, 33, 0.0)                # door cells unknown, not occupied
    assert _inferred(b, 21, 21, 35, 39) == 0


def test_narrow_entrance_closed_at_cell_resolution_is_inferred():
    """A door narrower than a cell, whose cell was marked occupied by the
    wall ends on both sides, closes the outline: the room is inferred."""
    b = _brain()
    _outline(b, 20, 20, 36, 40)                 # door cell rendered occupied
    assert _inferred(b, 21, 21, 35, 39) > 0


def test_incomplete_outline_is_not_inferred():
    b = _brain()
    _outline(b, 20, 20, 36, 40)
    _set(b, 35, 21, 36, 39, 0.0)                # bottom side never seen
    _set(b, 36, 21, 40, 39, 0.0)                # nor the space below it
    _set(b, 40, 10, 41, 50, P.LOGODDS_MIN)      # free space beyond
    assert _inferred(b, 21, 21, 35, 39) == 0


def test_false_wall_observations_enclosing_unknown_cells_are_inferred():
    """Spurious occupied cells (noise) around a pocket never scanned create a
    small false solid region."""
    b = _brain()
    _set(b, 50, 60, 54, 64, P.LOGODDS_MAX)      # a 4 x 4 block of false hits
    _set(b, 51, 61, 53, 63, 0.0)                # with 2 x 2 unknown inside
    assert _inferred(b, 51, 61, 53, 63) == 4


def test_disposal_center_interior_is_excluded_when_the_center_is_known():
    b = _brain()
    _outline(b, 30, 30, 42, 52)                 # the center's outline
    # Center 176 x 96 px at rows 30..41, cols 30..51; world coordinates:
    x0, y0 = 30 * 8 - SIZE[0] / 2, SIZE[1] / 2 - 42 * 8
    x1, y1 = 52 * 8 - SIZE[0] / 2, SIZE[1] / 2 - 30 * 8
    for x in np.linspace(x0 + 4, x1 - 4, 8):
        b.dc.add(float(x), y1 - 1.0)
        b.dc.add(float(x), y0 + 1.0)
    assert b.dc.bbox() is not None
    assert _inferred(b, 31, 31, 41, 51) == 0


def test_disposal_center_seen_from_one_face_only_is_still_excluded():
    """Only the north face's points: the box estimate is thin, but it touches
    the interior region, which is excluded as a whole."""
    b = _brain()
    _outline(b, 30, 30, 42, 52)
    x0, x1 = 30 * 8 - SIZE[0] / 2, 52 * 8 - SIZE[0] / 2
    y1 = SIZE[1] / 2 - 30 * 8
    for x in np.linspace(x0 + 4, x1 - 4, 10):
        b.dc.add(float(x), y1 - 1.0)
    assert _inferred(b, 31, 31, 41, 51) == 0


def test_disposal_center_never_observed_is_inferred_as_solid():
    """Without any observation of the center (no box, no shared box), its
    interior is indistinguishable from a solid box and is inferred."""
    b = _brain()
    _outline(b, 30, 30, 42, 52)
    assert b.dc.bbox() is None
    assert _inferred(b, 31, 31, 41, 51) > 0


def test_box_sharing_a_wall_with_a_sealed_room_infers_both():
    """Mixed solid and hollow structures: two enclosed regions separated by a
    shared wall are both inferred; the room is the false positive."""
    b = _brain()
    _outline(b, 20, 20, 36, 36)                 # box
    _outline(b, 20, 35, 36, 52)                 # sealed room, shared column 35
    assert _inferred(b, 21, 21, 35, 35) > 0
    assert _inferred(b, 21, 36, 35, 51) > 0


def test_large_enclosed_region_above_the_area_cap_is_not_inferred():
    b = _brain()
    _outline(b, 5, 5, 45, 45)                   # 320 x 320 px = 102,400 px^2 > 70,000
    assert _inferred(b, 6, 6, 44, 44) == 0
