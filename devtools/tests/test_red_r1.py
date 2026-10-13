"""Red R1: border inference, map serialization, placement and submission."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod.observation import Observation  # noqa: E402
from swarm_rescue.solutions.eod.wallmap import SPAN_MARGIN, WallEvidence, render  # noqa: E402


def _border_evidence(h=300, w=400, gap=None, thickness=6):
    """Evidence on the inner face of a border frame, optionally with a gap
    (columns ``gap[0]:gap[1]`` of the top side have no wall and no hits)."""
    ev = np.zeros((h, w), np.float32)
    face = thickness - 1
    ev[face, :] = 3.0
    ev[h - 1 - face, :] = 3.0
    ev[:, face] = 3.0
    ev[:, w - 1 - face] = 3.0
    if gap is not None:
        ev[:16, gap[0]:gap[1]] = 0.0
    return ev


def test_border_gap_is_not_filled_beyond_the_margin():
    gap = (150, 230)                                    # 80 px opening in the top border
    wall = render(_border_evidence(gap=gap))
    top = wall[:4, :]
    assert top[:, 20:140].all()                         # observed border drawn
    filled = np.flatnonzero(top[:, gap[0]:gap[1]].any(axis=0))
    # Only a 12 px margin next to each observed end may be drawn inside the
    # gap. The bound is written out, not read from the module, so that a
    # change of the module constant cannot loosen the test.
    assert SPAN_MARGIN == 12
    assert 0 < filled.size <= 2 * 12 + 2
    assert not top[:, gap[0] + 14:gap[1] - 14].any()
    assert gap[1] - 14 - (gap[0] + 14) == 52


def test_border_band_ablation_switch():
    ev = _border_evidence()
    with_band = render(ev)
    without = render(ev, border=False)
    assert with_band[:4, 50:350].all()                  # band from the edge to the face
    assert not without[:3, 50:350].any()                # only the evidence ridge remains
    assert without.sum() < with_band.sum()


def test_interior_obstacle_near_the_edge_creates_no_band_elsewhere():
    h, w = 300, 400
    ev = np.zeros((h, w), np.float32)
    ev[10:13, 100:140] = 4.0                            # a short wall 10 px below the top edge
    wall = render(ev)
    assert wall[:16, 100:140].any()                     # it is drawn where it was seen
    assert not wall[:16, :100 - SPAN_MARGIN - 1].any()  # nothing elsewhere along the edge
    assert not wall[:16, 140 + SPAN_MARGIN + 1:].any()
    assert not wall[16:, :].any() or wall[16:, :].sum() < 50


def test_interior_obstacle_far_from_the_edge_is_not_a_border():
    ev = np.zeros((300, 400), np.float32)
    ev[40:42, 100:200] = 5.0                            # 40 px from the top edge
    wall = render(ev)
    assert not wall[:20, :].any()
    assert wall[38:45, 100:200].any()


def test_unknown_space_is_never_drawn():
    assert render(np.zeros((300, 400), np.float32)).sum() == 0
    sparse = np.zeros((300, 400), np.float32)
    sparse[150, 200] = 1.0                              # one isolated return
    assert render(sparse).sum() == 0


@pytest.mark.parametrize("size", [(1250, 800), (1113, 750), (1660, 1122), (401, 299)])
def test_map_serialization_shape_and_values(size):
    from swarm_rescue.solutions.eod.red import PlaceBrain
    brain = PlaceBrain(0, 3, size, 2000, 90.0)
    grid = brain._build_map()
    w, h = size
    assert grid.shape == (h, w) and grid.dtype == np.uint8
    assert set(np.unique(grid)).issubset({0, 1})
    ev = WallEvidence(size)
    angles = np.linspace(-math.pi, math.pi, P.LIDAR_RAYS)
    ranges = np.full(P.LIDAR_RAYS, 120.0)
    for heading in np.linspace(0.0, math.radians(2.0), 20):   # rays 4 px apart: densify
        ev.add_returns(0.0, 0.0, heading, ranges, angles, np.ones(P.LIDAR_RAYS, bool))
    wall = render(ev.counts)
    rows, cols = np.nonzero(wall)
    # The ring of returns at radius 120 is drawn at the scorer's indexing.
    r = np.hypot(cols - w / 2.0, h / 2.0 - rows)
    assert rows.size > 0 and np.all(np.abs(r - 121.5) < 6.0)


def _obs(step, walltime=0.0, inventory=2):
    angles = np.linspace(-math.pi, math.pi, P.LIDAR_RAYS)
    return Observation(step=step, walltime=walltime, identifier=0, n_drones=1,
                       size_area=(400, 300), max_steps=1000, max_walltime=60.0,
                       gps=(0.0, 0.0), compass=0.0, odometer=(0.0, 0.0, 0.0),
                       lidar=np.full(P.LIDAR_RAYS, 290.0), lidar_angles=angles,
                       semantic=[], grasping=False, health=50, inventory=inventory)


def test_submission_timing_deadline_and_walltime_safety():
    from swarm_rescue.solutions.eod.red import PlaceBrain
    brain = PlaceBrain(0, 1, (400, 300), 1000, 60.0)
    assert brain.t_deadline == 495 and brain.t_submit == 470
    assert not brain._should_submit(_obs(400))
    assert brain._should_submit(_obs(470))             # alone: no teammate snapshot needed
    other = PlaceBrain(1, 2, (400, 300), 1000, 60.0)
    other.fleet[0] = (0.0, 0.0, 300, False)            # a lower identifier was heard: not leader
    assert not other._should_submit(_obs(496))
    assert other._should_submit(_obs(496, walltime=0.89 * 60.0))
    assert "walltime" in other.diag["submit_reason"]


def test_placement_is_a_rising_edge_and_respects_the_center():
    from swarm_rescue.solutions.eod.red import PlaceBrain
    brain = PlaceBrain(0, 1, (400, 300), 1000, 60.0)
    brain.step(_obs(1))
    brain.est.x, brain.est.y = 0.0, 0.0
    brain.prev_place_cmd = 0
    brain.dc.box = (100.0, -20.0, 160.0, 20.0)         # center 100 px away
    brain.dc._dirty = False
    obs = _obs(brain.t_rally - 10)
    cmd = brain._emergency_place(obs, *_view_and_others(brain, obs))
    assert cmd.place_bomb == 1
    brain.prev_place_cmd = 1
    cmd = brain._emergency_place(obs, *_view_and_others(brain, obs))
    assert cmd.place_bomb == 0                          # no repeated edge
    brain.prev_place_cmd = 0
    brain.dc.box = (-10.0, -10.0, 10.0, 10.0)          # drone inside the center margin
    cmd = brain._emergency_place(obs, *_view_and_others(brain, obs))
    assert cmd.place_bomb == 0


def _view_and_others(brain, obs):
    from swarm_rescue.solutions.eod.motion import LocalView
    return LocalView(brain.est.heading, obs.lidar, obs.lidar_angles), []
