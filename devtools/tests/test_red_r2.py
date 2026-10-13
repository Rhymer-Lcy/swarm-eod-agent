"""Red R2: relayed placement claims, re-validation of placement targets and
solid-obstacle interiors in the placement weights.

The Phase 3 diagnosis (``devtools/simlab/red_wall_diagnosis.py``) found
placements within one blast radius of each other: claims were heard only
within radio range of their owner, and a chosen target was kept for up to
240 steps even after a bomb placed next to it had been gossiped. It also
found that solid boxes count with their whole area in the wall score while
the lidar maps only their outline. These tests check each mechanism, against
the frozen R1 where the old behaviour differs.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.comparators.p2_540054b import red as r1_red  # noqa: E402
from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod import red as r2_red  # noqa: E402
from swarm_rescue.solutions.eod.comms import make_message  # noqa: E402
from swarm_rescue.solutions.eod.observation import Observation  # noqa: E402

ANGLES = np.linspace(-math.pi, math.pi, P.LIDAR_RAYS)


def _obs(step, messages=(), inventory=2, ident=2):
    return Observation(step=step, walltime=0.0, identifier=ident, n_drones=4,
                       size_area=(400, 300), max_steps=1000, max_walltime=60.0,
                       gps=(0.0, 0.0), compass=0.0, odometer=(0.0, 0.0, 0.0),
                       lidar=np.full(P.LIDAR_RAYS, 290.0), lidar_angles=ANGLES,
                       semantic=[], grasping=False, health=50, inventory=inventory,
                       messages=list(messages))


def _brain(module, ident=2):
    brain = module.PlaceBrain(ident, 4, (400, 300), 1000, 60.0)
    brain.step(_obs(1, ident=ident))
    return brain


def _msg(sender, step, **fields):
    return make_message(sender, step, (0.0, 0.0), r="R", **fields)


def test_relayed_claim_is_merged_with_its_origin_step():
    r2 = _brain(r2_red)
    # Drone 3 relays drone 0's claim, made at step 90, in its message of step 100.
    r2._read_messages(_obs(101, [_msg(3, 100, pr=((0, 50.0, 20.0, 90),))]))
    assert r2.place_claims[0] == (50.0, 20.0, 90)
    # An older relayed copy never overwrites a newer statement.
    r2._read_messages(_obs(102, [_msg(1, 101, pr=((0, -80.0, 0.0, 70),))]))
    assert r2.place_claims[0] == (50.0, 20.0, 90)
    # The owner's own newer statement wins; its own id and malformed items are ignored.
    r2._read_messages(_obs(103, [_msg(0, 102, pc=(60.0, 25.0),
                                      pr=((2, 1.0, 1.0, 99), ("x", 1, 2, 3), (1, 2.0)))]))
    assert r2.place_claims[0] == (60.0, 25.0, 102)
    assert 2 not in r2.place_claims and 1 not in r2.place_claims
    # R1 does not learn claims of drones out of radio range at all.
    r1 = _brain(r1_red)
    r1._read_messages(_obs(101, [_msg(3, 100, pr=((0, 50.0, 20.0, 90),))]))
    assert 0 not in r1.place_claims


def test_a_withdrawn_claim_is_deleted_only_by_a_newer_message():
    r2 = _brain(r2_red)
    r2._read_messages(_obs(101, [_msg(0, 100, pc=(60.0, 25.0))]))
    r2._read_messages(_obs(102, [_msg(3, 101, pr=((0, 60.0, 25.0, 100),))]))
    assert 0 in r2.place_claims
    r2._read_messages(_obs(104, [_msg(0, 103)]))          # owner no longer claims
    assert 0 not in r2.place_claims


def test_outgoing_message_relays_only_fresh_claims():
    r2 = _brain(r2_red)
    r2.place_claims = {0: (10.0, 0.0, 100), 1: (90.0, 0.0, 100 - r2_red.CLAIM_TTL - 1)}
    r2._compose_message(_obs(100))
    assert r2.msg_out["pr"] == ((0, 10.0, 0.0, 100),)


def _placing(module):
    brain = _brain(module)
    brain.est.x, brain.est.y = -150.0, 0.0
    brain.place_target = (100.0, 0.0)
    brain.place_decided = brain.t_place + 100
    brain.prev_place_cmd = 0
    return brain


def _place_step(brain, step):
    from swarm_rescue.solutions.eod.motion import LocalView
    obs = _obs(step)
    return brain._place(obs, LocalView(brain.est.heading, obs.lidar, ANGLES), [])


def test_target_next_to_a_newly_known_bomb_is_given_up():
    step = 0
    r2 = _placing(r2_red)
    step = r2.place_decided + 10
    r2.placed.append((100.0 + 0.5 * r2.blast_radius, 0.0))    # gossiped meanwhile
    _place_step(r2, step)
    assert r2.diag["targets_dropped"] == 1
    assert r2.place_target is None or math.hypot(r2.place_target[0] - 100.0,
                                                 r2.place_target[1]) > 1.0
    # The frozen R1 keeps the redundant target.
    r1 = _placing(r1_red)
    r1.placed.append((100.0 + 0.5 * r1.blast_radius, 0.0))
    _place_step(r1, step)
    assert r1.place_target == (100.0, 0.0)


def test_target_far_from_known_bombs_is_kept():
    r2 = _placing(r2_red)
    r2.placed.append((100.0 + 0.9 * r2.blast_radius, 0.0))    # beyond the overlap distance
    _place_step(r2, r2.place_decided + 10)
    assert r2.diag["targets_dropped"] == 0
    assert r2.place_target == (100.0, 0.0)


def test_relayed_lower_identifier_claim_makes_the_target_yield():
    r2 = _placing(r2_red)
    step = r2.place_decided + 10
    r2._read_messages(_obs(step - 1, [_msg(3, step - 2, pr=((0, 110.0, 5.0, step - 5),))]))
    _place_step(r2, step)
    assert r2.place_target is None or math.hypot(r2.place_target[0] - 100.0,
                                                 r2.place_target[1]) > 1.0
    # A claim older than the time-to-live no longer counts.
    r2b = _placing(r2_red)
    r2b.place_claims[0] = (110.0, 5.0, step - r2_red.CLAIM_TTL - 1)
    _place_step(r2b, step)
    assert r2b.place_target == (100.0, 0.0)


# ------------------------------------------------- solid obstacle interiors
def _grid_with_box(brain, gap=False, big=False):
    """Agent grid: free arena, one box outline (occupied ring, unknown inside)."""
    g = brain.grid
    g.logodds[:, :] = P.LOGODDS_MIN
    g.observed[:, :] = True
    r0, r1, c0, c1 = (2, 35, 2, 47) if big else (12, 22, 15, 27)
    g.logodds[r0:r1 + 1, c0:c1 + 1] = P.LOGODDS_MAX           # ring ...
    g.logodds[r0 + 1:r1, c0 + 1:c1] = 0.0                      # ... around unknown cells
    g.observed[r0 + 1:r1, c0 + 1:c1] = False
    if gap:
        g.logodds[r0, (c0 + c1) // 2] = P.LOGODDS_MIN          # an unobserved face: open
    g.version += 1
    return (r0 + 1, r1, c0 + 1, c1)


def test_enclosed_unknown_region_is_taken_as_solid():
    r2 = _brain(r2_red)
    r0, r1, c0, c1 = _grid_with_box(r2)
    solid = r2._solid_interiors()
    assert solid[r0:r1, c0:c1].all() and solid.sum() == (r1 - r0) * (c1 - c0)


def test_disposal_center_interior_is_not_solid():
    r2 = _brain(r2_red)
    r0, r1, c0, c1 = _grid_with_box(r2)
    # The same enclosed region, observed as the disposal center (world box of
    # the inner cells), is a navigation obstacle but not a wall.
    x0, y1 = r2.grid.to_world(r0, c0)
    x1, y0 = r2.grid.to_world(r1 - 1, c1 - 1)
    r2.dc.box = (x0, y0, x1, y1)
    r2.dc._dirty = False
    assert not r2._solid_interiors().any()


def test_open_outline_unexplored_space_and_large_regions_are_not_solid():
    r2 = _brain(r2_red)
    _grid_with_box(r2, gap=True)
    assert not r2._solid_interiors().any()
    r2b = _brain(r2_red)
    g = r2b.grid
    g.logodds[:, :] = 0.0                                      # nothing mapped yet
    g.logodds[10:14, 10:14] = P.LOGODDS_MIN
    g.version += 1
    assert not r2b._solid_interiors().any()
    r2c = _brain(r2_red)
    _grid_with_box(r2c, big=True)                              # 32 x 44 cells, 90,112 px^2
    assert not r2c._solid_interiors().any()


def _placement_setup(module):
    """800 x 600 arena (blast radius 72 px): a box outline and three thin walls.

    Outline only, the three thin walls offer more wall within one blast than
    any spot next to the box; counting the box interior, the box offers far
    more.
    """
    brain = module.PlaceBrain(0, 1, (800, 600), 1000, 60.0)
    brain.step(_obs(1, ident=0))
    brain.est.x, brain.est.y = 0.0, 0.0
    g = brain.grid
    g.logodds[:, :] = P.LOGODDS_MIN
    g.observed[:, :] = True
    g.logodds[12:23, 15:28] = P.LOGODDS_MAX                    # box ring, grid cells
    g.logodds[13:22, 16:27] = 0.0                              # unknown interior
    g.observed[13:22, 16:27] = False
    g.version += 1
    ev = np.zeros((600, 800), np.uint16)
    ev[96, 120:224] = ev[183, 120:224] = 40                    # box outline, pixels
    ev[96:184, 120] = ev[96:184, 223] = 40
    for row in (420, 428, 436):                                # three thin walls
        ev[row, 450:750] = 40
    brain.evidence.counts = ev
    return brain


def test_placement_counts_box_interiors_and_r1_does_not():
    box = (171.5 - 400.0, 300.0 - 139.5)
    walls = (600.0 - 400.0, 300.0 - 428.0)

    def nearer_box(target):
        return math.hypot(target[0] - box[0], target[1] - box[1]) < \
            math.hypot(target[0] - walls[0], target[1] - walls[1])

    r2 = _placement_setup(r2_red)
    t2 = r2._choose_place_target(r2.t_place + 1)
    assert t2 is not None and nearer_box(t2), t2
    assert r2.diag["solid_cells"] == 9 * 11
    r1 = _placement_setup(r1_red)
    t1 = r1._choose_place_target(r1.t_place + 1)
    assert t1 is not None and not nearer_box(t1), t1
