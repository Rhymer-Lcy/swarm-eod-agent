"""Phase 5, candidate B4-S: approach seeking while carrying, and the grasp guard.

B4-S (``solutions/eod/blue.py``) differs from the frozen B2 (``p3_067818a``):

1. Where B2's carrier has no exit, B4-S looks for one. Center unknown: it
   explores reachable unsearched or frontier cells along planned paths (B2
   waited at its start area). Center known without a *verified* approach (a
   supported, unpenalized witnessed stand-off, or map-checked unpenalized
   radial stand-offs from an own box): B2's radial goals are used as in B2
   unless they are unreachable or an approach on them failed; then it visits
   view positions around the box for ``ESCAPE_STEPS``, excluding those from
   which no face was seen. Everywhere else the delivery is B2's.
2. A drone fetching a bomb keeps its grasper off while a bomb held by another
   drone, or a carrying teammate, is within grasping reach.
3. A teammate's center box is accepted only when finite and well ordered.

Every case below runs the same inputs through the frozen B2 (and, where it
tells something, the rejected B3) as the baseline. State tests set brain state
directly; closed-loop tests use the offline approximation (development
signals, not simulator rounds). Geometry comes from ``delivery_fixtures``,
never from official maps; ground truth is used only by assertions.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.comparators.p3_067818a import blue as b2_blue  # noqa: E402
from devtools.simlab import delivery_fixtures as DF  # noqa: E402
from devtools.tests import test_b3_seek_center as TB3  # noqa: E402
from devtools.tests import test_blue_delivery as TBD  # noqa: E402
from devtools.tests import test_p4_risk_a as RA  # noqa: E402
from swarm_rescue.solutions.eod import blue as b4_blue  # noqa: E402
from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod.comms import make_message  # noqa: E402
from swarm_rescue.solutions.eod.motion import LocalView  # noqa: E402
from swarm_rescue.solutions.eod.perception import BombDetection  # noqa: E402

BOX = RA.BOX                     # the pocket center, x -140..20, y -40..40; open north
VIEW = LocalView(0.0, None, RA.ANGLES)


def _carry(brain, step, messages=()):
    """One carry decision at ``step`` (the brain holds a bomb)."""
    obs = RA._obs(step)
    if messages:
        obs.messages.extend(messages)
        brain._read_messages(obs)
    brain.carried = None
    return brain._carry(obs, LocalView(brain.est.heading, None, RA.ANGLES), [])


def _unknown_east(brain, x_from=150.0):
    """Leave the map east of ``x_from`` unobserved (a frontier exists)."""
    g = brain.grid
    cols = np.arange(g.nx)
    xs = (cols + 0.5) * g.cell - g.width / 2.0
    east = xs > x_from
    g.logodds[:, east] = 0.0
    g.observed[:, east] = False
    g.version += 1


def _reachable(brain, goal) -> bool:
    field = brain.planner.field([brain.grid.to_cell(brain.est.x, brain.est.y)],
                                allow_unknown=True, step=10,
                                unknown_cost=b4_blue.CARRY_UNKNOWN_COST)
    return bool(np.isfinite(field[brain.grid.to_cell(*goal)]))


# ------------------------------------------------------- A. center unknown
def test_unknown_center_carrier_seeks_a_reachable_goal_instead_of_home():
    """B2 drives at its start area (the mechanism of the Phase 4 corridor
    0/10); B4-S chooses a reachable unsearched or frontier goal and follows a
    planned path that crosses no mapped wall."""
    walls, _, _ = RA._rasters("pocket")
    b2 = RA._brain(module=b2_blue)
    _unknown_east(b2)
    _carry(b2, 10)
    assert b2.state == "carry" and b2.target == b2.home
    b4 = RA._brain(module=b4_blue)
    _unknown_east(b4)
    _carry(b4, 10)
    assert b4.state == "seek" and b4._seek_kind == "search"
    goal = b4._seek_goal
    assert goal is not None and goal != b4.home and _reachable(b4, goal)
    r, c = b4.grid.to_cell(*goal)
    assert b4.grid.frontier()[r, c] or (b4.grid.free()[r, c] and not b4.searched[r, c])
    assert not RA._hits(walls, b4.est.x, b4.est.y, *b4.target)
    assert b4.target == goal                     # teammates are told the seek goal


def test_nothing_left_to_seek_falls_back_along_a_planned_path_then_resumes():
    """No frontier and everything searched: seeking pauses (no invented goal),
    the carrier heads for its start area along a planned path, and seeks again
    once a frontier appears after ``SEEK_RECHECK`` steps."""
    walls, _, _ = RA._rasters("pocket")
    b4 = RA._brain(module=b4_blue, home=(-60.0, 120.0))
    b4.searched[:, :] = True
    _carry(b4, 10)
    assert b4.diag["seek_exhausted"] == 1 and b4._seek_goal is None
    assert b4.state == "carry"
    assert not RA._hits(walls, b4.est.x, b4.est.y, *b4.target)   # B2 aimed straight through
    _carry(b4, 11)
    assert b4.diag["seek_exhausted"] == 1                         # paused, not re-evaluated
    _unknown_east(b4)
    _carry(b4, 10 + b4_blue.SEEK_RECHECK + 1)
    assert b4._seek_goal is not None and b4._seek_goal[0] > 100.0
    assert _reachable(b4, b4._seek_goal)


def test_missing_sensors_and_messages_still_give_a_finite_command():
    """No lidar, no semantic data, no GPS or compass, no messages, center
    unknown, carrying: the brain still returns a finite command."""
    b4 = RA._brain(module=b4_blue)
    _unknown_east(b4)
    obs = RA._obs(10)
    obs.lidar, obs.semantic = None, None
    cmd = b4.step(obs)
    for v in (cmd.forward, cmd.lateral, cmd.rotation):
        assert math.isfinite(v)
    assert b4.diag.get("errors", 0) == 0


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_hidden_center_carriers_find_it_and_deliver(seed):
    """Closed loop, ``hidden`` fixture (center out of sight at the end of a
    70 px corridor; both drones already carry; the center was never seen).
    Frozen B2 delivers nothing (test_b3_seek_center). B4-S must find the
    center with every carrier and deliver at least one bomb, without a drop.
    A second carrier may still lack a route: the planner's inflation closes
    this corridor in maps drawn under pose error (Phase 4 P4-A016), which is
    the separate narrow-passage track (B4-N), not this candidate."""
    disposed, brains, _ = TB3._hidden_run(b4_blue, seed=seed)
    assert disposed >= 1, [b.diag for b in brains]
    assert all(b.dc.known for b in brains)
    assert all(b.diag["seek_goals"] > 0 for b in brains)
    assert all(b.diag["drops"] == 0 for b in brains)


# ------------------------------------------- A. center known: transitions
def test_teammate_box_switches_search_to_view_positions_then_witnesses_to_b2():
    """A valid teammate box turns the search into view-position seeking; own
    witnessed stand-offs then make the approach verified and B2's delivery
    takes over (no seek goal)."""
    b4 = RA._brain(module=b4_blue)
    _unknown_east(b4)
    _carry(b4, 10)
    assert b4._seek_kind == "search"
    msg = make_message(3, 10, (300.0, 200.0), r="B", dc=BOX)
    _carry(b4, 11, messages=[msg])
    assert b4.dc.shared_box == BOX
    assert b4._seek_kind == "view"
    gx, gy = b4._seek_goal
    gap = max(BOX[0] - gx, 0.0, gx - BOX[2]), max(BOX[1] - gy, 0.0, gy - BOX[3])
    assert abs(math.hypot(*gap) - b4_blue.SEEK_VIEW_OFFSET) < 1.0
    assert _reachable(b4, (gx, gy))
    b4.approach.observe(b4.grid, *RA.NORTH_VIEW, RA._north_face_points(), 12)
    _carry(b4, 13)
    assert b4._verified_approach(13)
    assert b4._seek_goal is None and b4.state in ("carry", "deliver")


def test_own_detection_of_the_open_face_verifies_the_approach():
    """Recovery from a new own observation: semantic hits on the open face
    record witnesses through the normal perception path; the next decision is
    B2's delivery."""
    from swarm_rescue.solutions.eod.observation import SemanticHit
    b4 = RA._brain(module=b4_blue, pose=(RA.NORTH_VIEW[0], RA.NORTH_VIEW[1], -math.pi / 2))
    b4.dc.shared_box = BOX
    _carry(b4, 10)
    assert b4.state == "seek"
    sem = [SemanticHit(distance=70.0 + P.SEMANTIC_BIAS - 2.0, angle=a,
                       kind="DISPOSAL_CENTER", grasped=False)
           for a in np.linspace(-0.6, 0.6, 9)]
    b4._perceive(RA._obs(11, semantic=sem))
    assert len(b4.approach) > 0
    _carry(b4, 12)
    assert b4._verified_approach(12) and b4._seek_goal is None


def test_failed_view_is_excluded_then_eligible_again_after_its_mark_expires():
    """A view goal whose travel budget expires is marked; the next goal lies
    outside the mark; after ``SEEK_MARK_STEPS`` the area is eligible again."""
    b4 = RA._brain(module=b4_blue)
    b4.dc.shared_box = BOX
    _carry(b4, 10)
    g1, budget = b4._seek_goal, b4._seek_budget
    _carry(b4, 10 + budget + 1)                   # no movement: the budget expires
    g2 = b4._seek_goal
    assert b4.diag["seek_expired"] == 1 and g2 != g1
    assert math.hypot(g2[0] - g1[0], g2[1] - g1[1]) >= b4_blue.SEEK_MARK_RADIUS
    later = 10 + budget + 1 + b4_blue.SEEK_MARK_STEPS + 1
    assert not b4._marked(np.array([g1[0]]), np.array([g1[1]]), later)[0]


def test_view_reached_without_sight_of_a_face_is_negative_evidence():
    """At a view position from which no face is seen (walls stop semantic
    rays), the position is marked and another one is chosen."""
    b4 = RA._brain(module=b4_blue)
    b4.dc.shared_box = BOX
    _carry(b4, 10)
    g1 = b4._seek_goal
    b4.est.x, b4.est.y = g1                       # arrived; nothing in sight
    b4.dc_live = []
    _carry(b4, 11)
    assert b4.diag["seek_views_empty"] == 1
    assert b4._seek_goal != g1 and b4._marked(np.array([g1[0]]), np.array([g1[1]]), 11)[0]


def test_seek_state_stays_bounded():
    b4 = RA._brain(module=b4_blue)
    for k in range(200):
        b4._mark((float(k), 0.0), 10)
    assert len(b4._seek_marks) <= b4_blue.SEEK_MAX_MARKS


# --------------------------------------------------- C. delivery safety
@pytest.mark.parametrize("pose", [(-60.0, -60.0, math.pi / 2), (-160.0, 0.0, 0.0),
                                  (40.0, 0.0, math.pi), (-60.0, -95.0, math.pi / 2)])
def test_no_terminal_push_or_path_through_a_wall_from_a_blocked_side(pose):
    """Safety invariant kept from B2: behind a wall of the pocket (center known
    from a box only) there is no certified terminal push and the carrier's
    target is never across a wall."""
    walls, _, _ = RA._rasters("pocket")
    b4 = RA._brain(pose=pose, module=b4_blue)
    b4.dc.shared_box = BOX
    b4.dc_live = []
    assert b4._terminal_aim(RA._obs(10)) is None
    _carry(b4, 10)
    assert b4.state in ("seek", "carry")
    assert not RA._hits(walls, b4.est.x, b4.est.y, *b4.target)


@pytest.mark.parametrize("seed", [1, 2])
def test_known_center_delivery_is_command_identical_to_b2(seed):
    """With a verified approach B4-S is B2: the Phase 3 corner carrier gives
    the same commands, outcome and final pose bit for bit, and the shared
    diagnostic counters agree; no seeking happens."""
    w2, brain2, cmds2, took2 = TBD._teleport_carrier(b2_blue, (18.0, -75.0), seed=seed)
    w4, brain4, cmds4, took4 = TBD._teleport_carrier(b4_blue, (18.0, -75.0), seed=seed)
    assert brain4.diag["seek_goals"] == 0 and brain4.diag["seek_steps"] == 0
    assert (w2.disposed, took2) == (w4.disposed, took4)
    assert cmds2 == cmds4
    d2, d4 = w2.drones[0], w4.drones[0]
    assert (d2.x, d2.y, d2.heading) == (d4.x, d4.y, d4.heading)
    assert {k: v for k, v in brain4.diag.items() if k in brain2.diag} == brain2.diag


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_box_only_complete_map_walled_side_start_now_delivers(seed):
    """Risk A deadlock (frozen B2 never delivers: test_p4_risk_a): center from
    a teammate's box only, the carrier's map already complete, its start area
    behind a walled side. B4-S seeks view positions, sees the open face and
    delivers, without a drop and without a terminal push from behind a wall."""
    disposed, diag, trace = RA._carrier_run("pocket", (60.0, -150.0), home=(-60.0, -150.0),
                                            see_first=[], shared_box=True, premap=True,
                                            seed=seed, module=b4_blue)
    assert disposed == 1, (diag, trace)
    assert trace["first_witness"] is not None
    assert diag["escapes"] > 0 and diag["seek_goals"] > 0 and diag["drops"] == 0
    assert trace["end_xy"][1] > BOX[3]                       # at the open north face


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_box_only_incomplete_map_is_b2_unchanged(seed):
    """Where B2's radial goals work (center from a box only, map still empty,
    start area on either side; Phase 4 Risk A), B4-S does not intervene: no
    escape, and the same delivery step and end position as B2, bit for bit.
    (A first version that replaced these goals by view positions around the
    box was slower and lost bombs offline: a box from a partial view put view
    positions inside the center; Phase 5 attempt P5-A001.)"""
    for home, start in (((-60.0, 130.0), RA.SOUTH), ((-60.0, -150.0), (60.0, -150.0))):
        d2, diag2, trace2 = RA._carrier_run("pocket", start, home=home, see_first=[],
                                            shared_box=True, seed=seed)
        d4, diag4, trace4 = RA._carrier_run("pocket", start, home=home, see_first=[],
                                            shared_box=True, seed=seed, module=b4_blue)
        assert d4 == d2 == 1, (home, diag4, trace4)
        assert diag4["escapes"] == 0 and diag4["seek_goals"] == 0
        assert (trace4["steps"], trace4["end_xy"]) == (trace2["steps"], trace2["end_xy"])


def _walled_witness_run(module, seed, steps=2500):
    """The Phase 4 S2-2 drone-0 mechanism on the pocket fixture: a carrier
    behind the south wall knows the center from a teammate's box, holds ONE own
    surface point of the walled south face, recorded 8 px too far south (a
    pose or range error, as drone 0's single point; Phase 4 Risk A used 20 px),
    and the stand-off witnessed for it behind the wall, which the map then
    supports (the segment's last ``SURFACE_STOP`` px are not checked); its map
    is complete. Returns (disposed, diag, terminal pushes started from behind
    the south wall, steps)."""
    from swarm_rescue.solutions.eod import delivery as D
    from devtools.simlab.offline_sim import OfflineWorld
    fx = DF.fixture("pocket", steps + 400)
    fx["start_area"] = [-60.0, -150.0]
    brains = []

    def factory(i):
        brains.append(module.RescueBrain(i, 1, (DF.W, DF.H), steps + 400))
        return brains[-1]

    world = OfflineWorld(fx, "rescue", 1, factory, seed=seed, bombs=[(250.0, -200.0)],
                         max_steps=steps + 400, carry_ratio_sigma=0.36)
    d, bomb, brain = world.drones[0], world.bombs[0], brains[0]
    world.step()
    walls, solid, outline = RA._rasters("pocket")
    RA._paint(brain.grid, walls | outline, unknown_px=solid & ~walls & ~outline)
    point = (-60.0, BOX[1] - 8.0)
    brain.dc.points.clear()
    brain.dc.box = None
    brain.dc.add(*point)
    brain.dc.shared_box = BOX
    brain.approach.witnesses.clear()
    brain.approach.witnesses[brain.grid.to_cell(-60.0, BOX[1] - D.STANDOFF)] = \
        (-60.0, BOX[1] - D.STANDOFF, point[0], point[1], 1)
    assert brain.approach.valid(brain.grid, 1), "precondition: the map supports the witness"
    d.x, d.y = -60.0, -110.0
    d.prev_x, d.prev_y, d.vx, d.vy = d.x, d.y, 0.0, 0.0
    brain.est.x, brain.est.y, brain.est.vx, brain.est.vy = d.x, d.y, 0.0, 0.0
    bomb.x = d.x + P.CARRY_OFFSET * math.cos(d.heading)
    bomb.y = d.y + P.CARRY_OFFSET * math.sin(d.heading)
    bomb.holder, d.carrying, d.grasp_angle = 0, 0, 0.0
    t0, blocked_pushes = world.step_index, 0
    while world.step() and world.disposed == 0 and world.step_index - t0 < steps:
        if brain.state == "deliver" and d.y < BOX[1] - 3.0:
            blocked_pushes += 1
    return world.disposed, dict(brain.diag), blocked_pushes, world.step_index - t0


@pytest.mark.parametrize("seed", [1, 2])
def test_walled_side_witness_no_longer_traps_the_carrier(seed):
    """S2-2 drone-0 mechanism. Frozen B2 keeps returning to the witness behind
    the wall (penalized, then re-used as relaxed goals) and never delivers;
    B4-S, once that witness is penalized, has no verified approach, seeks a
    view of the open face and delivers."""
    d2, diag2, _, _ = _walled_witness_run(b2_blue, seed)
    assert d2 == 0 and diag2["approach_failures"] >= 2, diag2
    d4, diag4, _, took = _walled_witness_run(b4_blue, seed)
    assert d4 == 1, diag4
    assert diag4["seek_goals"] > 0, diag4
    assert diag4["drops"] == 0, diag4


def test_closed_loop_is_deterministic():
    a = _walled_witness_run(b4_blue, 1)
    b = _walled_witness_run(b4_blue, 1)
    assert a == b


# ---------------------------------------------------- B. bomb possession
def _fetcher(module):
    brain = module.RescueBrain(0, 2, (DF.W, DF.H), 3000)
    est = brain.est
    est.x, est.y, est.heading = 0.0, 0.0, 0.0
    est.initialized = True
    est.heading_var = math.radians(2.0) ** 2
    brain.home = (0.0, 0.0)
    brain.claim = module.Bomb(60.0, 0.0, 0, 5)
    brain.state = "approach"
    brain.approach_start = 9
    return brain


def _fetch(brain, step, grasped_near=False, messages=()):
    dets = [BombDetection(60.0, 0.0, False, 60.0, 0.0)]
    if grasped_near:
        dets.append(BombDetection(30.0, 25.0, True, math.hypot(30.0, 25.0), math.atan2(25.0, 30.0)))
    brain.detections = dets
    obs = RA._obs(step, grasping=False)
    if messages:
        obs.messages.extend(messages)
        brain._read_messages(obs)
    return brain._fetch(obs, VIEW, [])


def test_grasper_stays_off_near_a_bomb_another_drone_holds():
    """Phase 4 S2-2: a fetching drone closed its grasper within reach of a
    teammate's carried bomb and held it too. B2 (baseline) closes the grasper
    here; B4-S does not, and closes it again once no held bomb is near."""
    assert _fetch(_fetcher(b2_blue), 10, grasped_near=True).grasper == 1
    b4 = _fetcher(b4_blue)
    assert _fetch(b4, 10, grasped_near=True).grasper == 0
    assert b4.diag["grasp_guards"] >= 1
    assert _fetch(b4, 11, grasped_near=False).grasper == 1


def test_grasper_stays_off_near_a_carrying_teammate_only_while_its_report_is_fresh():
    near = make_message(1, 10, (40.0, 10.0), r="B", c=True)
    b4 = _fetcher(b4_blue)
    assert _fetch(b4, 11, messages=[near]).grasper == 0
    b4 = _fetcher(b4_blue)
    not_carrying = make_message(1, 10, (40.0, 10.0), r="B", c=False)
    assert _fetch(b4, 11, messages=[not_carrying]).grasper == 1
    b4 = _fetcher(b4_blue)
    _fetch(b4, 11, messages=[near])
    assert _fetch(b4, 20).grasper == 1           # report older than 3 steps: no suppression


def test_bomb_records_newest_information_wins_and_a_release_is_reacquirable():
    t = b4_blue.BombTable()
    t.merge_remote(10.0, 0.0, 1, 50)              # carried, newer
    t.merge_remote(10.0, 0.0, 0, 40)              # stale "free" report arrives later
    assert t.items[0].status == 1 and not t.free(60)
    t.merge_remote(14.0, 0.0, 0, 60)              # released afterwards
    assert t.items[0].status == 0 and t.free(61)
    t.items[0].fail_until = 461                   # own grasp failure: skipped, not forever
    assert not t.free(400) and t.free(462)


def test_malformed_center_boxes_are_ignored():
    """B2 accepted any 4-tuple as a teammate's box (a non-finite one made the
    center "known"); B4-S accepts only finite, well-ordered boxes."""
    bad = [(math.nan, 0.0, 1.0, 1.0), (0.0, 1.0), (5.0, 0.0, 1.0, 1.0), ("a", 0, 1, 1)]
    for box in bad:
        b4 = RA._brain(module=b4_blue)
        b4._read_messages(_with(RA._obs(5), make_message(2, 4, (0.0, 0.0), dc=box)))
        assert b4.dc.shared_box is None, box
    b2 = RA._brain(module=b2_blue)
    b2._read_messages(_with(RA._obs(5), make_message(2, 4, (0.0, 0.0), dc=bad[0])))
    assert b2.dc.known                            # the baseline defect
    b4 = RA._brain(module=b4_blue)
    b4._read_messages(_with(RA._obs(5), make_message(2, 4, (0.0, 0.0), dc=BOX)))
    assert b4.dc.shared_box == BOX


def _with(obs, msg):
    obs.messages.append(msg)
    return obs
