"""Phase 4, Risk A: the blue radial fallback of the frozen B2 (characterization).

``delivery.radial_sources`` is B2's goal source when a carrier holds no
supported witnessed stand-off: outward stand-offs from the center's middle,
map-checked, and the *unchecked* set when every candidate is rejected. These
tests establish when that fallback is reached, whether its goals can lie on
a blocked side, and whether the terminal movement (which has its own
line-of-sight certificate) can be led through a wall by them. They describe
B2 as frozen (the package ``p3_067818a``; since Phase 5 the agent under
``solutions/`` is the candidate B4-S, whose behaviour in the same cases is
tested in ``test_b4s.py`` with these helpers); they do not change it. Geometry: ``delivery_fixtures`` (``pocket``:
center walled on west, south and east with 2 px gaps; ``narrow``: north face
behind a 30 px opening, east face open), never official map coordinates.
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
from swarm_rescue.solutions.eod import delivery as D  # noqa: E402
from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod.grid import OccupancyGrid  # noqa: E402
from swarm_rescue.solutions.eod.motion import LocalView  # noqa: E402
from swarm_rescue.solutions.eod.observation import Observation, SemanticHit  # noqa: E402

ANGLES = np.linspace(-math.pi, math.pi, P.LIDAR_RAYS)
BOX = (DF.DC["x"] - DF.DC["width"] / 2, DF.DC["y"] - DF.DC["height"] / 2,
       DF.DC["x"] + DF.DC["width"] / 2, DF.DC["y"] + DF.DC["height"] / 2)   # -140,-40,20,40
SOUTH = (-60.0, -95.0)          # behind the south wall (free, 50 px below it)
NORTH_VIEW = (-60.0, 110.0)     # in front of the open north face


# ---------------------------------------------------------------- helpers
def _rasters(scenario):
    fx = DF.fixture(scenario)
    walls = fx["walls"].astype(bool)
    solid = walls.copy()
    r0, c0 = DF._px_row(BOX[3]), DF._px_col(BOX[0])
    r1, c1 = DF._px_row(BOX[1]), DF._px_col(BOX[2])
    solid[r0:r1, c0:c1] = True
    dc_outline = np.zeros_like(walls)
    dc_outline[r0:r1, c0:c1] = True
    dc_outline[r0 + 8:r1 - 8, c0 + 8:c1 - 8] = False
    return walls, solid, dc_outline


def _hits(raster, x0, y0, x1, y1) -> bool:
    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 2
    for t in np.linspace(0.0, 1.0, n):
        r, c = DF._px_row(y0 + t * (y1 - y0)), DF._px_col(x0 + t * (x1 - x0))
        if 0 <= r < raster.shape[0] and 0 <= c < raster.shape[1] and raster[r, c]:
            return True
    return False


def _paint(grid: OccupancyGrid, occupied_px: np.ndarray, unknown_px: np.ndarray = None):
    """Every cell observed: occupied where the raster is set, else free;
    cells where ``unknown_px`` is set stay unobserved."""
    k = int(grid.cell)
    pad = np.zeros((grid.ny * k, grid.nx * k), bool)
    pad[:occupied_px.shape[0], :occupied_px.shape[1]] = occupied_px
    occ = pad.reshape(grid.ny, k, grid.nx, k).any(axis=(1, 3))
    grid.logodds[:, :] = P.LOGODDS_MIN
    grid.logodds[occ] = P.LOGODDS_MAX
    grid.observed[:, :] = True
    if unknown_px is not None:
        pad = np.zeros((grid.ny * k, grid.nx * k), bool)
        pad[:unknown_px.shape[0], :unknown_px.shape[1]] = unknown_px
        unk = pad.reshape(grid.ny, k, grid.nx, k).all(axis=(1, 3))
        grid.logodds[unk] = 0.0
        grid.observed[unk] = False
    grid.version += 1


def _brain(scenario="pocket", pose=(SOUTH[0], SOUTH[1], math.pi / 2), home=(-60.0, 120.0),
           module=b2_blue):
    """A carrier (B2 unless ``module`` names another agent) whose map shows
    the walls and the center's outline (the lidar maps a box's outline; its
    interior stays unobserved)."""
    walls, solid, outline = _rasters(scenario)
    brain = module.RescueBrain(0, 1, (DF.W, DF.H), 3000)
    interior = solid & ~walls & ~outline
    _paint(brain.grid, walls | outline, unknown_px=interior)
    est = brain.est
    est.x, est.y, est.heading = pose
    est.vx = est.vy = 0.0
    est.initialized = True
    est.heading_var = math.radians(2.0) ** 2
    brain.home = home
    return brain


def _valid_goal(cell, grid, walls, solid) -> bool:
    """Free and in sight of the nearest point of the center past no wall."""
    x, y = grid.to_world(*cell)
    if solid[DF._px_row(y), DF._px_col(x)]:
        return False
    qx, qy = min(max(x, BOX[0]), BOX[2]), min(max(y, BOX[1]), BOX[3])
    return not _hits(walls, x, y, qx, qy)


def _north_face_points():
    return [(float(x), BOX[3] - 1.0) for x in range(-136, 20, 6)]


def _obs(step, semantic=None, grasping=True):
    return Observation(step=step, walltime=0.0, identifier=0, n_drones=1,
                       size_area=(DF.W, DF.H), max_steps=3000, max_walltime=600.0,
                       gps=None, compass=None, odometer=None,
                       lidar=np.full(P.LIDAR_RAYS, 290.0), lidar_angles=ANGLES,
                       semantic=semantic or [], grasping=grasping, health=50, inventory=0)


def _reach(brain, goals, at) -> bool:
    field = brain.planner.field(goals, allow_unknown=True, step=10,
                                unknown_cost=b2_blue.CARRY_UNKNOWN_COST)
    return bool(np.isfinite(field[brain.grid.to_cell(*at)]))


# ------------------------------------------------- when the fallback is reached
def test_witnessed_standoffs_suppress_the_fallback():
    brain = _brain()
    brain.approach.observe(brain.grid, *NORTH_VIEW, _north_face_points(), 1)
    for p in _north_face_points():
        brain.dc.add(*p)
    goals = brain._dc_sources(10)
    assert goals and brain.diag["radial_fallbacks"] == 0


def test_missing_semantic_shared_box_only_targets_the_unreachable_middle():
    """Case 1 (missing semantic observations): the center is known only from a
    teammate's box. The fallback's only goal is the center's middle, inside
    the mapped outline, so no route exists from outside: the carrier falls
    back to its start area and holds the bomb there."""
    brain = _brain()
    brain.dc.shared_box = BOX
    assert not brain.dc.points and len(brain.approach) == 0
    goals = brain._dc_sources(10)
    assert brain.diag["radial_fallbacks"] == 1
    assert goals == [brain.grid.to_cell((BOX[0] + BOX[2]) / 2, (BOX[1] + BOX[3]) / 2)]
    assert not _reach(brain, goals, SOUTH)
    assert not _reach(brain, goals, NORTH_VIEW)
    target = brain._path_target_dc(_obs(10))
    # No route to any goal: it heads for the start area (here north, past the
    # open face); a carrier whose start area lies behind a walled side would
    # stay there (closed-loop test below).
    assert math.hypot(target[0] - brain.est.x, target[1] - brain.est.y) > 0.0


def test_stale_shared_box_moves_the_fallback_goal_with_it():
    """Case 2 (stale center observation): a teammate's box offset by 60 px
    puts the single fallback goal 60 px from the real middle."""
    brain = _brain()
    brain.dc.shared_box = (BOX[0], BOX[1] - 60.0, BOX[2], BOX[3] - 60.0)
    goals = brain._dc_sources(10)
    x, y = brain.grid.to_world(*goals[0])
    assert abs(y - (-60.0)) < 8.0                          # free space behind the south wall
    walls, solid, _ = _rasters("pocket")
    assert not _valid_goal(goals[0], brain.grid, walls, solid)
    # It is reachable from behind the wall, and there the carrier finds no
    # certificate for a terminal push: a route goal without an exit.
    assert _reach(brain, goals, SOUTH)
    brain.est.x, brain.est.y = x, y
    brain.dc_live = []
    assert brain._terminal_aim(_obs(10)) is None


def test_all_checked_candidates_rejected_returns_unchecked_goals_behind_walls():
    """Case 3 (all stand-offs blocked): when every map-checked radial
    candidate is rejected, the unchecked set is returned, and it contains
    goals on blocked sides. Here the stored surface points are the north
    face's, recorded 20 px too far south (a heading or position error at
    recording time), so their outward stand-offs land in the outline or
    behind the side walls, and the map rejects them all."""
    brain = _brain()
    for x, y in _north_face_points():
        brain.dc.add(x, y - 20.0)
    for p in ((BOX[0] + 1.0, -20.0), (BOX[2] - 1.0, -20.0), (BOX[0] + 1.0, -35.0),
              (BOX[2] - 1.0, -35.0)):
        brain.dc.add(*p)
    box = brain.dc.bbox()
    pts = brain.dc.points
    cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    cand = []
    for qx, qy in pts[::5]:
        n = math.hypot(qx - cx, qy - cy) or 1.0
        cand.append((qx + 32.0 * (qx - cx) / n, qy + 32.0 * (qy - cy) / n, qx, qy))
    arr = np.array(cand)
    occ = brain.grid.occupied().reshape(-1)
    cells = brain.grid._cells(arr[:, 0], arr[:, 1])
    checked = (cells >= 0) & ~occ[np.maximum(cells, 0)]
    checked &= D.segments_clear(brain.grid, arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3],
                                stop_short=D.SURFACE_STOP)
    goals = D.radial_sources(brain.grid, pts, box)
    walls, solid, _ = _rasters("pocket")
    bad = [c for c in goals if not _valid_goal(c, brain.grid, walls, solid)]
    if checked.any():
        pytest.skip("geometry left a checked candidate; the unchecked branch was not reached")
    assert bad, "the unchecked fallback returned no blocked-side goal"


def test_terminal_push_is_never_certified_from_a_blocked_side():
    """Goals are not movements: at any fallback goal behind a wall, the
    terminal push has no certificate (walls stop semantic rays, and no
    supported witness exists there), so the carrier never drives at the
    center through the wall, at any distance."""
    walls, solid, _ = _rasters("pocket")
    for pose in ((-60.0, -60.0, math.pi / 2), (-160.0, 0.0, 0.0), (40.0, 0.0, math.pi),
                 (-60.0, -95.0, math.pi / 2)):
        brain = _brain(pose=pose)
        brain.dc.shared_box = BOX
        brain.dc_live = []                                 # nothing visible through walls
        assert brain._terminal_aim(_obs(10)) is None
        view = LocalView(brain.est.heading, None, ANGLES)
        brain.carried = None
        brain._carry(_obs(10), view, [])
        assert brain.state == "carry"
        assert not _hits(walls, brain.est.x, brain.est.y, *brain.target)


def test_center_walled_on_three_sides_routes_only_to_the_open_face():
    """Case 4 (three-sided center): with witnesses from the open face, every
    goal is valid and reachable from behind the south wall around the walls."""
    brain = _brain()
    brain.approach.observe(brain.grid, *NORTH_VIEW, _north_face_points(), 1)
    goals = brain._dc_sources(10)
    walls, solid, _ = _rasters("pocket")
    assert goals and all(_valid_goal(c, brain.grid, walls, solid) for c in goals)
    assert _reach(brain, goals, SOUTH)


def test_recovery_after_a_new_observation():
    """Case 6 (recovery): a carrier on the fallback receives one detection of
    the open face; the next field uses the witnessed stand-offs, which are
    valid and reachable."""
    brain = _brain(pose=(NORTH_VIEW[0], NORTH_VIEW[1], -math.pi / 2))
    brain.dc.shared_box = BOX
    brain._dc_sources(10)
    assert brain.diag["radial_fallbacks"] == 1
    sem = [SemanticHit(distance=70.0 + P.SEMANTIC_BIAS - 2.0, angle=a,
                       kind="DISPOSAL_CENTER", grasped=False)
           for a in np.linspace(-0.6, 0.6, 9)]
    brain._perceive(_obs(11, semantic=sem))
    assert len(brain.approach) > 0
    goals = brain._dc_sources(12)
    assert brain.diag["radial_fallbacks"] == 1             # not reached again
    walls, solid, _ = _rasters("pocket")
    assert goals and all(_valid_goal(c, brain.grid, walls, solid) for c in goals)


# ----------------------------------------------- closed loop (offline sim)
def _carrier_run(scenario, start, home, see_first, steps=2500, seed=1, shared_box=False,
                 premap=False, module=b2_blue):
    """One carrier in the offline approximation, started at ``start`` with a
    bomb attached. ``see_first``: positions visited for 30 steps each before
    the carry (what it may witness). ``shared_box``: the carrier's own
    observations of the center are cleared and the center is known from a
    teammate's box only (an emulated memory state, like the teleport).
    ``premap``: the carrier's grid is set to the walls and the center's
    outline (a map completed earlier, for example by its own exploration).
    Returns (disposed, diagnostics, trace) with the steps, counted from the
    start of the carry, of the first witnessed stand-off and of the last
    fallback field, and the farthest north the carrier went."""
    from devtools.simlab.offline_sim import OfflineWorld
    fx = DF.fixture(scenario, steps + 400)
    fx["start_area"] = list(home)
    brains = []

    def factory(i):
        brains.append(module.RescueBrain(i, 1, (DF.W, DF.H), steps + 400))
        return brains[-1]

    world = OfflineWorld(fx, "rescue", 1, factory, seed=seed, bombs=[(250.0, -200.0)],
                         max_steps=steps + 400, carry_ratio_sigma=0.36)
    d, bomb, brain = world.drones[0], world.bombs[0], brains[0]

    def teleport(at):
        d.x, d.y = at
        d.prev_x, d.prev_y, d.vx, d.vy = d.x, d.y, 0.0, 0.0
        brain.est.x, brain.est.y, brain.est.vx, brain.est.vy = d.x, d.y, 0.0, 0.0
        brain.path, brain.target = [], None

    world.step()
    for at in see_first:
        teleport(at)
        for _ in range(30):
            world.step()
    if shared_box:
        brain.dc.points.clear()
        brain.dc.box = None
        brain.dc.shared_box = BOX
        brain.approach.witnesses.clear()
    if premap:
        walls, solid, outline = _rasters(scenario)
        _paint(brain.grid, walls | outline, unknown_px=solid & ~walls & ~outline)
    teleport(start)
    bomb.x = d.x + P.CARRY_OFFSET * math.cos(d.heading)
    bomb.y = d.y + P.CARRY_OFFSET * math.sin(d.heading)
    bomb.holder, d.carrying, d.grasp_angle = 0, 0, 0.0
    t0 = world.step_index
    trace = {"first_witness": None, "last_fallback": None, "max_y": d.y, "steps": 0}
    fallbacks = brain.diag["radial_fallbacks"]
    while world.step() and world.disposed == 0 and world.step_index - t0 < steps:
        t = world.step_index - t0
        if trace["first_witness"] is None and len(brain.approach) > 0:
            trace["first_witness"] = t
        if brain.diag["radial_fallbacks"] != fallbacks:
            fallbacks = brain.diag["radial_fallbacks"]
            trace["last_fallback"] = t
        trace["max_y"] = max(trace["max_y"], d.y)
    trace["steps"] = world.step_index - t0
    trace["end_xy"] = (round(d.x, 1), round(d.y, 1))
    return world.disposed, dict(brain.diag), trace


# Outcomes below were measured in attempt P4-A002 (docs/EXPERIMENTS.md,
# section 9) and are pinned per seed; the offline approximation is
# deterministic for a given seed.

@pytest.mark.parametrize("seed", [1, 2, 3])
def test_closed_loop_box_only_carrier_recovers_while_its_map_is_incomplete(seed):
    """Center known from a box only, own map still empty. The fallback goal
    (the center's middle) is reachable through unknown cells, so it draws the
    carrier around the walls until it sees the open face; then witnessed
    stand-offs take over and it delivers. Both start areas, open side and
    walled side."""
    for home, start in (((-60.0, 130.0), SOUTH), ((-60.0, -150.0), (60.0, -150.0))):
        disposed, diag, trace = _carrier_run("pocket", start, home=home, see_first=[],
                                             shared_box=True, seed=seed)
        assert diag["radial_fallbacks"] > 0
        assert disposed == 1, (home, diag, trace)
        assert trace["first_witness"] is not None
        assert trace["last_fallback"] < trace["first_witness"]
        assert trace["end_xy"][1] > BOX[3]                  # delivered at the open north face


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_closed_loop_box_only_carrier_with_complete_map_deadlocks_behind_the_wall(seed):
    """Confirmed failure mechanism (Risk A): center known from a box only,
    the carrier's map already complete (walls and the center's outline), start
    area behind a walled side. The fallback's only goal is the enclosed middle,
    unreachable; the carrier returns towards its start area behind the south
    wall, never sees the open face and holds the bomb to the end. It never
    pushes at the center through the wall (no terminal aim at all)."""
    disposed, diag, trace = _carrier_run("pocket", (60.0, -150.0), home=(-60.0, -150.0),
                                         see_first=[], shared_box=True, premap=True, seed=seed)
    assert disposed == 0, (diag, trace)
    assert trace["first_witness"] is None
    assert diag["radial_fallbacks"] > 30 and trace["last_fallback"] > 2000
    assert trace["max_y"] < BOX[1]                          # stayed south of the center
    assert diag["live_aims"] == 0 and diag["witness_aims"] == 0


def test_closed_loop_box_only_carrier_with_complete_map_and_open_side_home_recovers():
    """The same complete map with the start area past the open face: the trip
    home passes in sight of the face, and the carrier delivers."""
    disposed, diag, trace = _carrier_run("pocket", SOUTH, home=(-60.0, 130.0), see_first=[],
                                         shared_box=True, premap=True, seed=1)
    assert disposed == 1, (diag, trace)
    assert trace["first_witness"] is not None


@pytest.mark.parametrize("seed,delivers", [(1, True), (2, True), (3, False)])
def test_closed_loop_unknown_narrow_entrance(seed, delivers):
    """Case 5 (narrow entrance): the carrier witnessed the north face only
    through a 30 px opening it cannot pass. The progress watch fires; in two
    seeds the carrier then finds and uses the east face, never pushing
    through the opening; in the third it keeps returning to the opening
    (every witness penalized, relaxed sources re-used): a livelock."""
    disposed, diag, trace = _carrier_run("narrow", (-60.0, 140.0), home=(-60.0, 140.0),
                                         see_first=[(-60.0, 120.0)], seed=seed)
    assert diag["approach_failures"] > 0
    assert bool(disposed) is delivers, (diag, trace)
    if delivers:
        assert trace["end_xy"][0] > BOX[2]                  # at the east face
    else:
        assert diag["relaxed_sources"] > 0 and trace["end_xy"][1] > BOX[3] + 30


