"""B2 delivery to partly walled disposal centers (Phase 3).

Regression tests for the Phase 2 Map03 failure mechanism on synthetic
geometry (``devtools/simlab/delivery_fixtures.py``, no official map
coordinates): a disposal center whose south and west faces lie 2 px from
long walls. They check route accessibility, terminal line of sight, forward
progress and terminal outcomes, and run the frozen B1 (``p2_540054b``) on the
same inputs where the old behaviour must fail.
"""

from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.comparators.p2_540054b import blue as b1_blue  # noqa: E402
from devtools.simlab import delivery_fixtures as DF  # noqa: E402
from swarm_rescue.solutions.eod import blue as b2_blue  # noqa: E402
from swarm_rescue.solutions.eod import delivery as D  # noqa: E402
from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod.grid import OccupancyGrid  # noqa: E402
from swarm_rescue.solutions.eod.motion import LocalView  # noqa: E402
from swarm_rescue.solutions.eod.observation import Observation  # noqa: E402

ANGLES = np.linspace(-math.pi, math.pi, P.LIDAR_RAYS)
DC_BOX = (DF.DC["x"] - DF.DC["width"] / 2, DF.DC["y"] - DF.DC["height"] / 2,
          DF.DC["x"] + DF.DC["width"] / 2, DF.DC["y"] + DF.DC["height"] / 2)   # -140,-40,20,40


# ---------------------------------------------------------------- helpers
def _truth_solid(scenario: str) -> np.ndarray:
    """Walls plus the disposal center, pixel raster (test ground truth)."""
    fx = DF.fixture(scenario)
    solid = fx["walls"].astype(bool).copy()
    r0, c0 = DF._px_row(DC_BOX[3]), DF._px_col(DC_BOX[0])
    r1, c1 = DF._px_row(DC_BOX[1]), DF._px_col(DC_BOX[2])
    solid[r0:r1, c0:c1] = True
    return solid


def _truth_walls(scenario: str) -> np.ndarray:
    return DF.fixture(scenario)["walls"].astype(bool)


def _segment_hits(raster: np.ndarray, x0, y0, x1, y1) -> bool:
    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 2
    for t in np.linspace(0.0, 1.0, n):
        x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        r, c = DF._px_row(y), DF._px_col(x)
        if 0 <= r < raster.shape[0] and 0 <= c < raster.shape[1] and raster[r, c]:
            return True
    return False


def _paint(grid: OccupancyGrid, raster: np.ndarray, known: np.ndarray = None) -> None:
    """Agent map with every cell observed, occupied where the raster is solid."""
    k = int(grid.cell)
    pad = np.zeros((grid.ny * k, grid.nx * k), bool)
    pad[:raster.shape[0], :raster.shape[1]] = raster
    occ = pad.reshape(grid.ny, k, grid.nx, k).any(axis=(1, 3))
    grid.logodds[:, :] = P.LOGODDS_MIN
    grid.logodds[occ] = P.LOGODDS_MAX
    grid.observed[:, :] = True
    if known is not None:              # leave cells outside ``known`` unobserved
        grid.logodds[~known] = 0.0
        grid.observed[~known] = False
    grid.version += 1


def _north_east_views():
    """Surface points seen from the open north and east sides, with observers."""
    views = []
    for ox in range(-150, 40, 20):                    # observers north of the center
        pts = [(float(px), DC_BOX[3] - 2.0) for px in range(-136, 20, 8)
               if abs(px - ox) < 120]
        views.append(((float(ox), 120.0), pts))
    for oy in range(-30, 60, 15):                      # observers east of the center
        pts = [(DC_BOX[2] - 2.0, float(py)) for py in range(-38, 40, 8)]
        views.append(((110.0, float(oy)), pts))
    return views


def _brain(module, scenario="corner", pose=(18.0, -75.0, 1.5708), known=None):
    brain = module.RescueBrain(0, 1, (DF.W, DF.H), 3000)
    _paint(brain.grid, _truth_solid(scenario), known)
    for (ox, oy), pts in _north_east_views():
        for p in pts:
            brain.dc.add(*p)
        if hasattr(brain, "approach"):
            brain.approach.observe(brain.grid, ox, oy, pts, 1)
    # Corner points of the stored model, as recorded with a few px of pose error.
    for p in ((18.0, -38.0), (-138.0, 38.0), (19.0, -35.0)):
        brain.dc.add(*p)
    brain.dc.bbox()
    est = brain.est
    est.x, est.y, est.heading = pose
    est.vx = est.vy = 0.0
    est.initialized = True
    est.heading_var = math.radians(2.0) ** 2
    brain.home = (-60.0, 120.0)
    return brain


def _obs(step, grasping=True, semantic=None):
    return Observation(step=step, walltime=0.0, identifier=0, n_drones=1,
                       size_area=(DF.W, DF.H), max_steps=3000, max_walltime=600.0,
                       gps=None, compass=None, odometer=None,
                       lidar=np.full(P.LIDAR_RAYS, 290.0), lidar_angles=ANGLES,
                       semantic=semantic if semantic is not None else [],
                       grasping=grasping, health=50, inventory=0)


def _carry_step(brain, step, grasp_side=0.0):
    brain.carried = None
    brain.grasp_side = grasp_side
    view = LocalView(brain.est.heading, None, ANGLES)
    return brain._carry(_obs(step), view, [])


def _goal_cells_b1(brain, now=10):
    """Goal cells the frozen B1 builds for its carry field (its own code path)."""
    box = brain.dc.bbox()
    cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    cells = []
    for px, py in brain.dc.points[::5] or [(cx, cy)]:
        dx, dy = px - cx, py - cy
        norm = math.hypot(dx, dy) or 1.0
        cells.append(brain.grid.to_cell(px + 32.0 * dx / norm, py + 32.0 * dy / norm))
    return sorted(set(cells))


def _goal_valid(grid, cell, walls, solid):
    """A goal is valid when it is free and sees the center's surface past no wall."""
    x, y = grid.to_world(*cell)
    if solid[DF._px_row(y), DF._px_col(x)]:
        return False
    qx = min(max(x, DC_BOX[0]), DC_BOX[2])
    qy = min(max(y, DC_BOX[1]), DC_BOX[3])
    return not _segment_hits(walls, x, y, qx, qy)


# ------------------------------------------------------- goal construction
def test_b1_goals_include_cells_behind_walls_and_b2_goals_do_not():
    walls, solid = _truth_walls("corner"), _truth_solid("corner")
    b1 = _brain(b1_blue)
    b1_bad = [c for c in _goal_cells_b1(b1) if not _goal_valid(b1.grid, c, walls, solid)]
    assert b1_bad, "the frozen B1 should build goals behind the walls (Map03 mechanism)"
    b2 = _brain(b2_blue)
    goals = b2._dc_sources(10)                              # the goals B2's field uses
    assert len(goals) >= 20 and not b2._relaxed and b2.diag["radial_fallbacks"] == 0
    assert all(_goal_valid(b2.grid, c, walls, solid) for c in goals)


def test_open_center_goals_surround_every_observed_face():
    b2 = _brain(b2_blue, scenario="open")
    walls, solid = _truth_walls("open"), _truth_solid("open")
    goals, _ = b2.approach.sources(b2.grid, 10)
    pts = [b2.grid.to_world(*c) for c in goals]
    assert any(y > DC_BOX[3] + 20 for _, y in pts)        # north face
    assert any(x > DC_BOX[2] + 20 for x, _ in pts)        # east face
    assert all(_goal_valid(b2.grid, c, walls, solid) for c in goals)


def test_unreachable_or_contradicted_witness_is_rejected():
    grid = OccupancyGrid((400, 300))
    _paint(grid, np.zeros((300, 400), bool))
    s = D.ApproachSet()
    s.observe(grid, 0.0, 100.0, [(0.0, 20.0)], 1)          # stand-off at (0, 56)
    s.observe(grid, 120.0, 0.0, [(60.0, 0.0)], 1)           # stand-off at (96, 0)
    assert len(s.valid(grid, 2)) == 2
    grid.mark_box(-20.0, 34.0, 20.0, 42.0)                  # a wall now crosses the first segment
    grid.mark_box(90.0, -6.0, 102.0, 6.0)                   # the second stand-off is an obstacle
    assert s.valid(grid, 3) == []
    goals, relaxed = s.sources(grid, 3)
    assert goals == [] and not relaxed


def test_stale_center_observation_falls_back_to_checked_radial_goals():
    grid = OccupancyGrid((400, 300))
    raster = np.zeros((300, 400), bool)
    raster[150 - 20:150 + 20, 200 - 30:200 + 30] = True     # center 60 x 40 at the origin
    raster[150 + 24:150 + 30, 100:300] = True               # wall 4 px below its south face
    _paint(grid, raster)
    pts = [(float(x), -18.0) for x in range(-28, 30, 4)] + \
          [(float(x), 18.0) for x in range(-28, 30, 4)]
    cells = D.radial_sources(grid, pts, (-30.0, -20.0, 30.0, 20.0))
    walls = np.zeros_like(raster)
    walls[150 + 24:150 + 30, 100:300] = True
    for r, c in cells:
        x, y = grid.to_world(r, c)
        assert y > 0.0, (x, y)                              # never behind the south wall


def test_segments_clear_matches_clear_along():
    rng = np.random.default_rng(4)
    grid = OccupancyGrid((400, 400))
    grid.logodds[:, :] = P.LOGODDS_MIN
    grid.logodds[10:40, 25] = P.LOGODDS_MAX
    grid.logodds[30, 5:45] = P.LOGODDS_MAX
    grid.version += 1
    seg = rng.uniform(-190, 190, (300, 4))
    for stop in (0.0, 12.0):
        got = D.segments_clear(grid, seg[:, 0], seg[:, 1], seg[:, 2], seg[:, 3], stop_short=stop)
        want = [grid.clear_along(*s, stop_short=stop) for s in seg]
        assert got.tolist() == want


def test_sparse_map_still_yields_reachable_goals():
    known = np.zeros((63, 88), bool)                         # grid of a 700 x 500 arena
    known[:20, :] = True                                     # only the northern band was mapped
    b2 = _brain(b2_blue, known=known)
    assert b2.grid.observed.sum() == known.sum()
    goals, _ = b2.approach.sources(b2.grid, 10)
    assert goals
    field = b2.planner.field(goals, allow_unknown=True, step=10, unknown_cost=10.0)
    assert np.isfinite(field[b2.grid.to_cell(18.0, -75.0)])


# ------------------------------------------------- terminal line of sight
def test_carrier_behind_wall_within_50px_b1_pushes_b2_reroutes():
    walls = _truth_walls("corner")
    b1 = _brain(b1_blue)
    _carry_step(b1, 10)
    assert b1.state == "deliver"                            # B1 skips the check within 50 px
    assert _segment_hits(walls, b1.est.x, b1.est.y, *b1.target)
    b2 = _brain(b2_blue)
    _carry_step(b2, 10)
    assert b2.state == "carry"
    assert not _segment_hits(walls, b2.est.x, b2.est.y, *b2.target)
    assert b2.diag["terminal_blocked"] == 1


def test_b2_route_from_blocked_side_reaches_a_standoff_without_crossing_walls():
    b2 = _brain(b2_blue)
    solid, walls = _truth_solid("corner"), _truth_walls("corner")
    goals, _ = b2.approach.sources(b2.grid, 10)
    field = b2.planner.field(goals, allow_unknown=True, step=10, unknown_cost=10.0)
    path = b2.planner.descend(field, b2.grid.to_cell(18.0, -75.0), max_len=600)
    assert path[-1] in set(goals)
    pts = [b2.grid.to_world(r, c) for r, c in path]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        assert not _segment_hits(solid, x0, y0, x1, y1)
    # It goes around the east end of the south wall (x = 170) and ends in sight.
    assert max(x for x, _ in pts) > 170.0
    assert _goal_valid(b2.grid, path[-1], walls, solid)


def test_witnessed_standoff_certifies_a_terminal_push_without_live_view():
    b2 = _brain(b2_blue, pose=(50.0, 0.0, 0.0))            # on an east stand-off
    b2.carried = None
    aim = b2._terminal_aim(_obs(10))
    assert aim is not None and aim[0] == pytest.approx(DC_BOX[2] - 2.0)
    assert b2.diag["witness_aims"] == 1
    b2.est.x, b2.est.y = 18.0, -75.0                        # behind the wall: none
    assert b2._terminal_aim(_obs(11)) is None


# ---------------------------------------------- progress, retries, waiting
def test_no_progress_penalizes_the_approach_and_replans():
    b2 = _brain(b2_blue, pose=(50.0, 0.0, 0.0))
    b2.dc_live = []
    goals0, _ = b2.approach.sources(b2.grid, 1)
    for step in range(1, b2_blue.APPROACH_PATIENCE + 3):
        b2._watch_progress(_obs(step), 28.0)                # stationary, no gain
    assert b2.diag["approach_failures"] == 1
    goals1, relaxed = b2.approach.sources(b2.grid, b2_blue.APPROACH_PATIENCE + 3)
    assert not relaxed and len(goals1) < len(goals0)
    near = [c for c in goals1 if math.hypot(b2.grid.to_world(*c)[0] - 50.0,
                                            b2.grid.to_world(*c)[1]) < D.PENALTY_RADIUS - 8]
    assert near == []
    # Inside the failed area there is no terminal push.
    assert b2._terminal_aim(_obs(b2_blue.APPROACH_PATIENCE + 4)) is None
    # The exclusion expires.
    later = b2_blue.APPROACH_PATIENCE + 3 + D.PENALTY_STEPS + 1
    assert len(b2.approach.sources(b2.grid, later)[0]) == len(goals0)


def test_travelling_carrier_is_not_penalized():
    b2 = _brain(b2_blue, pose=(-60.0, -80.0, 0.0))
    for step in range(1, 400):
        b2.est.x = -60.0 + 0.6 * step                       # moving east along the wall
        b2._watch_progress(_obs(step), 40.0 + 0.1 * step)
    assert b2.diag["approach_failures"] == 0


def test_repeated_failures_escalate_and_never_leave_no_goal():
    grid = OccupancyGrid((400, 300))
    _paint(grid, np.zeros((300, 400), bool))
    s = D.ApproachSet()
    s.observe(grid, 0.0, 100.0, [(0.0, 20.0)], 1)
    assert s.penalize(0.0, 56.0, 10) == D.PENALTY_STEPS
    assert s.penalize(0.0, 56.0, 20) == 2 * D.PENALTY_STEPS
    assert s.penalize(0.0, 56.0, 30) == 4 * D.PENALTY_STEPS
    assert s.penalize(0.0, 56.0, 40) == D.PENALTY_MAX_STEPS
    goals, relaxed = s.sources(grid, 50)
    assert relaxed and len(goals) == 1                      # every goal failed: try again


def test_idle_drone_waits_clear_of_the_center_on_a_reachable_cell():
    b2 = _brain(b2_blue, pose=(-60.0, 120.0, 0.0))
    view = LocalView(0.0, None, ANGLES)
    for ident in range(4):
        b2.id = ident
        b2._idle_spot = None
        b2.path = []
        b2._idle(_obs(10, grasping=False), view, [])       # the waiting behaviour itself
        aim = b2.path_goal
        assert b2.state == "idle"
        assert b2.dc.distance(*aim) >= b2_blue.IDLE_DC_CLEARANCE
        r, c = b2.grid.to_cell(*aim)
        assert b2.planner.cost(False, 10)[r, c] > 0
        assert np.isfinite(b2._field_from_me(10)[r, c])


def test_grasper_stays_on_through_penalty_and_replanning():
    b2 = _brain(b2_blue)
    for step in range(1, 300):
        cmd = _carry_step(b2, step)
        assert cmd.grasper == 1
    assert b2.state == "carry"


def test_delivery_module_reads_only_agent_inputs():
    path = REPO / "src" / "swarm_rescue" / "solutions" / "eod" / "delivery.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            imported.add(node.module or "")
        elif isinstance(node, ast.Import):
            imported.update(a.name for a in node.names)
    assert imported <= {"__future__", "math", "typing", "numpy", "grid"}, imported
    text = path.read_text(encoding="utf-8")
    assert "true_" not in text and "fixture" not in text.lower()


# ------------------------------------------------- closed loop (offline)
def _teleport(world, brain, at):
    """Move the simulated drone and, consistently, the brain's estimate."""
    d = world.drones[0]
    d.x, d.y = at
    d.prev_x, d.prev_y, d.vx, d.vy = d.x, d.y, 0.0, 0.0
    brain.est.x, brain.est.y, brain.est.vx, brain.est.vy = d.x, d.y, 0.0, 0.0
    brain.path, brain.target = [], None


def _teleport_carrier(module, at, seed=1, steps=1200):
    """Offline world: learn the center from two sides, then carry from ``at``.

    The drone first sees the center from the start area (north) and from an
    east viewpoint near the walled corner, as the Map03 drones saw theirs
    while working around it. The harness then moves the drone (and,
    consistently, its estimate) to ``at`` with a bomb attached, emulating a
    carrier that arrived there; the brain has to deliver on its own sensing.
    """
    from devtools.simlab.offline_sim import OfflineWorld
    fx = DF.fixture("corner", steps + 100)
    brains, commands = [], []

    class Recording:
        def __init__(self, brain):
            self.brain = brain

        def outgoing_message(self):
            return self.brain.outgoing_message()

        def step(self, obs):
            cmd = self.brain.step(obs)
            commands.append((obs.grasping, cmd.grasper))
            return cmd

    def factory(i):
        brains.append(module.RescueBrain(i, 1, (DF.W, DF.H), steps + 100))
        return Recording(brains[-1])

    world = OfflineWorld(fx, "rescue", 1, factory, seed=seed, bombs=[(250.0, -200.0)],
                         max_steps=steps + 100)
    for _ in range(40):                                   # sees the center from the north
        world.step()
    d, bomb, brain = world.drones[0], world.bombs[0], brains[0]
    _teleport(world, brain, (70.0, -25.0))                # east, next to the walled corner
    for _ in range(20):
        world.step()
    _teleport(world, brain, at)
    bomb.x = d.x + P.CARRY_OFFSET * math.cos(d.heading)
    bomb.y = d.y + P.CARRY_OFFSET * math.sin(d.heading)
    bomb.holder, d.carrying, d.grasp_angle = 0, 0, 0.0
    commands.clear()
    start = world.step_index
    while world.step() and world.disposed == 0 and world.step_index - start < steps:
        pass
    return world, brain, commands, world.step_index - start


@pytest.mark.parametrize("seed", [1, 2])
@pytest.mark.parametrize("at", [(18.0, -75.0), (-175.0, 10.0)], ids=["south", "west"])
def test_closed_loop_carrier_behind_wall_b1_fails_b2_delivers(at, seed):
    w1, _, _, _ = _teleport_carrier(b1_blue, at, seed=seed)
    assert w1.disposed == 0 and w1.drones[0].carrying is not None
    w2, brain, commands, took = _teleport_carrier(b2_blue, at, seed=seed)
    assert w2.disposed == 1, brain.diag
    assert took < 1000
    # The grasp held from the start of the carry until the disposal.
    held = [g for g, _ in commands]
    first_free = held.index(False) if False in held else len(held)
    assert all(grasper == 1 for _, grasper in commands[:first_free])
    assert first_free >= 20


def test_closed_loop_parked_teammate_on_the_approach():
    """A teammate that never moves sits in front of the north face."""
    from devtools.simlab.offline_sim import OfflineWorld
    fx = DF.fixture("open", 1300)

    class Parked:
        def outgoing_message(self):
            return None

        def step(self, obs):
            from swarm_rescue.solutions.eod.observation import Command
            return Command()

    brains = []

    def factory(i):
        if i == 1:
            return Parked()
        brains.append(b2_blue.RescueBrain(i, 2, (DF.W, DF.H), 1300))
        return brains[-1]

    world = OfflineWorld(fx, "rescue", 2, factory, seed=3,
                         bombs=[(-60.0, 200.0)], max_steps=1300)
    world.drones[1].x, world.drones[1].y = -60.0, 62.0      # 22 px from the north face
    world.drones[1].prev_x, world.drones[1].prev_y = -60.0, 62.0
    world.run()
    assert world.disposed == 1
