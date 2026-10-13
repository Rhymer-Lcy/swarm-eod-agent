"""Unit and offline-integration tests for the agent package ``solutions/eod``.

None of these tests needs an OpenGL context. Integration tests run the
brains in the offline approximation on a small synthetic arena, so they do
not depend on locally extracted fixtures.
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

from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod.comms import TeamTable, make_message, valid_messages  # noqa: E402
from swarm_rescue.solutions.eod.estimator import PoseEstimator  # noqa: E402
from swarm_rescue.solutions.eod.geometry import (  # noqa: E402
    body_to_world, clip_unit_vector, world_to_body, wrap)
from swarm_rescue.solutions.eod.grid import OccupancyGrid, Planner, _wavefront  # noqa: E402
from swarm_rescue.solutions.eod.motion import (  # noqa: E402
    max_speed_for_clearance, stopping_distance, thrust_for_velocity)
from swarm_rescue.solutions.eod.observation import SemanticHit  # noqa: E402
from swarm_rescue.solutions.eod.perception import (  # noqa: E402
    DisposalCenterModel, bomb_detections, entity_mask)
from swarm_rescue.solutions.eod.wallmap import (  # noqa: E402
    WallEvidence, decode_snapshot, merge, render)

SOLUTIONS = REPO / "src" / "swarm_rescue" / "solutions"
ANGLES = np.linspace(-math.pi, math.pi, P.LIDAR_RAYS)


# --------------------------------------------------------------- geometry
def test_wrap_and_frames_round_trip():
    assert wrap(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap(-0.5) == pytest.approx(-0.5)
    for heading in (-2.0, 0.0, 0.7, 3.0):
        fx, fy = world_to_body(3.0, -4.0, heading)
        wx, wy = body_to_world(fx, fy, heading)
        assert (wx, wy) == pytest.approx((3.0, -4.0))
    # Positive lateral thrust points to the left of the heading.
    assert body_to_world(0.0, 1.0, 0.0) == pytest.approx((0.0, 1.0))


def test_clip_unit_vector_matches_simulator_rule():
    assert clip_unit_vector(2.0, 0.0) == pytest.approx((1.0, 0.0))
    fx, fy = clip_unit_vector(1.0, 1.0)
    assert math.hypot(fx, fy) == pytest.approx(1.0)
    assert clip_unit_vector(0.3, -0.4) == pytest.approx((0.3, -0.4))


# -------------------------------------------------------------- estimator
def _simulate(estimator, commands, start=(0.0, 0.0, 0.3), gps=True, compass=True):
    """Exact model, noise-free sensors; returns true and estimated poses."""
    x, y, h = start
    vx = vy = 0.0
    estimator.update((x, y) if gps else None, h if compass else None, (0.0, 0.0, 0.0))
    for f, l, r in commands:
        estimator.set_command(f, l, r, carrying=False)
        ff, ll = clip_unit_vector(f, l)
        ux, uy = body_to_world(ff, ll, h)
        nx = x + P.POS_VEL * vx + P.POS_GAIN * ux
        ny = y + P.POS_VEL * vy + P.POS_GAIN * uy
        vx = P.VEL_DECAY * vx + P.VEL_GAIN * ux
        vy = P.VEL_DECAY * vy + P.VEL_GAIN * uy
        nh = wrap(h + P.ROT_GAIN * r)
        d = math.hypot(nx - x, ny - y)
        alpha = wrap(math.atan2(ny - y, nx - x) - h) if d > 0 else 0.0
        odo = (d, alpha, wrap(nh - h))
        x, y, h = nx, ny, nh
        estimator.update((x, y) if gps else None, h if compass else None, odo)
    return (x, y, h), (estimator.x, estimator.y, estimator.heading)


def test_estimator_tracks_exact_motion():
    cmds = [(1.0, 0.0, 0.0)] * 30 + [(0.0, 0.5, 0.5)] * 20 + [(-1.0, 0.2, -1.0)] * 20
    truth, est = _simulate(PoseEstimator(), cmds)
    assert est[0] == pytest.approx(truth[0], abs=1e-6)
    assert est[1] == pytest.approx(truth[1], abs=1e-6)
    assert wrap(est[2] - truth[2]) == pytest.approx(0.0, abs=1e-6)


def test_estimator_dead_reckons_through_gps_and_compass_outage():
    est = PoseEstimator()
    est.update((0.0, 0.0), 0.3, (0.0, 0.0, 0.0))
    cmds = [(1.0, 0.0, 0.2)] * 40
    # Continue without GPS or compass: the model alone must keep the pose.
    truth, got = _simulate(est, cmds, start=(est.x, est.y, est.heading), gps=False,
                           compass=False)
    assert math.hypot(got[0] - truth[0], got[1] - truth[1]) < 1e-6
    assert est.steps_without_gps == 41      # the initial update plus 40 steps


def test_estimator_waits_for_initial_fix():
    est = PoseEstimator()
    est.update(None, 0.1, None)
    assert not est.initialized
    est.update((5.0, 6.0), 0.1, None)
    assert est.initialized and (est.x, est.y) == (5.0, 6.0)


# ---------------------------------------------------------- grid / planner
def _box_scan(grid, x, y, half=100.0):
    """Lidar ranges from (x, y) inside an axis-aligned square room."""
    ranges = []
    for a in ANGLES:
        c, s = math.cos(a), math.sin(a)
        tx = (half - x) / c if c > 1e-9 else ((-half - x) / c if c < -1e-9 else 1e9)
        ty = (half - y) / s if s > 1e-9 else ((-half - y) / s if s < -1e-9 else 1e9)
        ranges.append(min(tx, ty, 300.0))
    return np.array(ranges)


def test_integrate_scan_marks_walls_and_free_space():
    grid = OccupancyGrid((400, 400))
    ranges = _box_scan(grid, 0.0, 0.0)
    for _ in range(3):
        grid.integrate_scan(0.0, 0.0, 0.0, ranges, ANGLES, np.ones(len(ranges), bool))
    r, c = grid.to_cell(0.0, 0.0)
    assert grid.free()[r, c]
    r, c = grid.to_cell(101.0, 0.0)
    assert grid.occupied()[r, c]
    r, c = grid.to_cell(160.0, 0.0)
    assert not grid.free()[r, c]           # beyond the wall: never observed
    assert grid.frontier().sum() == 0      # closed room: no free/unknown border


def test_carve_only_rays_add_no_obstacle():
    grid = OccupancyGrid((400, 400))
    ranges = np.full(len(ANGLES), 50.0)
    grid.integrate_scan(0.0, 0.0, 0.0, ranges, ANGLES, np.zeros(len(ranges), bool))
    assert not grid.occupied().any()


def test_planner_routes_around_a_wall_and_fallback_agrees():
    grid = OccupancyGrid((400, 400))
    grid.logodds[:, :] = P.LOGODDS_MIN            # all free
    r0, c = grid.to_cell(0.0, 120.0)
    r1, _ = grid.to_cell(0.0, -60.0)
    grid.logodds[r0:r1 + 1, c] = P.LOGODDS_MAX    # vertical wall at x = 0
    planner = Planner(grid)
    start = grid.to_cell(-100.0, 0.0)
    goal = grid.to_cell(100.0, 0.0)
    field = planner.field([goal], allow_unknown=False)
    assert np.isfinite(field[start])
    path = planner.descend(field, start)
    assert path[-1] == goal
    # The path must pass below the wall end (y < -60).
    ys = [grid.to_world(r, cc)[1] for r, cc in path]
    assert min(ys) < -60.0
    wave = _wavefront(planner.cost(False), [goal])
    assert np.isfinite(wave[start]) == np.isfinite(field[start])


# ------------------------------------------------------------------ motion
def test_stopping_distance_and_speed_limit_are_consistent():
    assert stopping_distance(0.0) == 0.0
    assert stopping_distance(11.0) > stopping_distance(5.0) > 0.0
    for clearance in (20.0, 40.0, 80.0):
        v = max_speed_for_clearance(clearance)
        assert stopping_distance(v) <= clearance - 6.0 + 0.5


def test_thrust_for_velocity_reaches_target_in_one_step():
    vx, vy, heading = 2.0, -1.0, 0.8
    target = (2.1, -0.8)                   # reachable within unit thrust
    f, l = thrust_for_velocity(vx, vy, heading, *target, carrying=False)
    ux, uy = body_to_world(f, l, heading)
    nvx = P.VEL_DECAY * vx + P.VEL_GAIN * ux
    nvy = P.VEL_DECAY * vy + P.VEL_GAIN * uy
    assert (nvx, nvy) == pytest.approx(target, abs=1e-6)


# ------------------------------------------------------------------ comms
def test_valid_messages_filters_foreign_and_own():
    good = make_message(3, 10, (1.0, 2.0), x=1)
    msgs = [good, make_message(1, 10, (0.0, 0.0)), {"v": "other", "id": 4, "p": (0, 0)},
            ("probe", 1), None, {"v": "eod1", "id": "x", "p": (0, 0)}]
    out = valid_messages(msgs, own_id=1)
    assert out == [good]


def test_team_table_keeps_newest():
    table = TeamTable()
    table.update(make_message(2, 50, (5.0, 5.0)), now=51)
    table.update(make_message(2, 40, (9.0, 9.0)), now=52)   # older content
    assert table.members[2].x == 5.0
    assert table.positions(now=53) == [(5.0, 5.0)]
    assert table.positions(now=80) == []


# -------------------------------------------------------------- perception
def test_bomb_detection_uses_semantic_and_lidar():
    # Bomb centre at range 80 straight ahead of a drone at the origin.
    surface = 80.0 - P.BOMB_RADIUS
    sem = [SemanticHit(surface + P.SEMANTIC_BIAS, 0.0, "BOMB", False),
           SemanticHit(150.0, 1.5, "DRONE", False)]
    lidar = np.full(len(ANGLES), 290.0)
    lidar[np.abs(ANGLES) < 0.1] = surface
    dets = bomb_detections(0.0, 0.0, 0.0, sem, lidar, ANGLES)
    assert len(dets) == 1
    assert dets[0].x == pytest.approx(80.0, abs=1.5)
    assert dets[0].y == pytest.approx(0.0, abs=2.0)
    mask = entity_mask(sem, lidar, ANGLES)
    assert mask[np.abs(ANGLES) < 0.05].all() and not mask[np.abs(ANGLES) > 0.4].all()


def test_disposal_center_model_box_and_distance():
    dc = DisposalCenterModel()
    for x in range(-50, 51, 10):
        dc.add(float(x), 100.0)
    assert dc.known
    assert dc.distance(0.0, 80.0) == pytest.approx(20.0, abs=1.0)
    assert dc.nearest_surface(48.0, 0.0) == pytest.approx((50.0, 100.0))


# ----------------------------------------------------------------- bombs
def test_bomb_table_merge_rules():
    from swarm_rescue.solutions.eod.blue import BombTable
    table = BombTable()
    table.observe_grasped(10.0, 10.0, step=5)          # unknown grasped: ignored
    assert table.items == []
    b = table.observe_free(10.0, 10.0, step=6)
    table.merge_remote(12.0, 9.0, status=2, seen=4)     # older: ignored
    assert b.status == 0
    table.merge_remote(12.0, 9.0, status=2, seen=9)     # newer: wins
    assert b.status == 2
    table.observe_free(11.0, 10.0, step=12)             # seen again: free
    assert b.status == 0 and len(table.items) == 1


# ---------------------------------------------------------------- wall map
def test_evidence_snapshot_round_trip_and_render():
    ev = WallEvidence((200, 120))
    ranges = np.full(len(ANGLES), 290.0)
    sel = np.abs(ANGLES) < 0.5
    ranges[sel] = 50.0 / np.cos(ANGLES[sel])           # wall at x = 50
    for _ in range(5):
        ev.add_returns(0.0, 0.0, 0.0, ranges, ANGLES, np.ones(len(ranges), bool))
    grid = decode_snapshot(ev.snapshot(), (120, 200))
    assert grid is not None and np.array_equal(grid, ev.counts.astype(np.float32))
    total = merge(ev, [ev.snapshot()])
    assert np.array_equal(total, 2 * ev.counts.astype(np.float32))
    wall = render(total)
    col = int(round(50 + 100))
    assert wall[60, col - 2:col + 3].any()             # the wall line is drawn
    assert not wall[60, 100:140].any()                 # free space stays free


def test_render_keeps_border_next_to_disposal_center():
    ev = np.zeros((120, 200), np.float32)
    ev[2:5, :] = 3.0                                   # top border observed
    wall = render(ev, dc_box=(-100.0, 20.0, -40.0, 60.0))
    assert wall[:6, 10:50].any()


# ---------------------------------------------------- compliance (static)
FORBIDDEN_CALLS = {"true_position", "true_angle", "true_velocity", "true_angular_velocity"}
FORBIDDEN_MODULES = {"gc", "ctypes", "inspect"}


def _agent_sources():
    return sorted((SOLUTIONS / "eod").glob("*.py")) + [SOLUTIONS / "my_drone_eval.py"]


def test_agent_code_uses_no_ground_truth_or_runtime_patching():
    for path in _agent_sources():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                assert node.func.attr not in FORBIDDEN_CALLS, path
                assert node.func.attr != "setattr", path
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                assert node.func.id not in {"setattr", "exec", "eval", "print"}, path
            if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
                    and node.value.id == "self":
                assert node.attr not in {"position", "angle", "velocity",
                                         "angular_velocity"}, (path, node.attr)
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                names = [a.name for a in node.names] + ([node.module] if
                                                         isinstance(node, ast.ImportFrom)
                                                         and node.module else [])
                for name in names:
                    assert name.split(".")[0] not in FORBIDDEN_MODULES, (path, name)
                    assert "devtools" not in name, (path, name)
                    assert name != "sys", (path, name)
        text = path.read_text(encoding="utf-8")
        assert "_playground" not in text and "sys.modules" not in text, path


def test_entry_point_selects_agent_classes():
    from swarm_rescue.solutions.eod.shell import EodPlaceDrone, EodRescueDrone
    from swarm_rescue.solutions.my_drone_eval import drone_class_for_mode
    assert drone_class_for_mode("rescue") is EodRescueDrone
    assert drone_class_for_mode("place") is EodPlaceDrone


# ------------------------------------------------------ offline integration
def _arena():
    walls = np.zeros((400, 600), np.uint8)
    walls[:6, :] = walls[-6:, :] = 1
    walls[:, :6] = walls[:, -6:] = 1
    walls[150:400, 300:306] = 1                         # partition from the bottom
    boundary = np.zeros_like(walls)
    boundary[:6, :] = boundary[-6:, :] = 1
    boundary[:, :6] = boundary[:, -6:] = 1
    return {"map": "Arena", "walls": walls, "boundary": boundary,
            "obstacle_render": walls, "starts": np.zeros((0, 2)),
            "size_area": [600, 400], "max_timestep_limit": 900,
            "max_walltime_limit": 60,
            "disposal_center": {"x": -220.0, "y": 130.0, "width": 100.0, "height": 60.0},
            "start_area": [-200.0, 30.0], "default_drones": 4}


def test_blue_brains_dispose_bombs_offline():
    from devtools.simlab.offline_sim import OfflineWorld
    from swarm_rescue.solutions.eod.blue import RescueBrain
    fx = _arena()
    world = OfflineWorld(fx, "rescue", 4,
                         lambda i: RescueBrain(i, 4, (600, 400), 900), seed=2,
                         bombs=[(-60.0, -120.0), (100.0, 120.0)], max_steps=900)
    out = world.run()
    assert out["disposed"] >= 1
    assert out["removed"] == 0


def test_red_brains_place_and_submit_offline():
    from devtools.simlab.offline_sim import OfflineWorld
    from swarm_rescue.solutions.eod.red import PlaceBrain
    fx = _arena()
    brains = []

    def factory(i):
        brains.append(PlaceBrain(i, 3, (600, 400), 900, 60.0))
        return brains[-1]

    world = OfflineWorld(fx, "place", 3, factory, seed=3, bombs_per_drone=1,
                         max_steps=900)
    world.run()
    assert world.submitted is not None
    assert world.submitted.shape == (400, 600) and world.submitted.dtype == np.uint8
    assert world.submit_step <= 450
    assert len(world.bombs) == 3
    # The submitted map must carry real walls and stay off the free interior.
    assert (world.submitted & fx["walls"]).sum() > 0.3 * fx["walls"].sum()
    assert world.submitted[150:250, 100:250].sum() == 0
