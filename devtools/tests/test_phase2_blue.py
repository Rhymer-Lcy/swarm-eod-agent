"""Phase 2 blue changes: equivalence of the optimizations with the frozen
Phase 1 code, transport behaviour, message validation and harness modes."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.comparators.p1_ddc6897 import grid as old_grid  # noqa: E402
from devtools.comparators.p1_ddc6897 import motion as old_motion  # noqa: E402
from devtools.comparators.p1_ddc6897 import perception as old_perception  # noqa: E402
from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod.comms import make_message, numeric_tuple  # noqa: E402
from swarm_rescue.solutions.eod.grid import OccupancyGrid, Planner  # noqa: E402
from swarm_rescue.solutions.eod.motion import lookahead_point  # noqa: E402
from swarm_rescue.solutions.eod.observation import Observation, SemanticHit  # noqa: E402
from swarm_rescue.solutions.eod.perception import entity_mask, live_surface_points  # noqa: E402

ANGLES = np.linspace(-math.pi, math.pi, P.LIDAR_RAYS)


# ------------------------------------------------- optimization equivalence
def test_integrate_scan_matches_phase1_implementation():
    rng = np.random.default_rng(0)
    new, old = OccupancyGrid((500, 340)), old_grid.OccupancyGrid((500, 340))
    for _ in range(60):
        x, y, h = rng.uniform(-200, 200), rng.uniform(-130, 130), rng.uniform(-3, 3)
        ranges = rng.uniform(5.0, 300.0, P.LIDAR_RAYS)
        valid = rng.random(P.LIDAR_RAYS) > 0.2
        new.integrate_scan(x, y, h, ranges, ANGLES, valid)
        old.integrate_scan(x, y, h, ranges, ANGLES, valid)
    assert np.array_equal(new.logodds, old.logodds)
    assert np.array_equal(new.observed, old.observed)
    assert np.array_equal(new.occupied(), old.occupied())


def test_occupied_cache_is_read_only_and_tracks_version():
    grid = OccupancyGrid((200, 200))
    first = grid.occupied()
    assert not first.flags.writeable
    with pytest.raises(ValueError):
        first[0, 0] = True
    grid.mark_box(-20, -20, 20, 20)
    assert grid.occupied().sum() > 0 and first.sum() == 0


def test_clear_along_matches_phase1_implementation():
    rng = np.random.default_rng(1)
    new, old = OccupancyGrid((400, 400)), old_grid.OccupancyGrid((400, 400))
    for g in (new, old):
        g.logodds[:, :] = P.LOGODDS_MIN
        g.logodds[10:40, 25] = P.LOGODDS_MAX
        g.version += 1
    for _ in range(300):
        a = rng.uniform(-190, 190, 4)
        stop = float(rng.choice([0.0, 24.0]))
        assert new.clear_along(*a, stop_short=stop) == old.clear_along(*a, stop_short=stop)


def test_lookahead_point_matches_phase1_implementation():
    rng = np.random.default_rng(2)
    for _ in range(300):
        path = [tuple(p) for p in np.cumsum(rng.normal(0, 12, (rng.integers(1, 30), 2)), axis=0)]
        x, y = rng.normal(0, 30, 2)
        blocked = set(rng.choice(len(path), size=len(path) // 2, replace=False).tolist())

        def visible(px, py, path=path, blocked=blocked):
            return path.index((px, py)) not in blocked
        assert lookahead_point(path, x, y, 60.0, visible) == \
            old_motion.lookahead_point(path, x, y, 60.0, visible)


def test_entity_mask_matches_phase1_implementation():
    rng = np.random.default_rng(3)
    kinds = ("BOMB", "DRONE", "DISPOSAL_CENTER", "OTHER")
    for _ in range(100):
        sem = [SemanticHit(float(rng.uniform(5, 200)), float(rng.uniform(-3.1, 3.1)),
                           kinds[int(rng.integers(4))], False) for _ in range(rng.integers(0, 12))]
        lidar = rng.uniform(5, 300, P.LIDAR_RAYS)
        for sel in (None, ("DISPOSAL_CENTER",), ("DRONE", "BOMB", "OTHER")):
            assert np.array_equal(entity_mask(sem, lidar, ANGLES, kinds=sel),
                                  old_perception.entity_mask(sem, lidar, ANGLES, kinds=sel))


# ------------------------------------------------------------- transport
def test_carry_field_prefers_observed_free_space():
    grid = OccupancyGrid((400, 400))
    grid.logodds[:, :] = P.LOGODDS_MIN                    # all observed free
    # Unknown band 40 px wide (5 cells) from the bottom edge up to y = 150,
    # leaving observed free space above it. Crossing the band costs about
    # 5 x (c - 1) extra cells, the detour over it about 19 extra cells:
    # crossing wins at c = 3 (10 < 19), the detour at c = 10 (45 > 19).
    r0, c0 = grid.to_cell(-20.0, 150.0)
    r1, c1 = grid.to_cell(20.0, -200.0)
    grid.logodds[r0:r1 + 1, c0:c1] = 0.0
    grid.version += 1
    planner = Planner(grid)
    goal = grid.to_cell(150.0, 0.0)
    start = grid.to_cell(-150.0, 0.0)
    for cost, through in ((3.0, True), (10.0, False)):
        field = planner.field([goal], allow_unknown=True, unknown_cost=cost)
        path = planner.descend(field, start)
        crosses = any(grid.unknown()[r, c] for r, c in path)
        assert crosses is through, cost


def test_live_disposal_center_points_follow_current_pose():
    hits = [SemanticHit(50.0 + P.SEMANTIC_BIAS - 2.0, 0.0, "DISPOSAL_CENTER", False),
            SemanticHit(80.0, 1.0, "BOMB", False)]
    pts = live_surface_points(10.0, -5.0, math.pi / 2, hits)
    assert len(pts) == 1
    assert pts[0] == pytest.approx((10.0, 45.0))
    assert live_surface_points(0.0, 0.0, 0.0, None) == []


def _obs(step, messages=(), grasping=False, semantic=None):
    lidar = np.full(P.LIDAR_RAYS, 290.0)
    return Observation(step=step, walltime=0.0, identifier=0, n_drones=3,
                       size_area=(400, 300), max_steps=1000, max_walltime=60.0,
                       gps=(0.0, 0.0), compass=0.0, odometer=(0.0, 0.0, 0.0),
                       lidar=lidar, lidar_angles=ANGLES, semantic=semantic or [],
                       grasping=grasping, health=50, inventory=0,
                       messages=list(messages))


def test_blue_skips_duplicate_and_malformed_digest_items():
    from swarm_rescue.solutions.eod.blue import RescueBrain
    brain = RescueBrain(0, 3, (400, 300), 1000)
    brain.step(_obs(1))
    good = make_message(1, 5, (50.0, 0.0), b=((100.0, 20.0, 0, 5),))
    bad = make_message(2, 5, (60.0, 0.0), b=((float("nan"), 1.0, 0, 5), ("x", 1, 0, 2), (1.0,)),
                       dp=((1.0, 2.0),))
    brain.step(_obs(2, [good, bad, good]))
    assert len(brain.bombs.items) == 1
    brain.step(_obs(3, [good]))                           # repeated item: skipped
    assert len(brain.bombs.items) == 1
    assert numeric_tuple((1, 2.5, 3, 4), 4) and not numeric_tuple((True, 1, 2, 3), 4)


def test_blue_delivery_aims_at_live_detection():
    from swarm_rescue.solutions.eod.blue import RescueBrain
    brain = RescueBrain(0, 1, (400, 300), 1000)
    brain.step(_obs(1))
    # A stored (stale) center model far to the left, live detections ahead.
    for x in range(-150, -100, 5):
        brain.dc.add(float(x), 0.0)
    brain.carried = None
    sem = [SemanticHit(40.0 + P.SEMANTIC_BIAS - 2.0, a, "DISPOSAL_CENTER", False)
           for a in (-0.1, 0.0, 0.1)]
    brain.step(_obs(2, grasping=True, semantic=sem))      # grasp event
    brain.dc.points = [(-120.0, 0.0)] * 20                # model points far away
    brain.dc.box = (-20.0, -40.0, 60.0, 40.0)             # within delivery range
    brain.dc._dirty = False
    brain.step(_obs(3, grasping=True, semantic=sem))
    assert brain.state == "deliver"
    assert brain.diag["live_aims"] >= 1
    assert brain.target[0] > 30.0                         # the live point ahead, not x = -120


# ------------------------------------------------------------- harness
def test_runner_native_mode_validation_and_labels():
    from devtools.simlab import runner
    args = runner.build_parser().parse_args(
        ["--label", "x", "--map", "Map01", "--mode", "rescue", "--seed", "1", "--native"])
    assert runner.run_mode(args) == "native"
    assert args.phase == "phase4" and args.budget == 24
    assert runner.main(["--label", "x", "--map", "Map01", "--mode", "rescue", "--seed", "1",
                        "--native", "--agent", "devtools.comparators.p1_ddc6897.shell:X",
                        "--bombs-file", "b.json"]) == 2
    assert runner.main(["--label", "x", "--map", "Map01", "--mode", "rescue", "--seed", "1",
                        "--native", "--no-competition", "--bombs-file", "b.json"]) == 2
    plain = runner.build_parser().parse_args(
        ["--label", "x", "--map", "Map01", "--mode", "rescue", "--seed", "1", "--no-truth-log"])
    assert runner.run_mode(plain) == "harness"
    inst = runner.build_parser().parse_args(
        ["--label", "x", "--map", "Map01", "--mode", "rescue", "--seed", "1"])
    assert runner.run_mode(inst) == "instrumented"


def test_runner_budget_refuses_when_phase_is_used(tmp_path):
    import json
    from devtools.simlab import runner
    ledger = tmp_path / "ledger.jsonl"
    with ledger.open("w", encoding="utf-8") as f:
        for i in range(24):
            f.write(json.dumps({"event": "started", "phase": "phase4"}) + "\n")
        # Earlier phases' rounds never count against the default (phase4) budget.
        f.write(json.dumps({"event": "started", "phase": "phase3"}) + "\n")
        f.write(json.dumps({"event": "started", "phase": "phase1"}) + "\n")
    assert runner.ledger_rounds(ledger, "phase4") == 24
    rc = runner.main(["--label", "x", "--map", "Map01", "--mode", "place", "--seed", "1",
                      "--runs-root", str(tmp_path)])
    assert rc == 3
    assert runner.ledger_rounds(ledger, "phase4") == 24   # nothing was started
    # A custom-geometry name is refused before anything starts when unknown.
    assert runner.main(["--label", "x", "--map", "P4NoSuchV1", "--mode", "place", "--seed", "1",
                        "--runs-root", str(tmp_path)]) == 2
    assert runner.ledger_rounds(ledger, "phase4") == 24


def test_occupancy_cpu_only_flags():
    from devtools.simlab import occupancy
    parser_args = ["--cpus", "0-1", "--cpu-only", "--egl-device", "3"]
    with pytest.raises(SystemExit):
        occupancy.main(parser_args)                       # mutually exclusive
    assert occupancy.parse_cpu_list("0-2,32") == [0, 1, 2, 32]
