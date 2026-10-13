"""Regression tests for carry-aware heading estimation (B1).

A single drone is simulated with the measured motion model and the noise
models of the supplied simulator (GPS and compass AR(1), phi 0.98, sigma 5 px
and 4 degrees; odometer white noise 0.2 px, 8 and 1 degrees). While a bomb is
held the drone turns by a random fraction of the command drawn per step from
the spread measured in the grasp probe (median 0.39, 10th-90th percentile
-0.01 to 0.90). Commands come from a closed heading loop on the *estimated*
heading, as in the agent, which is what turns a small per-step prediction
error into a persistent offset: the controller stops turning once the
estimate says the target heading is reached.

The frozen Phase 1 estimator (``devtools.comparators.p1_ddc6897``) is run on
the same noise realization to show that each scenario reproduces the
original failure mechanism.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Callable, Dict, Optional

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.comparators.p1_ddc6897.estimator import PoseEstimator as FrozenEstimator  # noqa: E402
from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod.estimator import PoseEstimator  # noqa: E402
from swarm_rescue.solutions.eod.geometry import clip_unit_vector, wrap  # noqa: E402

PHI = 0.98
DEG = math.pi / 180.0


def _always(_step: int) -> bool:
    return True


def run_scenario(estimator_cls, *, steps: int, seed: int,
                 carrying: Callable[[int], bool] = lambda t: False,
                 odometer_on: Callable[[int], bool] = _always,
                 compass_on: Callable[[int], bool] = _always,
                 gps_on: Callable[[int], bool] = _always,
                 start_heading: float = 0.3,
                 targets: Optional[Callable[[int], float]] = None,
                 kick: Optional[Dict[int, float]] = None,
                 forward: float = 0.6,
                 legacy_api: bool = False) -> Dict[str, np.ndarray]:
    """Closed-loop run; returns per-step truth, estimate and flags."""
    rng = np.random.default_rng(seed)
    est = estimator_cls()
    x = y = vx = vy = 0.0
    h = start_heading
    gps_n = np.zeros(2)
    comp_n = 0.0
    gps_w = P.GPS_NOISE * math.sqrt(1 - PHI ** 2)
    comp_w = P.COMPASS_NOISE * math.sqrt(1 - PHI ** 2)
    if targets is None:
        def targets(t: int) -> float:
            return wrap(start_heading + 1.3 * ((t // 60) % 5) - 2.0)
    kick = kick or {}
    held_prev = carrying(0)

    def update(t: int, odo, held: bool) -> None:
        nonlocal gps_n, comp_n
        gps_n = PHI * gps_n + rng.normal(0.0, gps_w, 2)
        comp_n = PHI * comp_n + rng.normal(0.0, comp_w)
        gps = (x + gps_n[0], y + gps_n[1]) if gps_on(t) else None
        compass = wrap(h + comp_n) if compass_on(t) else None
        odo = odo if odometer_on(t) else None
        if legacy_api:
            est.update(gps, compass, odo)
        else:
            est.update(gps, compass, odo, grasping=held)

    update(0, (0.0, 0.0, 0.0), held_prev)
    rows = []
    for t in range(1, steps + 1):
        held = carrying(t - 1)
        r = max(-1.0, min(1.0, 0.8 * wrap(targets(t) - est.heading) / P.ROT_GAIN))
        f, l = clip_unit_vector(forward, 0.0)
        est.set_command(f, l, r, held)
        # Physics of the step.
        gain = P.CARRY_GAIN_FACTOR if held else 1.0
        ux, uy = f * math.cos(h) - l * math.sin(h), f * math.sin(h) + l * math.cos(h)
        nx = x + P.POS_VEL * vx + P.POS_GAIN * gain * ux
        ny = y + P.POS_VEL * vy + P.POS_GAIN * gain * uy
        vx = P.VEL_DECAY * vx + P.VEL_GAIN * gain * ux
        vy = P.VEL_DECAY * vy + P.VEL_GAIN * gain * uy
        ratio = 1.0
        if held:
            ratio = float(np.clip(rng.normal(0.39, 0.36), -0.2, 1.0))
        nh = wrap(h + ratio * P.ROT_GAIN * r + kick.get(t, 0.0))
        d = math.hypot(nx - x, ny - y)
        alpha = math.atan2(ny - y, nx - x) - h if d > 0 else 0.0
        odo = (d + rng.normal(0.0, 0.2), wrap(alpha + rng.normal(0.0, 8 * DEG)),
               wrap(nh - h + rng.normal(0.0, DEG)))
        x, y, h = nx, ny, nh
        held_now = carrying(t)
        update(t, odo, held_now)
        rows.append((t, x, y, h, est.x, est.y, est.heading, held or held_now))
    arr = np.array(rows)
    err = np.abs(np.arctan2(np.sin(arr[:, 6] - arr[:, 3]), np.cos(arr[:, 6] - arr[:, 3])))
    return {"t": arr[:, 0], "heading_err": err / DEG,
            "pos_err": np.hypot(arr[:, 4] - arr[:, 1], arr[:, 5] - arr[:, 2]),
            "carry": arr[:, 7] > 0, "est": est}


def _p95(values: np.ndarray) -> float:
    return float(np.percentile(values, 95))


def _rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(values ** 2)))


# ------------------------------------------------------------------ carrying
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_carried_rotation_with_noisy_odometry(seed):
    carry = lambda t: t >= 100        # noqa: E731
    old = run_scenario(FrozenEstimator, steps=1600, seed=seed, carrying=carry, legacy_api=True)
    new = run_scenario(PoseEstimator, steps=1600, seed=seed, carrying=carry)
    old_err = old["heading_err"][old["carry"]]
    new_err = new["heading_err"][new["carry"]]
    assert old_err.size == new_err.size == 1501
    # The failure mechanism is reproduced: a persistent offset while carrying.
    assert _p95(old_err) > 15.0
    # Carry-aware estimation keeps the error near the compass noise level.
    assert _p95(new_err) < 9.0
    assert _p95(new_err) < 0.5 * _p95(old_err)
    assert _rms(new_err) < 5.0
    assert _p95(new["pos_err"][new["carry"]]) < 14.0
    assert new["est"].carry_odometer_steps >= 1500


def test_free_flight_estimation_is_not_degraded():
    for seed in (4, 5, 6):
        old = run_scenario(FrozenEstimator, steps=1500, seed=seed, legacy_api=True)
        new = run_scenario(PoseEstimator, steps=1500, seed=seed)
        assert new["est"].carry_odometer_steps == 0
        assert _rms(new["heading_err"]) <= _rms(old["heading_err"]) + 0.2
        assert _p95(new["heading_err"]) <= _p95(old["heading_err"]) + 0.1
        assert _p95(new["pos_err"]) <= _p95(old["pos_err"]) + 0.5


def test_free_flight_matches_previous_estimator_exactly_without_contact():
    """No carrying and no model violation: the two estimators coincide."""
    old = run_scenario(FrozenEstimator, steps=400, seed=7, legacy_api=True)
    new = run_scenario(PoseEstimator, steps=400, seed=7)
    if new["est"].contact_rotation_steps:
        pytest.skip("a 3.5-sigma odometer outlier occurred in this realization")
    assert np.max(np.abs(new["heading_err"] - old["heading_err"])) < 1e-6
    assert np.max(np.abs(new["pos_err"] - old["pos_err"])) < 1e-6


def test_carrying_transitions_grasp_and_release():
    def carry(t: int) -> bool:
        return (t // 150) % 2 == 1      # alternate 150 steps free / carrying
    new = run_scenario(PoseEstimator, steps=1500, seed=8, carrying=carry)
    old = run_scenario(FrozenEstimator, steps=1500, seed=8, carrying=carry, legacy_api=True)
    t = new["t"].astype(int)
    edges = np.flatnonzero(np.diff([carry(int(v)) for v in t])) + 1
    assert len(edges) == 10
    near = np.zeros(len(t), dtype=bool)
    for e in edges:
        near[max(0, e - 3):e + 20] = True
    assert _p95(new["heading_err"][near]) < 10.0
    assert _p95(new["heading_err"]) < 9.0
    assert _p95(old["heading_err"]) > 2.0 * _p95(new["heading_err"])


def test_released_drone_recovers_heading_quickly():
    """After a long carry, the free-flight error must not linger."""
    carry = lambda t: 100 <= t < 700          # noqa: E731
    new = run_scenario(PoseEstimator, steps=1100, seed=9, carrying=carry)
    old = run_scenario(FrozenEstimator, steps=1100, seed=9, carrying=carry, legacy_api=True)
    after = new["t"] >= 750
    assert _p95(new["heading_err"][after]) < 5.0
    assert _p95(old["heading_err"][after]) > _p95(new["heading_err"][after])


# ---------------------------------------------------------------- dropouts
def test_odometer_dropout_while_carrying():
    carry = lambda t: t >= 50                  # noqa: E731
    odo = lambda t: not (300 <= t < 500)       # noqa: E731
    new = run_scenario(PoseEstimator, steps=900, seed=10, carrying=carry, odometer_on=odo)
    est = new["est"]
    assert est.carry_model_steps == 200
    out = (new["t"] >= 300) & (new["t"] < 500)
    # Model-based carrying with an inflated variance: bounded, compass-anchored.
    assert _p95(new["heading_err"][out]) < 12.0
    back = new["t"] >= 560
    assert _p95(new["heading_err"][back]) < 9.0


@pytest.mark.parametrize("seed", [11, 21])
def test_compass_dropout_and_recovery_while_carrying(seed):
    carry = lambda t: t >= 50                  # noqa: E731
    compass = lambda t: not (300 <= t < 600)   # noqa: E731
    twin = run_scenario(PoseEstimator, steps=1000, seed=seed, carrying=carry)
    new = run_scenario(PoseEstimator, steps=1000, seed=seed, carrying=carry, compass_on=compass)
    out = (new["t"] >= 300) & (new["t"] < 600)
    # Odometer-only heading is a random walk of about 1 degree per step.
    assert _p95(new["heading_err"][out]) < 40.0
    assert new["est"].heading_var < (5 * DEG) ** 2     # converged again at the end
    # Within 50 steps of the compass returning, the estimate rejoins the run
    # that never lost the compass (same noise realization).
    recovered = new["t"] >= 650
    assert np.max(np.abs(new["heading_err"][recovered] - twin["heading_err"][recovered])) < 1.0


def test_estimator_survives_all_sensors_missing_while_carrying():
    carry = lambda t: t >= 20                  # noqa: E731
    off = lambda t: not (200 <= t < 260)       # noqa: E731
    new = run_scenario(PoseEstimator, steps=500, seed=12, carrying=carry,
                       odometer_on=off, compass_on=off, gps_on=off)
    assert np.all(np.isfinite(new["heading_err"]))
    assert _p95(new["heading_err"][new["t"] >= 330]) < 9.0


# ------------------------------------------------------- wrapping and command
def test_heading_wraps_across_pi_while_carrying():
    targets = lambda t: math.pi - 0.05 if (t // 80) % 2 == 0 else -math.pi + 0.05   # noqa: E731
    new = run_scenario(PoseEstimator, steps=800, seed=13, carrying=lambda t: True,
                       start_heading=math.pi - 0.1, targets=targets)
    est = new["est"]
    assert -math.pi <= est.heading < math.pi
    assert np.max(new["heading_err"]) < 20.0
    assert _p95(new["heading_err"]) < 9.0


def test_stale_command_is_not_reused():
    """A step without a recorded command must not repeat the previous one."""
    def drive(estimator, record_second: bool) -> float:
        estimator.update((0.0, 0.0), 0.0, (0.0, 0.0, 0.0), grasping=False)
        estimator.set_command(0.0, 0.0, 1.0, carrying=False)
        estimator.update((0.0, 0.0), None, (0.0, 0.0, P.ROT_GAIN), grasping=False)
        if record_second:
            estimator.set_command(0.0, 0.0, 0.0, carrying=False)
        # The shell applied a zero command (the brain failed): no rotation.
        estimator.update((0.0, 0.0), None, (0.0, 0.0, 0.0), grasping=False)
        return estimator.heading

    assert drive(PoseEstimator(), True) == pytest.approx(P.ROT_GAIN)
    est = PoseEstimator()
    assert drive(est, False) == pytest.approx(P.ROT_GAIN)
    assert est.unknown_command_steps == 1
    # The Phase 1 estimator repeats the stale full-rate command but notices
    # the 10-degree discrepancy only because it exceeds its threshold; a
    # small stale command would pass unnoticed:
    frozen = FrozenEstimator()
    frozen.update((0.0, 0.0), 0.0, (0.0, 0.0, 0.0))
    frozen.set_command(0.0, 0.0, 0.3, carrying=False)
    frozen.update((0.0, 0.0), None, (0.0, 0.0, 0.3 * P.ROT_GAIN))
    frozen.update((0.0, 0.0), None, (0.0, 0.0, 0.0))
    assert frozen.heading == pytest.approx(0.6 * P.ROT_GAIN)
    current = PoseEstimator()
    current.update((0.0, 0.0), 0.0, (0.0, 0.0, 0.0), grasping=False)
    current.set_command(0.0, 0.0, 0.3, carrying=False)
    current.update((0.0, 0.0), None, (0.0, 0.0, 0.3 * P.ROT_GAIN), grasping=False)
    current.update((0.0, 0.0), None, (0.0, 0.0, 0.0), grasping=False)
    assert current.heading == pytest.approx(0.3 * P.ROT_GAIN)


def test_abrupt_rotation_disturbance_is_measured():
    kick = {200: 25 * DEG, 400: -40 * DEG}
    new = run_scenario(PoseEstimator, steps=600, seed=14, kick=kick)
    assert new["est"].contact_rotation_steps >= 2
    near = ((new["t"] >= 200) & (new["t"] < 230)) | ((new["t"] >= 400) & (new["t"] < 430))
    assert _p95(new["heading_err"][near]) < 6.0


def test_carry_gain_settles_near_design_value():
    new = run_scenario(PoseEstimator, steps=600, seed=15, carrying=lambda t: True)
    est = new["est"]
    p_minus = est.heading_var + (1 * DEG) ** 2
    gain = p_minus / (p_minus + P.COMPASS_NOISE ** 2)
    assert 0.15 < gain < 0.3
