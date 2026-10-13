"""Tests for the evaluation-only truth logger (devtools/simlab/diagnostics.py)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.simlab.diagnostics import TruthSink  # noqa: E402


class _Est:
    x, y, heading = 1.0, 2.0, 0.5


class _Brain:
    est = _Est()
    target = (10.0, 20.0)
    state = "carry"


class _FakeDrone:
    def __init__(self, ident):
        self.identifier = ident
        self.elapsed_timestep = 0
        self.drone_health = 50
        self.reward = 0.0
        self.removed = False
        self._grasped = []
        self._brain = _Brain()

    def true_position(self):
        return np.array([float(self.elapsed_timestep), 0.0])

    def true_angle(self):
        return 0.0

    def carried_bombs_count(self):
        return 0

    def grasped_bombs(self):
        return self._grasped


def test_sink_accumulates_reward_and_logs_brain_state(tmp_path):
    sink = TruthSink(bomb_every=10_000)
    drone = _FakeDrone(3)
    for step in range(5):
        drone.elapsed_timestep = step
        drone._grasped = ["bomb"] if step in (1, 2) else []
        drone.reward = 1.0 if step == 3 else 0.0     # per-step value, reset by the framework
        sink.record(drone)
    summary = sink.save(tmp_path)
    data = np.load(tmp_path / "truth.npz")
    assert data["drones"].shape == (5, len(TruthSink.DRONE_FIELDS))
    assert data["drones"][-1, 7] == 1.0               # cumulative reward
    assert data["brain"].shape == (5, 8)
    codes = json.loads(str(data["state_codes"]))
    assert codes == {"/carry": 0}
    per = summary["per_drone"][3]
    assert per["grasp_events"] == 1 and per["releases_with_reward"] == 1
    assert summary["team"]["transport_failures"] == 0
