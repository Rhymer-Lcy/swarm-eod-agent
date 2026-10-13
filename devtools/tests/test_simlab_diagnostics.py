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


def test_wrapper_times_control_and_returns_the_agent_command(tmp_path):
    from devtools.simlab.diagnostics import with_truth_log

    class Agent:
        identifier = 2

        def control(self):
            return {"forward": 0.5}

    sink = TruthSink()
    wrapped = with_truth_log(Agent, sink)()
    assert wrapped.control() == {"forward": 0.5}
    assert wrapped.control() == {"forward": 0.5}
    assert sink.control_calls == {2: 2}
    summary = sink.save(tmp_path)
    assert summary["controller"]["calls"] == 2


def test_submission_evidence_is_captured_once(tmp_path):
    from swarm_rescue.solutions.eod.wallmap import WallEvidence

    class Dc:
        def bbox(self):
            return (-10.0, -10.0, 10.0, 10.0)

    class RedBrain:
        submitted = False
        evidence = WallEvidence((40, 30))
        snapshots = {}
        dc = Dc()
        final_map = None

    drone = _FakeDrone(1)
    drone._brain = RedBrain()
    sink = TruthSink()
    sink.capture_submission(drone)
    assert sink.submission is None                     # not submitted yet
    drone._brain.evidence.counts[5, 7] = 4
    drone._brain.submitted = True
    drone._brain.final_map = np.ones((30, 40), np.uint8)
    sink.capture_submission(drone)
    drone._brain.evidence.counts[5, 7] = 9             # later changes are ignored
    sink.capture_submission(drone)
    assert sink.submission["evidence"][5, 7] == 4
    sink.record(drone)
    summary = sink.save(tmp_path)
    data = np.load(tmp_path / "submission_evidence.npz")
    assert data["evidence"].shape == (30, 40) and data["evidence"][5, 7] == 4
    assert data["submitted_map"].sum() == 1200
    assert summary["submission_evidence_drone"] == 1
