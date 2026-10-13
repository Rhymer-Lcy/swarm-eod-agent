"""Evaluation-only ground-truth logging for simulation rounds.

``with_truth_log(cls, sink)`` returns a subclass of an agent class whose only
changes are that ``define_message_for_all`` first copies ground truth (drone
pose, health, grasp state, bombs) into ``sink`` and then returns the agent's
own message unchanged, and that ``control`` is timed (controller time per
drone-step) and, for red, the submitting brain's merged wall evidence is
copied once after submission for offline mapping ablations. The framework calls ``define_message_for_all`` once
per step for every drone, before ``control()`` and outside the control-step
ground-truth guard, so the recorded state is exactly the state the agent's
next ``control()`` observes through its sensors.

Information flows one way: from the simulation into the sink. The sink is
never reachable from agent code, and the wrapper does not touch agent state.
This module must never be imported by anything under ``solutions/``.
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np


class TruthSink:
    """Collects per-step ground truth for drones and bombs."""

    DRONE_FIELDS = ("step", "drone", "x", "y", "angle", "health", "grasped",
                    "reward", "removed", "inventory")
    BOMB_FIELDS = ("step", "bomb", "x", "y", "grasped")

    def __init__(self, bomb_every: int = 5) -> None:
        self.drone_rows: List[tuple] = []
        self.bomb_rows: List[tuple] = []
        self.bomb_every = max(1, int(bomb_every))
        self._bomb_ids: Dict[int, int] = {}
        # The framework resets ``Agent.reward`` before every physics step, so
        # the per-step value is accumulated here into a running total.
        self._reward_total: Dict[int, float] = {}
        self.agent_reports: Dict[int, Any] = {}
        self.submitted_step: Optional[int] = None
        self.brain_rows: List[tuple] = []
        self._state_codes: Dict[str, int] = {}
        self.control_seconds: Dict[int, float] = {}
        self.control_calls: Dict[int, int] = {}
        self.control_max: Dict[int, float] = {}
        self.submission: Optional[Dict[str, Any]] = None

    def add_control_time(self, ident: int, seconds: float) -> None:
        self.control_seconds[ident] = self.control_seconds.get(ident, 0.0) + seconds
        self.control_calls[ident] = self.control_calls.get(ident, 0) + 1
        self.control_max[ident] = max(self.control_max.get(ident, 0.0), seconds)

    def capture_submission(self, drone: Any) -> None:
        """Copy the submitting red brain's merged wall evidence (read-only).

        Used offline for mapping ablations (for example border rendering on
        or off) on the evidence of a real round; nothing flows back.
        """
        if self.submission is not None:
            return
        brain = getattr(drone, "_brain", None)
        if brain is None or not getattr(brain, "submitted", False):
            return
        evidence = getattr(brain, "evidence", None)
        snapshots = getattr(brain, "snapshots", None)
        if evidence is None or snapshots is None:
            return
        from swarm_rescue.solutions.eod.wallmap import merge
        self.submission = {
            "drone": int(drone.identifier),
            "evidence": merge(evidence, (v[1] for v in snapshots.values())),
            "dc_box": brain.dc.bbox() if getattr(brain, "dc", None) is not None else None,
            "submitted_map": getattr(brain, "final_map", None),
        }

    def record(self, drone: Any) -> None:
        step = int(drone.elapsed_timestep) + 1
        ident = int(drone.identifier)
        pos = drone.true_position()
        angle = float(drone.true_angle())
        inventory = int(drone.carried_bombs_count())
        total = self._reward_total.get(ident, 0.0) + float(drone.reward)
        self._reward_total[ident] = total
        self.drone_rows.append((
            step, ident, float(pos[0]), float(pos[1]), angle,
            int(drone.drone_health), len(drone.grasped_bombs()),
            total, bool(getattr(drone, "removed", False)),
            inventory,
        ))
        if (getattr(drone, "_exploration_map_submission", None) is not None
                and self.submitted_step is None):
            self.submitted_step = step
        brain = getattr(drone, "_brain", None)
        if brain is not None and getattr(brain, "est", None) is not None:
            # Read-only snapshot of the agent's internal state for offline
            # analysis (what it believed and intended at this step).
            est = brain.est
            target = getattr(brain, "target", None) or (math.nan, math.nan)
            label = str(getattr(brain, "state", ""))
            phase = str(getattr(brain, "phase", ""))
            code = self._state_codes.setdefault(f"{phase}/{label}", len(self._state_codes))
            self.brain_rows.append((step, ident, float(est.x), float(est.y), float(est.heading),
                                    float(target[0]), float(target[1]), code))
        report = getattr(drone, "diagnostics_report", None)
        if callable(report):
            try:
                self.agent_reports[ident] = report()
            except Exception as exc:  # diagnostics must never break a run
                self.agent_reports[ident] = {"error": repr(exc)}
        if ident == 0 and step % self.bomb_every == 0:
            self._record_bombs(drone, step)

    def _record_bombs(self, drone: Any, step: int) -> None:
        from swarm_rescue.simulation.elements.bomb import Bomb

        playground = getattr(drone, "_playground", None)
        if playground is None:
            return
        for element in playground.elements:
            if not isinstance(element, Bomb):
                continue
            key = id(element)
            if key not in self._bomb_ids:
                self._bomb_ids[key] = len(self._bomb_ids)
            pos = element.true_position()
            self.bomb_rows.append((step, self._bomb_ids[key], float(pos[0]),
                                   float(pos[1]), bool(element.grasped_by)))

    def save(self, directory: Path) -> Dict[str, Any]:
        directory.mkdir(parents=True, exist_ok=True)
        drones = np.array(self.drone_rows, dtype=np.float64).reshape(
            -1, len(self.DRONE_FIELDS))
        bombs = np.array(self.bomb_rows, dtype=np.float64).reshape(
            -1, len(self.BOMB_FIELDS))
        brain = np.array(self.brain_rows, dtype=np.float64).reshape(-1, 8)
        np.savez_compressed(directory / "truth.npz", drones=drones,
                            bombs=bombs, brain=brain,
                            drone_fields=np.array(self.DRONE_FIELDS),
                            bomb_fields=np.array(self.BOMB_FIELDS),
                            brain_fields=np.array(("step", "drone", "est_x", "est_y",
                                                   "est_heading", "target_x", "target_y",
                                                   "state_code")),
                            state_codes=np.array(json.dumps(self._state_codes)))
        with (directory / "agent_reports.json").open("w",
                                                      encoding="utf-8") as f:
            json.dump(self.agent_reports, f, indent=1, sort_keys=True,
                      default=_json_default)
        summary = summarize_truth(drones, bombs)
        calls = sum(self.control_calls.values())
        if calls:
            total = sum(self.control_seconds.values())
            summary["controller"] = {
                "calls": calls,
                "total_s": round(total, 3),
                "mean_ms_per_drone_step": round(1000.0 * total / calls, 4),
                "max_ms": round(1000.0 * max(self.control_max.values()), 2),
            }
        if self.submission is not None:
            sub = self.submission
            np.savez_compressed(
                directory / "submission_evidence.npz",
                evidence=sub["evidence"].astype(np.float32),
                dc_box=np.array(sub["dc_box"] if sub["dc_box"] is not None else [],
                                dtype=np.float64),
                submitted_map=(sub["submitted_map"] if sub["submitted_map"] is not None
                               else np.zeros((0, 0), np.uint8)))
            summary["submission_evidence_drone"] = sub["drone"]
        return summary


def _json_default(value: Any) -> Any:
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    return repr(value)


def summarize_truth(drones: np.ndarray, bombs: np.ndarray) -> Dict[str, Any]:
    """Per-round diagnostic metrics derived from the truth log.

    These are diagnostics, not scores: collisions are inferred from health
    decrements and transport failures from grasp losses without a reward.
    """
    summary: Dict[str, Any] = {}
    if drones.size == 0:
        return summary
    idx = {name: i for i, name in enumerate(TruthSink.DRONE_FIELDS)}
    per_drone = {}
    for ident in np.unique(drones[:, idx["drone"]]).astype(int):
        rows = drones[drones[:, idx["drone"]] == ident]
        rows = rows[np.argsort(rows[:, idx["step"]])]
        health = rows[:, idx["health"]]
        grasped = rows[:, idx["grasped"]] > 0
        reward = rows[:, idx["reward"]]
        xy = rows[:, [idx["x"], idx["y"]]]
        travel = float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0
        grasp_events = int(np.sum(grasped[1:] & ~grasped[:-1]))
        release_events = np.flatnonzero(~grasped[1:] & grasped[:-1]) + 1
        disposals = 0
        failures = 0
        for i in release_events:
            if reward[i] > reward[i - 1] + 1e-9:
                disposals += 1
            else:
                failures += 1
        per_drone[int(ident)] = {
            "health_final": int(health[-1]),
            "health_lost": int(health[0] - health[-1]),
            "travel_px": round(travel, 1),
            "grasp_events": grasp_events,
            "releases_with_reward": disposals,
            "releases_without_reward": failures,
            "reward_final": float(reward[-1]),
            "steps_carrying": int(np.sum(grasped)),
            "removed": bool(rows[-1, idx["removed"]]),
        }
    summary["per_drone"] = per_drone
    summary["team"] = {
        "health_lost_total": int(sum(d["health_lost"] for d in per_drone.values())),
        "drones_removed": int(sum(d["removed"] for d in per_drone.values())),
        "grasp_events": int(sum(d["grasp_events"] for d in per_drone.values())),
        "transport_failures": int(sum(d["releases_without_reward"]
                                      for d in per_drone.values())),
        "drones_with_reward": int(sum(d["reward_final"] > 0
                                      for d in per_drone.values())),
        "travel_px_total": round(sum(d["travel_px"] for d in per_drone.values()), 1),
    }
    if bombs.size:
        bidx = {name: i for i, name in enumerate(TruthSink.BOMB_FIELDS)}
        summary["bombs_seen_in_log"] = int(len(np.unique(bombs[:, bidx["bomb"]])))
    return summary


def with_truth_log(base_cls: type, sink: TruthSink) -> type:
    """Subclass ``base_cls`` so that every drone logs truth into ``sink``."""

    class TruthLogged(base_cls):  # type: ignore[misc, valid-type]
        def define_message_for_all(self):
            message = super().define_message_for_all()
            sink.record(self)
            return message

        def control(self):
            # Controller time only (the agent's own control step); the
            # simulator's time is the round's wall time minus the sum.
            started = time.perf_counter()
            command = super().control()
            sink.add_control_time(int(self.identifier), time.perf_counter() - started)
            sink.capture_submission(self)
            return command

    TruthLogged.__name__ = f"TruthLogged{base_cls.__name__}"
    TruthLogged.__qualname__ = TruthLogged.__name__
    return TruthLogged


def angle_diff(a: float, b: float) -> float:
    """Signed smallest difference a - b in radians."""
    return math.atan2(math.sin(a - b), math.cos(a - b))
