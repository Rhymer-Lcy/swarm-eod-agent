"""Scripted characterization probes (development only).

``DynamicsProbe`` (place mode) runs a fixed schedule per drone identifier and
logs, every step, the true state next to every measured sensor value and the
command issued. It answers questions the controllers depend on: thrust and
rotation response, the lateral sign, command normalization, odometer
semantics, GPS and compass noise, lidar and semantic angle conventions,
collision damage, message latency and range, and bomb placement rules.

``GraspProbe`` (rescue mode, with a bombs file) measures grasping, carrying
and disposal mechanics.

Probes read ground truth inside ``control()`` and must therefore run without
``--competition``. They are diagnostics, not agents: never import them from
``solutions/``.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.ray_sensors.drone_semantic_sensor import DroneSemanticSensor

from .fixtures import DEFAULT_OUT, load_fixture

_LOG: Dict[str, List[Any]] = {"steps": [], "lidar": [], "semantic": [],
                              "messages": [], "events": []}
_FIXTURES: Dict[str, dict] = {}
SUBMIT_STEP = 420


def save_probe_log(run_dir: Path) -> None:
    """Write the probe log next to the run record (called by the runner)."""
    steps = np.array(_LOG["steps"], dtype=np.float64)
    lidar = np.array(_LOG["lidar"], dtype=np.float64)
    np.savez_compressed(run_dir / "probe.npz", steps=steps, lidar=lidar,
                        step_fields=np.array(STEP_FIELDS))
    with (run_dir / "probe_extra.json").open("w", encoding="utf-8") as f:
        json.dump({"semantic": _LOG["semantic"], "messages": _LOG["messages"],
                   "events": _LOG["events"]}, f)


STEP_FIELDS = (
    "step", "drone", "tx", "ty", "ta", "tvx", "tvy", "tw",
    "gx", "gy", "compass", "odo_d", "odo_a", "odo_t",
    "c_fwd", "c_lat", "c_rot", "c_grasp", "c_place",
    "health", "inventory", "grasped", "lidar_min",
)


def _fixture(map_size) -> Optional[dict]:
    key = f"{int(map_size[0])}x{int(map_size[1])}"
    if key not in _FIXTURES:
        sizes = {"1250x800": "Map01", "1200x900": "Map02", "1113x750": "Map03",
                 "1660x1122": "Map04", "1200x1200": "Map05"}
        name = sizes.get(key)
        try:
            _FIXTURES[key] = load_fixture(name, DEFAULT_OUT) if name else None
        except OSError:
            _FIXTURES[key] = None
    return _FIXTURES[key]


def _free_distance(walls: np.ndarray, x: float, y: float, direction: float,
                   limit: float = 400.0, circles=(), boxes=()) -> float:
    """Distance from (x, y) along ``direction`` to the first obstacle.

    Obstacles are wall pixels, circles ``(cx, cy, r)`` and axis-aligned boxes
    ``(xmin, ymin, xmax, ymax)`` in world coordinates.
    """
    h, w = walls.shape
    step = 2.0
    d = 0.0
    cos_d, sin_d = math.cos(direction), math.sin(direction)
    while d < limit:
        px = x + d * cos_d
        py = y + d * sin_d
        col = int(round(px + w / 2))
        row = int(round(h / 2 - py))
        if not (0 <= row < h and 0 <= col < w) or walls[row, col]:
            return d
        for cx, cy, r in circles:
            if (px - cx) ** 2 + (py - cy) ** 2 < r * r:
                return d
        for x0, y0, x1, y1 in boxes:
            if x0 <= px <= x1 and y0 <= py <= y1:
                return d
        d += step
    return limit


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class _ProbeBase(DroneAbstract):
    """Shared logging and truth-based helpers for probes."""

    def __init__(self, identifier: Optional[int] = None, misc_data=None, **kwargs):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self._k = 0
        self._phase = "start"
        self._phase_k = 0
        self._target_heading: Optional[float] = None
        self._prev_place = 0

    def define_message_for_all(self):
        pos = self.true_position()
        return ("probe", int(self.identifier), int(self.elapsed_timestep),
                float(pos[0]), float(pos[1]))

    # -- helpers -------------------------------------------------------
    def _open_direction(self, prefer_far: bool = True) -> float:
        """Direction with the most (or least) free space, from truth.

        Other drones (positions from their truth messages) and the disposal
        center count as obstacles, so ``prefer_far=False`` finds the nearest
        real wall rather than a neighbour or the center.
        """
        fixture = _fixture(self.size_area)
        pos = self.true_position()
        circles = [(m[3], m[4], 24.0) for _, m in self.communicator.received_messages
                   if isinstance(m, tuple) and m and m[0] == "probe"]
        boxes = []
        if fixture is not None:
            dc = fixture["disposal_center"]
            boxes.append((dc["x"] - dc["width"] / 2 - 15, dc["y"] - dc["height"] / 2 - 15,
                          dc["x"] + dc["width"] / 2 + 15, dc["y"] + dc["height"] / 2 + 15))
        best, best_d = 0.0, (-1.0 if prefer_far else 1e9)
        for k in range(72):
            direction = -math.pi + k * (2 * math.pi / 72)
            if fixture is None:
                d_all = d_wall = 100.0
            else:
                d_all = _free_distance(fixture["walls"], pos[0], pos[1], direction,
                                       circles=circles, boxes=boxes)
                d_wall = _free_distance(fixture["walls"], pos[0], pos[1], direction)
            if prefer_far:
                if d_all > best_d:
                    best, best_d = direction, d_all
            elif d_wall <= d_all + 1e-9 and d_wall < best_d:
                # The wall is the first thing hit along this direction.
                best, best_d = direction, d_wall
        return best

    def _turn_to(self, cmd: dict, heading: float, tol: float = 0.04) -> bool:
        err = _wrap(heading - self.true_angle())
        if abs(err) < tol:
            cmd["rotation"] = 0.0
            return True
        cmd["rotation"] = float(np.clip(err / 0.18, -1.0, 1.0))
        return False

    def _log(self, cmd: dict) -> None:
        pos = self.true_position()
        vel = self.true_velocity()
        gps = self.measured_gps_position()
        compass = self.measured_compass_angle()
        odo = self.odometer_values()
        lidar = self.lidar_values()
        nan = float("nan")
        self._LOG_STEP = int(self.elapsed_timestep)
        _LOG["steps"].append((
            self.elapsed_timestep, self.identifier, pos[0], pos[1],
            self.true_angle(), vel[0], vel[1], self.true_angular_velocity(),
            gps[0] if gps is not None else nan,
            gps[1] if gps is not None else nan,
            compass if compass is not None else nan,
            *(odo if odo is not None else (nan, nan, nan)),
            cmd.get("forward", 0.0), cmd.get("lateral", 0.0),
            cmd.get("rotation", 0.0), cmd.get("grasper", 0),
            cmd.get("place_bomb", 0),
            self.drone_health, self.carried_bombs_count(),
            len(self.grasped_bombs()),
            float(np.min(lidar)) if lidar is not None else nan,
        ))
        for sender, msg in self.communicator.received_messages:
            if isinstance(msg, tuple) and msg and msg[0] == "probe":
                _LOG["messages"].append((
                    int(self.elapsed_timestep), int(self.identifier), msg[1],
                    msg[2], math.hypot(msg[3] - pos[0], msg[4] - pos[1])))

    def _log_sensors(self) -> None:
        pos = self.true_position()
        lidar = self.lidar_values()
        if lidar is not None:
            _LOG["lidar"].append((self.elapsed_timestep, self.identifier,
                                  pos[0], pos[1], self.true_angle(), *lidar))
        sem = self.semantic_values()
        if sem is not None:
            _LOG["semantic"].append({
                "step": int(self.elapsed_timestep), "drone": int(self.identifier),
                "tx": float(pos[0]), "ty": float(pos[1]),
                "ta": float(self.true_angle()),
                "hits": [(float(d.distance), float(d.angle), d.entity_type.name,
                          bool(d.grasped)) for d in sem]})

    def _event(self, name: str, **data: Any) -> None:
        _LOG["events"].append({"step": int(self.elapsed_timestep),
                               "drone": int(self.identifier), "event": name,
                               **data})


class DynamicsProbe(_ProbeBase):
    """Place-mode probe: one scripted experiment per drone identifier."""

    def control(self):
        self._k += 1
        k = self._k
        cmd = {"forward": 0.0, "lateral": 0.0, "rotation": 0.0, "grasper": 0,
               "place_bomb": 0}
        role = int(self.identifier)
        if k > 30:
            getattr(self, f"_role_{role}", self._role_idle)(cmd, k - 30)
        if k % 5 == 0 or role == 4 or k in (1, 2, 3):
            self._log_sensors()
        self._log(cmd)
        if role == 0 and k == SUBMIT_STEP:
            w, h = self.size_area
            self.submit_exploration_map(np.zeros((h, w), dtype=np.uint8))
            self._event("submitted")
        return cmd

    def _role_idle(self, cmd: dict, k: int) -> None:
        pass

    def _align_then(self, cmd: dict, k: int, prefer_far: bool = True,
                    offset: float = 0.0) -> bool:
        if self._target_heading is None:
            self._target_heading = _wrap(self._open_direction(prefer_far) + offset)
            self._event("target_heading", heading=self._target_heading)
        if self._phase == "start":
            if self._turn_to(cmd, self._target_heading):
                self._phase = "go"
                self._phase_k = k
                self._event("aligned")
            return False
        return True

    def _role_0(self, cmd: dict, k: int) -> None:
        """Full thrust 25 steps, coast 25, full reverse until nearly stopped."""
        if not self._align_then(cmd, k):
            return
        t = k - self._phase_k
        if t < 25:
            cmd["forward"] = 1.0
        elif t < 50:
            cmd["forward"] = 0.0
        elif t < 90:
            speed = float(np.hypot(*self.true_velocity()))
            cmd["forward"] = -1.0 if speed > 0.3 else 0.0

    def _role_1(self, cmd: dict, k: int) -> None:
        """Half thrust 50 steps, then half reverse 25 steps."""
        if not self._align_then(cmd, k):
            return
        t = k - self._phase_k
        if t < 50:
            cmd["forward"] = 0.5
        elif t < 75:
            cmd["forward"] = -0.5

    def _role_2(self, cmd: dict, k: int) -> None:
        """Rotation ladder: +1, -0.5, +0.25 for 36 steps each."""
        if k <= 36:
            cmd["rotation"] = 1.0
        elif k <= 72:
            cmd["rotation"] = -0.5
        elif k <= 108:
            cmd["rotation"] = 0.25

    def _role_3(self, cmd: dict, k: int) -> None:
        """Lateral +1 for 25 steps with the open direction on the left."""
        if not self._align_then(cmd, k, offset=-math.pi / 2):
            return
        t = k - self._phase_k
        if t < 25:
            cmd["lateral"] = 1.0
        elif t < 50:
            cmd["lateral"] = -1.0

    def _role_4(self, cmd: dict, k: int) -> None:
        """Stationary: noise statistics and sensor conventions."""

    def _role_5(self, cmd: dict, k: int) -> None:
        """forward=1 and lateral=1 together (normalization), then a curve."""
        if not self._align_then(cmd, k, offset=math.pi / 4):
            return
        t = k - self._phase_k
        if t < 20:
            cmd["forward"] = 1.0
            cmd["lateral"] = 1.0
        elif t < 50:
            cmd["forward"] = 0.6
            cmd["rotation"] = 0.5

    def _role_6(self, cmd: dict, k: int) -> None:
        """Drive into the nearest wall and keep pushing (collision damage)."""
        if not self._align_then(cmd, k, prefer_far=False):
            return
        t = k - self._phase_k
        if t < 300:
            cmd["forward"] = 0.6

    def _role_7(self, cmd: dict, k: int) -> None:
        """Speed ladder: 0.25 for 70 steps then 0.75 for 50 steps."""
        if not self._align_then(cmd, k):
            return
        t = k - self._phase_k
        if t < 70:
            cmd["forward"] = 0.25
        elif t < 120:
            cmd["forward"] = 0.75

    def _role_8(self, cmd: dict, k: int) -> None:
        """Placement rules: rising edge, separation, inventory."""
        schedule = {10: 1, 11: 1, 12: 1, 13: 0, 14: 1, 15: 0}
        if k in schedule:
            cmd["place_bomb"] = schedule[k]
            if k in (10, 14):
                self._event("place_attempt", inventory=self.carried_bombs_count())
            return
        if 15 < k <= 40:
            if not self._align_then(cmd, k):
                return
            if k - self._phase_k < 8:
                cmd["forward"] = 0.8
        if k in (60, 61):
            cmd["place_bomb"] = 1 if k == 60 else 0
            self._event("place_attempt_far", inventory=self.carried_bombs_count())

    def _role_9(self, cmd: dict, k: int) -> None:
        """Place a bomb, try to place while grasping, grasp it, carry, release."""
        if k == 10:
            cmd["place_bomb"] = 1
            self._event("place_attempt", inventory=self.carried_bombs_count())
            return
        if 20 <= k < 140:
            cmd["grasper"] = 1
        if k == 25:
            cmd["place_bomb"] = 1
            self._event("place_while_grasping",
                        inventory=self.carried_bombs_count(),
                        grasped=len(self.grasped_bombs()))
        if 40 <= k < 140:
            if not self._align_then(cmd, k):
                return
            t = k - self._phase_k
            if t < 60:
                cmd["forward"] = 1.0
        if k == 140:
            self._event("release", grasped=len(self.grasped_bombs()))


class GraspProbe(_ProbeBase):
    """Rescue-mode probe: grasp, carry and dispose, scripted per drone.

    Use with ``layout_grasp_probe(fixture)``: bomb ``i`` (i = 0..3) lies
    straight below drone ``i`` of the start grid. Drones 4..9 first move to
    the right, out of the delivery corridors, and then hold. From step 80 the
    test drones run, staggered by 40 steps:

    * drone 0: frontal approach, then forward into the disposal center
      (bomb leading);
    * drone 1: approach with the left side, then strafe into the center with
      the bomb leading on the left;
    * drone 2: frontal approach, then reverse into the center (bomb
      trailing);
    * drone 3: frontal approach, carry away from the center for 60 steps at
      full thrust, release, watch the bomb for 150 steps, re-grasp and
      deliver frontally.

    A drone still carrying 300 steps after starting delivery spins in place
    while pushing for 40 steps (logged as ``spin``).
    """

    START = 80
    STAGGER = 40

    def __init__(self, identifier: Optional[int] = None, misc_data=None, **kwargs):
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self._bomb = None
        self._dc = None
        self._home = None
        self._deliver_k = None
        self._was_grasping = False

    def control(self):
        self._k += 1
        k = self._k
        cmd = {"forward": 0.0, "lateral": 0.0, "rotation": 0.0, "grasper": 0}
        ident = int(self.identifier)
        if k == 1:
            fixture = _fixture(self.size_area)
            self._dc = fixture["disposal_center"]
            self._home = np.array(self.true_position())
            if ident <= 3:
                self._bomb = self._find_bomb_below()
        if ident >= 4:
            target = np.array([-250.0 + 30.0 * (ident - 4), self._home[1]])
            if k > 5:
                self._goto(cmd, target, 0.0, speed=0.5)
        elif self._bomb is not None and k >= self.START + self.STAGGER * ident:
            getattr(self, f"_plan_{ident}")(cmd, k)
        self._track()
        if k % 3 == 0:
            self._log_sensors()
        self._log(cmd)
        return cmd

    # -- truth helpers ---------------------------------------------------
    def _find_bomb_below(self):
        from swarm_rescue.simulation.elements.bomb import Bomb
        pos = self.true_position()
        best, best_d = None, 1e9
        for element in self._playground.elements:
            if isinstance(element, Bomb):
                b = element.true_position()
                d = abs(b[0] - pos[0]) + (0.0 if b[1] < pos[1] else 1e6)
                if d < best_d:
                    best, best_d = element, d
        return best

    def _bomb_xy(self) -> np.ndarray:
        return np.array(self._bomb.true_position())

    def _track(self) -> None:
        grasping = bool(self.grasped_bombs())
        if self._bomb is not None and not self._bomb.removed:
            b = self._bomb_xy()
            pos = self.true_position()
            ang = self.true_angle()
            rel = b - pos
            body = (rel[0] * math.cos(ang) + rel[1] * math.sin(ang),
                    -rel[0] * math.sin(ang) + rel[1] * math.cos(ang))
            _LOG["events"].append({"step": int(self.elapsed_timestep),
                                   "drone": int(self.identifier),
                                   "event": "bomb_track",
                                   "bx": float(b[0]), "by": float(b[1]),
                                   "body_x": float(body[0]), "body_y": float(body[1]),
                                   "grasped": grasping})
        if grasping and not self._was_grasping:
            self._event("grasped")
        if self._was_grasping and not grasping:
            self._event("grasp_lost", reward=float(self.reward))
        self._was_grasping = grasping

    def _goto(self, cmd: dict, target: np.ndarray, heading: float,
              speed: float = 0.6, stop: float = 0.0) -> float:
        """Truth-based holonomic move: hold ``heading``, translate to target.

        Uses the measured lateral convention: positive lateral thrust moves
        the drone to its left (+90 degrees).
        """
        pos = self.true_position()
        delta = target - pos
        dist = float(np.hypot(*delta))
        self._turn_to(cmd, heading, tol=0.02)
        if dist > stop:
            mag = min(speed, 0.03 * (dist - stop) + 0.1)
            world = delta / max(dist, 1e-6) * mag
            ang = self.true_angle()
            cmd["forward"] = float(np.clip(world[0] * math.cos(ang)
                                           + world[1] * math.sin(ang), -1, 1))
            cmd["lateral"] = float(np.clip(-world[0] * math.sin(ang)
                                           + world[1] * math.cos(ang), -1, 1))
        return dist

    def _dc_entry(self) -> np.ndarray:
        """Point inside the center straight above this drone's start column."""
        return np.array([self._home[0], self._dc["y"]])

    def _approach(self, cmd: dict, side: float) -> bool:
        if self.grasped_bombs():
            return True
        b = self._bomb_xy()
        pos = self.true_position()
        bearing = math.atan2(b[1] - pos[1], b[0] - pos[0])
        cmd["grasper"] = 1
        self._goto(cmd, b, _wrap(bearing - side), speed=0.35)
        return False

    def _deliver(self, cmd: dict, side: float, k: int) -> None:
        cmd["grasper"] = 1
        if self._deliver_k is None:
            self._deliver_k = k
            self._event("deliver_start", side=side)
        entry = self._dc_entry()
        pos = self.true_position()
        bearing = math.atan2(entry[1] - pos[1], entry[0] - pos[0])
        t = k - self._deliver_k
        if 300 <= t < 340:
            if t == 300:
                self._event("spin")
            cmd["rotation"] = 1.0
            cmd["forward"] = 0.4
            return
        self._goto(cmd, entry, _wrap(bearing - side), speed=0.6)

    # -- plans -------------------------------------------------------------
    def _plan_0(self, cmd: dict, k: int) -> None:
        if self._deliver_k is not None or self._approach(cmd, 0.0):
            self._deliver(cmd, 0.0, k)

    def _plan_1(self, cmd: dict, k: int) -> None:
        if self._deliver_k is not None or self._approach(cmd, math.pi / 2):
            self._deliver(cmd, math.pi / 2, k)

    def _plan_2(self, cmd: dict, k: int) -> None:
        if self._deliver_k is not None or self._approach(cmd, 0.0):
            self._deliver(cmd, math.pi, k)

    def _plan_3(self, cmd: dict, k: int) -> None:
        if self._phase == "start":
            if self._approach(cmd, 0.0):
                self._phase = "carry_away"
                self._phase_k = k
            return
        if self._phase == "carry_away":
            cmd["grasper"] = 1
            if k - self._phase_k < 60:
                ang = self.true_angle()
                # Full thrust straight down (away from the center), any heading.
                cmd["forward"] = float(math.cos(-math.pi / 2 - ang))
                cmd["lateral"] = float(math.sin(-math.pi / 2 - ang))
            else:
                self._phase = "released"
                self._phase_k = k
                self._event("release")
            return
        if self._phase == "released":
            if k - self._phase_k > 150:
                self._phase = "regrasp"
            return
        if self._deliver_k is not None or self._approach(cmd, 0.0):
            self._deliver(cmd, 0.0, k)


def layout_grasp_probe(fixture: dict) -> dict:
    """Bombs straight below the first four start positions (78 px down)."""
    starts = fixture["starts"]
    bombs = [{"x": float(starts[i][0]), "y": float(starts[i][1] - 78.0), "theta": 0.0}
             for i in range(4)]
    return {"map_name": fixture["map"], "zones_config": [], "bombs": bombs,
            "generator": "devtools.simlab.probe.layout_grasp_probe "
                         "(synthetic probe layout)"}
