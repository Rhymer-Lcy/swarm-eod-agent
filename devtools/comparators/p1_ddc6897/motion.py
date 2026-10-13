"""Velocity control, local obstacle avoidance and path following.

The drone is holonomic: it can translate in any direction regardless of its
heading, so motion and heading are controlled separately. Desired world-frame
velocities are turned into thrust with a one-step (deadbeat) inversion of the
measured motion model, limited to the unit thrust disc. Speed is capped by
the clearance ahead using the measured braking distance, and a short-range
repulsion from lidar returns and teammates keeps the drone off walls and
other drones.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from . import params as P
from .geometry import clip_unit_vector, world_to_body, wrap


def _stopping_table() -> Tuple[np.ndarray, np.ndarray]:
    speeds = np.arange(0.0, 12.25, 0.25)
    dists = []
    for v0 in speeds:
        v, d = v0, 0.0
        for _ in range(60):
            if v <= 0.0:
                break
            d += P.POS_VEL * v - P.POS_GAIN
            v = P.VEL_DECAY * v - P.VEL_GAIN
        dists.append(max(d, 0.0))
    return speeds, np.array(dists)


_SPEEDS, _STOP = _stopping_table()


def stopping_distance(speed: float) -> float:
    return float(np.interp(speed, _SPEEDS, _STOP))


def max_speed_for_clearance(clearance: float, margin: float = 6.0) -> float:
    """Largest speed that can still stop within ``clearance - margin``."""
    room = clearance - margin
    if room <= 0.0:
        return 0.0
    return float(np.interp(room, _STOP, _SPEEDS))


def thrust_for_velocity(vx: float, vy: float, heading: float, target_vx: float,
                        target_vy: float, carrying: bool) -> Tuple[float, float]:
    """Body-frame (forward, lateral) thrust that best reaches a target velocity."""
    gain = P.VEL_GAIN * (P.CARRY_GAIN_FACTOR if carrying else 1.0)
    ux = (target_vx - P.VEL_DECAY * vx) / gain
    uy = (target_vy - P.VEL_DECAY * vy) / gain
    norm = math.hypot(ux, uy)
    if norm > 1.0:
        ux, uy = ux / norm, uy / norm
    f, l = world_to_body(ux, uy, heading)
    return clip_unit_vector(f, l)


def rotation_toward(heading: float, desired: float, gain: float = 0.8) -> float:
    err = wrap(desired - heading)
    return max(-1.0, min(1.0, gain * err / P.ROT_GAIN))


class LocalView:
    """Lidar returns of one step as body-relative points in the world frame."""

    def __init__(self, heading: float, lidar: Optional[np.ndarray],
                 angles: np.ndarray, valid: Optional[np.ndarray] = None,
                 max_range: float = 120.0) -> None:
        self.ok = lidar is not None
        if not self.ok:
            self.px = self.py = self.dist = np.zeros(0)
            return
        keep = lidar < max_range
        if valid is not None:
            keep &= valid
        a = angles[keep] + heading
        d = lidar[keep]
        self.dist = d
        self.px = d * np.cos(a)
        self.py = d * np.sin(a)

    def clearance(self, direction: float, half_width: float) -> float:
        """Free distance along ``direction`` for a corridor of ``half_width``."""
        if not self.ok or self.dist.size == 0:
            return 120.0
        c, s = math.cos(direction), math.sin(direction)
        along = self.px * c + self.py * s
        across = np.abs(-self.px * s + self.py * c)
        inside = (along > 0) & (across < half_width)
        if not np.any(inside):
            return 120.0
        reach = along[inside] - np.sqrt(np.maximum(half_width ** 2 - across[inside] ** 2, 0.0))
        return float(max(np.min(reach), 0.0))

    def repulsion(self, radius: float) -> Tuple[float, float]:
        if not self.ok or self.dist.size == 0:
            return 0.0, 0.0
        near = self.dist < radius
        if not np.any(near):
            return 0.0, 0.0
        d = np.maximum(self.dist[near], 1.0)
        w = (radius - d) / radius
        return (float(-np.sum(w * self.px[near] / d)),
                float(-np.sum(w * self.py[near] / d)))

    def min_distance(self) -> float:
        return float(np.min(self.dist)) if self.ok and self.dist.size else 1e9


def teammate_repulsion(x: float, y: float, others: Iterable[Tuple[float, float]],
                       radius: float = P.TEAMMATE_REPEL_RADIUS) -> Tuple[float, float]:
    rx = ry = 0.0
    for ox, oy in others:
        dx, dy = x - ox, y - oy
        d = math.hypot(dx, dy)
        if 1e-3 < d < radius:
            w = (radius - d) / radius
            rx += w * dx / d
            ry += w * dy / d
    return rx, ry


def lookahead_point(path: Sequence[Tuple[float, float]], x: float, y: float,
                    lookahead: float, visible) -> Tuple[float, float]:
    """Farthest path point within ``lookahead`` that ``visible`` accepts."""
    if not path:
        return x, y
    nearest = min(range(len(path)), key=lambda i: (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2)
    best = path[nearest]
    for i in range(nearest, len(path)):
        px, py = path[i]
        if math.hypot(px - x, py - y) > lookahead:
            break
        if visible(px, py):
            best = (px, py)
    return best


class StuckMonitor:
    def __init__(self, window: int = P.STUCK_WINDOW,
                 distance: float = P.STUCK_DISTANCE) -> None:
        self.window = window
        self.distance = distance
        self._hist: deque = deque(maxlen=window)

    def update(self, x: float, y: float, wanted_speed: float) -> bool:
        self._hist.append((x, y, wanted_speed))
        if len(self._hist) < self.window:
            return False
        x0, y0, _ = self._hist[0]
        wanted = sum(h[2] for h in self._hist) / len(self._hist)
        return wanted > 1.5 and math.hypot(x - x0, y - y0) < self.distance

    def reset(self) -> None:
        self._hist.clear()


def drive(est, target: Tuple[float, float], view: LocalView, *, cruise: float,
          carrying: bool, others: Sequence[Tuple[float, float]] = (),
          arrive_radius: float = 6.0, stop_at_target: bool = True,
          repel_walls: bool = True) -> Tuple[float, float, float]:
    """Thrust (forward, lateral) towards ``target`` and the speed requested."""
    dx, dy = target[0] - est.x, target[1] - est.y
    dist = math.hypot(dx, dy)
    if dist < arrive_radius and stop_at_target:
        f, l = thrust_for_velocity(est.vx, est.vy, est.heading, 0.0, 0.0, carrying)
        return f, l, 0.0
    direction = math.atan2(dy, dx)
    half_width = P.DRONE_RADIUS + (P.CARRY_OFFSET + 4.0 if carrying else 3.0)
    clearance = view.clearance(direction, half_width)
    speed = min(cruise, max_speed_for_clearance(clearance))
    if stop_at_target:
        speed = min(speed, max_speed_for_clearance(dist + 6.0, margin=6.0) + 0.3)
    if carrying:
        speed = min(speed, P.CARRY_SPEED + 0.5)
    vx, vy = speed * math.cos(direction), speed * math.sin(direction)
    if repel_walls:
        rx, ry = view.repulsion(P.WALL_REPEL_RADIUS + (8.0 if carrying else 0.0))
        vx += 3.0 * rx
        vy += 3.0 * ry
    tx, ty = teammate_repulsion(est.x, est.y, others)
    vx += 4.0 * tx
    vy += 4.0 * ty
    f, l = thrust_for_velocity(est.vx, est.vy, est.heading, vx, vy, carrying)
    return f, l, speed
