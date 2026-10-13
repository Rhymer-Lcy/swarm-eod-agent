"""Pose estimation from GPS, compass, odometer and the known motion model.

The simulator's motion is deterministic given the commands (see ``params``),
so the estimator dead-reckons with the exact model and uses the sensors for
two things: the odometer detects when the model was violated (contact with a
wall, another drone or a pushed bomb; carrying) and then supplies the
measured displacement and rotation, and GPS and compass, whose errors are
slowly varying AR(1) processes, pull the dead-reckoned pose towards the
absolute frame with small gains. Early on the gains follow a running average
(the optimal estimate of a constant offset), later they settle at a floor so
that slow drift is still corrected.

Every sensor may be missing in a step (disabled or non-finite); the estimate
then continues from the model alone and its uncertainty grows.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from . import params as P
from .geometry import clip_unit_vector, wrap


class PoseEstimator:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.heading = 0.0
        self.initialized = False
        self.position_sigma = 1e3
        self.heading_sigma = math.pi
        self._n_gps = 0
        self._n_compass = 0
        self._last_cmd = (0.0, 0.0, 0.0)
        self._carrying = False
        self._disturbed_steps = 0
        self._heading_disturbed_steps = 0
        self.disturbance_count = 0
        self.steps_without_gps = 0

    # ------------------------------------------------------------------
    def set_command(self, forward: float, lateral: float, rotation: float,
                    carrying: bool) -> None:
        """Record the command that the next physics step will apply."""
        f, l = clip_unit_vector(forward, lateral)
        r = max(-1.0, min(1.0, rotation))
        self._last_cmd = (f, l, r)
        self._carrying = carrying

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def position(self) -> Tuple[float, float]:
        return self.x, self.y

    # ------------------------------------------------------------------
    def update(self, gps: Optional[Tuple[float, float]], compass: Optional[float],
               odometer: Optional[Tuple[float, float, float]]) -> None:
        if not self.initialized:
            self._initialize(gps, compass)
            return
        self._predict(odometer)
        self._correct(gps, compass)

    def _initialize(self, gps, compass) -> None:
        if gps is not None:
            self.x, self.y = float(gps[0]), float(gps[1])
            self.position_sigma = P.GPS_NOISE
            self._n_gps = 1
        if compass is not None:
            self.heading = float(compass)
            self.heading_sigma = P.COMPASS_NOISE
            self._n_compass = 1
        self.initialized = gps is not None and compass is not None

    def _predict(self, odometer) -> None:
        f, l, r = self._last_cmd
        gain = P.CARRY_GAIN_FACTOR if self._carrying else 1.0
        c, s = math.cos(self.heading), math.sin(self.heading)
        ux, uy = f * c - l * s, f * s + l * c
        dx = P.POS_VEL * self.vx + P.POS_GAIN * gain * ux
        dy = P.POS_VEL * self.vy + P.POS_GAIN * gain * uy
        dtheta = P.ROT_GAIN * r
        disturbed = self._carrying
        if odometer is not None:
            d_odo, a_odo, t_odo = odometer
            if abs(wrap(t_odo - dtheta)) > P.ODOMETER_ANGLE_TOL:
                dtheta = t_odo
                self._heading_disturbed_steps = 10
            else:
                self._heading_disturbed_steps = max(0, self._heading_disturbed_steps - 1)
            d_pred = math.hypot(dx, dy)
            tol = max(P.ODOMETER_DIST_TOL, 0.08 * d_pred)
            if abs(d_odo - d_pred) > tol or self._carrying:
                # Measured magnitude; direction from the model when it agrees
                # with the noisy odometer bearing, else from the odometer.
                odo_dir = self.heading + a_odo
                if d_pred > 0.5:
                    model_dir = math.atan2(dy, dx)
                    direction = model_dir if abs(wrap(odo_dir - model_dir)) < 0.45 else odo_dir
                else:
                    direction = odo_dir
                dx, dy = d_odo * math.cos(direction), d_odo * math.sin(direction)
                disturbed = disturbed or abs(d_odo - d_pred) > tol
        if disturbed:
            # Velocity consistent with the realized displacement.
            pvx = (dx - P.POS_GAIN * gain * ux) / P.POS_VEL
            pvy = (dy - P.POS_GAIN * gain * uy) / P.POS_VEL
            self.vx = P.VEL_DECAY * pvx + P.VEL_GAIN * gain * ux
            self.vy = P.VEL_DECAY * pvy + P.VEL_GAIN * gain * uy
            self._disturbed_steps = 10
            self.disturbance_count += 1
        else:
            self.vx = P.VEL_DECAY * self.vx + P.VEL_GAIN * gain * ux
            self.vy = P.VEL_DECAY * self.vy + P.VEL_GAIN * gain * uy
            self._disturbed_steps = max(0, self._disturbed_steps - 1)
        self.x += dx
        self.y += dy
        self.heading = wrap(self.heading + dtheta)
        growth = 0.05 + 0.03 * math.hypot(dx, dy) + (0.5 if disturbed else 0.0)
        self.position_sigma = min(1e3, self.position_sigma + growth)
        self.heading_sigma = min(math.pi, self.heading_sigma + math.radians(0.02))

    def _correct(self, gps, compass) -> None:
        if compass is not None:
            self._n_compass += 1
            gain = max(P.HEADING_GAIN_MIN, 1.0 / self._n_compass)
            if self._heading_disturbed_steps > 0:
                gain = max(gain, 0.03)
            self.heading = wrap(self.heading + gain * wrap(float(compass) - self.heading))
            self.heading_sigma = max(math.radians(1.2),
                                     min(self.heading_sigma, P.COMPASS_NOISE * math.sqrt(
                                         max(gain, 1.0 / 99.0) * 99.0)))
        if gps is not None:
            self._n_gps += 1
            self.steps_without_gps = 0
            gain = max(P.POSITION_GAIN_BASE + P.POSITION_GAIN_PER_SPEED * self.speed,
                       1.0 / self._n_gps)
            if self._disturbed_steps > 0:
                gain = max(gain, P.DISTURBED_POSITION_GAIN)
            ex, ey = float(gps[0]) - self.x, float(gps[1]) - self.y
            self.x += gain * ex
            self.y += gain * ey
            self.position_sigma = max(2.5, min(self.position_sigma, P.GPS_NOISE))
        else:
            self.steps_without_gps += 1
