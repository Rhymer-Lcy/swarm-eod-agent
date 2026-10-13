"""Pose estimation from GPS, compass, odometer and the known motion model.

The simulator's motion is deterministic given the commands (see ``params``),
so the estimator dead-reckons with the exact model and uses the sensors for
two things: the odometer detects when the model was violated (contact with a
wall, another drone or a pushed bomb) and then supplies the measured
displacement and rotation, and GPS and compass, whose errors are slowly
varying AR(1) processes, pull the dead-reckoned pose towards the absolute
frame.

Heading (carry-aware since B1). The measured rotation model holds only for
an unloaded drone: with a bomb on the grasper the drone turns by a variable
fraction of the command (median 0.39, 10th to 90th percentile -0.01 to 0.90
in the grasp probe), and the per-step deviations are mostly below any
threshold that the odometer's 1 degree noise allows. The heading prediction
therefore uses the measured odometer rotation, unconditionally, whenever the
step may have been affected by a bomb (the drone carried at the time of the
command or carries now) or the applied command is unknown; it uses the model
only for an unloaded step that the odometer confirms. Without an odometer, a
carrying step is predicted with the median ratio and a large variance.

The heading variance is propagated with the prediction (zero for a
confirmed model step, the odometer variance for a measured step, the
ratio spread for a modelled carrying step) and the compass correction uses
the scalar Kalman gain ``P / (P + R)``, bounded below by a small floor that
keeps slow drift corrected and above by 0.5 so that one compass reading
never replaces the estimate. For an unloaded drone this reproduces the
previous estimator exactly: the model adds no variance, so the gain is the
running average ``1 / n``, and an unloaded step that the odometer shows to
violate the model (contact) keeps the Phase 1 treatment (measured rotation,
gain at least 0.03 for ten steps). While carrying the gain settles near
0.2, which the AR(1)
compass error analysis in docs/AGENT_DESIGN.md (section 3) shows to be close
to the minimum heading error achievable with these two sensors.

Commands are consumed: ``set_command`` records the command that the next
physics step applies and the next ``update`` uses it once. If no command was
recorded (the brain failed and the shell applied a zero command), the step is
treated as unknown.

Every sensor may be missing in a step (disabled or non-finite); the estimate
then continues from the model alone and its uncertainty grows.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from . import params as P
from .geometry import clip_unit_vector, wrap

_COMPASS_VAR = P.COMPASS_NOISE ** 2
_ODOMETER_THETA_VAR = P.ODOMETER_THETA_NOISE ** 2


class PoseEstimator:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.heading = 0.0
        self.initialized = False
        self.position_sigma = 1e3
        self.heading_var = math.pi ** 2
        self._n_gps = 0
        self._cmd: Optional[Tuple[float, float, float]] = None
        self._cmd_carrying = False
        self._disturbed_steps = 0
        self._heading_disturbed_steps = 0
        self.disturbance_count = 0
        self.steps_without_gps = 0
        # Diagnostics: how each heading prediction was made.
        self.carry_odometer_steps = 0
        self.carry_model_steps = 0
        self.contact_rotation_steps = 0
        self.unknown_command_steps = 0

    # ------------------------------------------------------------------
    def set_command(self, forward: float, lateral: float, rotation: float,
                    carrying: bool) -> None:
        """Record the command that the next physics step will apply."""
        f, l = clip_unit_vector(forward, lateral)
        r = max(-1.0, min(1.0, rotation))
        self._cmd = (f, l, r)
        self._cmd_carrying = bool(carrying)

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    @property
    def position(self) -> Tuple[float, float]:
        return self.x, self.y

    @property
    def heading_sigma(self) -> float:
        return math.sqrt(self.heading_var)

    # ------------------------------------------------------------------
    def update(self, gps: Optional[Tuple[float, float]], compass: Optional[float],
               odometer: Optional[Tuple[float, float, float]],
               grasping: Optional[bool] = None) -> None:
        """One step: ``grasping`` is the grasp state observed at this step."""
        if not self.initialized:
            self._cmd = None
            self._initialize(gps, compass)
            return
        self._predict(odometer, grasping)
        self._correct(gps, compass)

    def _initialize(self, gps, compass) -> None:
        if gps is not None:
            self.x, self.y = float(gps[0]), float(gps[1])
            self.position_sigma = P.GPS_NOISE
            self._n_gps = 1
        if compass is not None:
            self.heading = float(compass)
            self.heading_var = _COMPASS_VAR
        self.initialized = gps is not None and compass is not None

    def _predict(self, odometer, grasping: Optional[bool]) -> None:
        cmd = self._cmd
        self._cmd = None
        known = cmd is not None
        if known:
            f, l, r = cmd
            carrying_cmd = self._cmd_carrying
        else:
            # The shell applies a zero command when the brain fails.
            f = l = r = 0.0
            carrying_cmd = bool(grasping)
            self.unknown_command_steps += 1
        carrying_now = carrying_cmd if grasping is None else bool(grasping)
        # A bomb may have affected this step if it was held when the command
        # was issued or is held now (grasp and release transitions).
        carry_step = carrying_cmd or carrying_now
        gain = P.CARRY_GAIN_FACTOR if carrying_cmd else 1.0
        c, s = math.cos(self.heading), math.sin(self.heading)
        ux, uy = f * c - l * s, f * s + l * c
        dx = P.POS_VEL * self.vx + P.POS_GAIN * gain * ux
        dy = P.POS_VEL * self.vy + P.POS_GAIN * gain * uy
        dtheta, rot_var = self._predict_rotation(odometer, r, known, carry_step)
        disturbed = carry_step or not known
        if odometer is not None:
            d_odo, a_odo, _ = odometer
            d_pred = math.hypot(dx, dy)
            tol = max(P.ODOMETER_DIST_TOL, 0.08 * d_pred)
            if abs(d_odo - d_pred) > tol or carry_step or not known:
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
        self.heading_var = min(math.pi ** 2, self.heading_var + rot_var)
        growth = 0.05 + 0.03 * math.hypot(dx, dy) + (0.5 if disturbed else 0.0)
        self.position_sigma = min(1e3, self.position_sigma + growth)

    def _predict_rotation(self, odometer, r: float, known: bool,
                          carry_step: bool) -> Tuple[float, float]:
        """Heading increment of this step and the variance it adds."""
        model = P.ROT_GAIN * r
        if odometer is not None:
            measured = float(odometer[2])
            if carry_step or not known:
                if carry_step:
                    self.carry_odometer_steps += 1
                self._heading_disturbed_steps = max(0, self._heading_disturbed_steps - 1)
                return measured, _ODOMETER_THETA_VAR
            if abs(wrap(measured - model)) > P.ODOMETER_ANGLE_TOL:
                # Unloaded but the model is violated (contact, push): the
                # Phase 1 handling is kept unchanged for unloaded flight.
                self.contact_rotation_steps += 1
                self._heading_disturbed_steps = 10
                return measured, 0.0
            self._heading_disturbed_steps = max(0, self._heading_disturbed_steps - 1)
            return model, 0.0
        if carry_step:
            self.carry_model_steps += 1
            spread = P.CARRY_ROTATION_RATIO_SIGMA * model
            return P.CARRY_ROTATION_RATIO * model, spread * spread + _ODOMETER_THETA_VAR
        if not known:
            # Zero command assumed; allow for an unexpected turn.
            return 0.0, P.ROT_GAIN ** 2 / 3.0
        return model, 0.0

    def _correct(self, gps, compass) -> None:
        if compass is not None:
            gain = self.heading_var / (self.heading_var + _COMPASS_VAR)
            gain = min(P.HEADING_GAIN_MAX, max(P.HEADING_GAIN_MIN, gain))
            if self._heading_disturbed_steps > 0:
                gain = max(gain, P.CONTACT_HEADING_GAIN)
            self.heading = wrap(self.heading + gain * wrap(float(compass) - self.heading))
            self.heading_var *= 1.0 - gain
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
