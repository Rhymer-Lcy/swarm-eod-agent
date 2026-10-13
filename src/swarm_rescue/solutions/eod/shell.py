"""Simulator-facing drone classes.

These are the only classes that touch the framework. They read sensors
exclusively through the official measured API (``measured_gps_position``,
``measured_compass_angle``, ``odometer_values``, ``lidar_values``,
``semantic_values``, ``grasped_bombs``, ``carried_bombs_count``, the
communicator's received messages, ``drone_health``, ``elapsed_timestep``,
``elapsed_walltime`` and ``is_inside_return_area``), translate them into an
``Observation``, run the role's brain and translate its ``Command`` back into
the actuator dictionary. No ground-truth or debug interface is used.

A brain exception never ends the round: the drone holds still for that step
and the error is counted for diagnostics.
"""

from __future__ import annotations

import math
import traceback
from typing import Any, Dict, List, Optional

import numpy as np

from swarm_rescue.simulation.drone.controller import CommandsDict
from swarm_rescue.simulation.drone.drone_abstract import DroneAbstract
from swarm_rescue.simulation.utils.misc_data import MiscData

from .observation import Command, Observation, SemanticHit, finite_or_none


def _finite_tuple(values, n: int) -> Optional[tuple]:
    if values is None:
        return None
    try:
        out = tuple(float(values[i]) for i in range(n))
    except (TypeError, IndexError, ValueError):
        return None
    return out if all(math.isfinite(v) for v in out) else None


class _EodDrone(DroneAbstract):
    """Shared shell: observation building, safe command conversion."""

    PLACE_MODE = False

    def __init__(self, identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None, **kwargs) -> None:
        super().__init__(identifier=identifier, misc_data=misc_data, **kwargs)
        self._brain = None
        self._angles: Optional[np.ndarray] = None
        self._errors = 0
        self._last_error = ""

    # -- framework hooks ---------------------------------------------------
    def define_message_for_all(self) -> Any:
        if self._brain is None:
            return None
        try:
            return self._brain.outgoing_message()
        except Exception:
            self._record_error()
            return None

    def control(self) -> CommandsDict:
        try:
            obs = self._observe()
            if self._brain is None:
                self._brain = self._make_brain(obs)
            cmd = self._brain.step(obs)
            if cmd.submit_map and self.PLACE_MODE:
                grid = self._brain.exploration_map()
                if grid is not None:
                    self.submit_exploration_map(grid)
        except Exception:
            self._record_error()
            cmd = Command()
        return self._to_commands(cmd)

    def diagnostics_report(self) -> Dict[str, Any]:
        report: Dict[str, Any] = {"errors": self._errors, "last_error": self._last_error}
        if self._brain is not None and hasattr(self._brain, "diagnostics"):
            report.update(self._brain.diagnostics())
        return report

    # -- helpers -----------------------------------------------------------
    def _record_error(self) -> None:
        self._errors += 1
        self._last_error = traceback.format_exc(limit=4)[-1500:]

    def _make_brain(self, obs: Observation):
        raise NotImplementedError

    def _observe(self) -> Observation:
        if self._angles is None:
            self._angles = np.asarray(self.lidar_rays_angles(), dtype=float).copy()
        misc = self._misc_data
        size = self.size_area or (misc.size_area if misc is not None else (1000, 1000))
        semantic: Optional[List[SemanticHit]] = None
        raw = self.semantic_values()
        if raw is not None:
            semantic = []
            for data in raw:
                kind = getattr(data.entity_type, "name", None)
                if kind is None or not math.isfinite(float(data.distance)):
                    continue
                semantic.append(SemanticHit(float(data.distance), float(data.angle),
                                            str(kind), bool(data.grasped)))
        messages: List[Any] = []
        if not self.communicator_is_disabled():
            messages = [msg for _, msg in self.communicator.received_messages]
        compass = self.measured_compass_angle()
        compass = float(compass) if compass is not None and math.isfinite(float(compass)) else None
        return Observation(
            step=int(self.elapsed_timestep),
            walltime=float(self.elapsed_walltime),
            identifier=int(self.identifier),
            n_drones=int(misc.number_drones) if misc is not None and misc.number_drones else 1,
            size_area=(int(size[0]), int(size[1])),
            max_steps=getattr(misc, "max_timestep_limit", None),
            max_walltime=getattr(misc, "max_walltime_limit", None),
            gps=_finite_tuple(self.measured_gps_position(), 2),
            compass=compass,
            odometer=_finite_tuple(self.odometer_values(), 3),
            lidar=finite_or_none(self.lidar_values()),
            lidar_angles=self._angles,
            semantic=semantic,
            grasping=len(self.grasped_bombs()) > 0,
            health=int(self.drone_health),
            inventory=int(self.carried_bombs_count()),
            messages=messages,
            comm_disabled=bool(self.communicator_is_disabled()),
            in_return_area=bool(self.is_inside_return_area),
        )

    def _to_commands(self, cmd: Command) -> CommandsDict:
        def unit(v: float) -> float:
            v = float(v)
            return max(-1.0, min(1.0, v)) if math.isfinite(v) else 0.0

        out: CommandsDict = {
            "forward": unit(cmd.forward),
            "lateral": unit(cmd.lateral),
            "rotation": unit(cmd.rotation),
            "grasper": 1 if cmd.grasper else 0,
        }
        if self.PLACE_MODE:
            out["place_bomb"] = 1 if cmd.place_bomb else 0
        return out


class EodRescueDrone(_EodDrone):
    """Blue team drone (rescue mode)."""

    def _make_brain(self, obs: Observation):
        from .blue import RescueBrain
        return RescueBrain(obs.identifier, obs.n_drones, obs.size_area, obs.max_steps)


class EodPlaceDrone(_EodDrone):
    """Red team drone (place mode)."""

    PLACE_MODE = True

    def _make_brain(self, obs: Observation):
        from .red import PlaceBrain
        return PlaceBrain(obs.identifier, obs.n_drones, obs.size_area, obs.max_steps,
                          obs.max_walltime)
