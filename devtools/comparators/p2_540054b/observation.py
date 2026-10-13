"""Data contracts between the simulator shell and the agent brains.

``Observation`` carries exactly what the official sensor API returned in one
control step (``None`` where a sensor is disabled); ``Command`` is what the
brain wants applied. Neither type references simulator objects.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List, Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class SemanticHit:
    distance: float          # to the detected surface, px (biased, see params)
    angle: float             # ray angle relative to the heading, rad, CCW
    kind: str                # "BOMB", "DISPOSAL_CENTER", "DRONE" or "OTHER"
    grasped: bool            # bomb currently held by some drone


@dataclass
class Observation:
    step: int
    walltime: float
    identifier: int
    n_drones: int
    size_area: Tuple[int, int]
    max_steps: Optional[int]
    max_walltime: Optional[float]
    gps: Optional[Tuple[float, float]]
    compass: Optional[float]
    odometer: Optional[Tuple[float, float, float]]
    lidar: Optional[np.ndarray]
    lidar_angles: np.ndarray
    semantic: Optional[List[SemanticHit]]
    grasping: bool
    health: int
    inventory: int
    messages: List[Any] = field(default_factory=list)
    comm_disabled: bool = False
    in_return_area: bool = False


@dataclass
class Command:
    forward: float = 0.0
    lateral: float = 0.0
    rotation: float = 0.0
    grasper: int = 0
    place_bomb: int = 0
    submit_map: bool = False


def finite_or_none(values: Optional[np.ndarray]) -> Optional[np.ndarray]:
    """Return a float copy, or None when the array is missing or not finite."""
    if values is None:
        return None
    arr = np.asarray(values, dtype=float).copy()
    if arr.size == 0 or not np.all(np.isfinite(arr)):
        return None
    return arr
