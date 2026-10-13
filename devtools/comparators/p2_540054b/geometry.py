"""Angles, frames and grid indexing.

World frame: origin at the map centre, x to the right, y up (the GPS frame).
Headings are counter-clockwise from +x. Body frame: +x forward, +y left.
Image indexing of a W x H map follows the scorer: ``col = round(x + W/2)``,
``row = round(H/2 - y)``.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def wrap_array(angles: np.ndarray) -> np.ndarray:
    return (angles + np.pi) % (2.0 * np.pi) - np.pi


def world_to_body(vx: float, vy: float, heading: float) -> Tuple[float, float]:
    c, s = math.cos(heading), math.sin(heading)
    return vx * c + vy * s, -vx * s + vy * c


def body_to_world(fx: float, fy: float, heading: float) -> Tuple[float, float]:
    c, s = math.cos(heading), math.sin(heading)
    return fx * c - fy * s, fx * s + fy * c


def clip_unit_vector(fx: float, fy: float) -> Tuple[float, float]:
    """Apply the simulator's thrust rule: clip each axis, then cap the norm."""
    fx = max(-1.0, min(1.0, fx))
    fy = max(-1.0, min(1.0, fy))
    norm = math.hypot(fx, fy)
    if norm > 1.0:
        fx, fy = fx / norm, fy / norm
    return fx, fy


def to_pixel(x: float, y: float, width: int, height: int) -> Tuple[int, int]:
    """World point to (row, col) of a 1 px image, without bounds checking."""
    return int(round(height / 2.0 - y)), int(round(x + width / 2.0))
