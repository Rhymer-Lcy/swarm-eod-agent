"""Turning raw semantic and lidar readings into world-frame facts.

* Bomb detections: adjacent semantic rays that report a bomb at a similar
  distance are grouped; the bearing is refined with the 2-degree lidar rays
  that return at the same range, and the centre is placed one bomb radius
  behind the detected surface.
* Lidar masking: a lidar return that lands on a drone, a bomb or the
  disposal center must not be mapped as a wall. Rays near a semantic entity
  detection at a matching range, rays ending near a known teammate or bomb,
  and rays ending inside the disposal-center box are flagged.
* Disposal center: semantic hits on the center are surface points; the
  model keeps a bounded set of them and a robust bounding box.
"""

from __future__ import annotations

import math
from typing import List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

from . import params as P
from .observation import SemanticHit

LIDAR_STEP = 2.0 * math.pi / (P.LIDAR_RAYS - 1)


def lidar_endpoints(x: float, y: float, heading: float, lidar: np.ndarray,
                    angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    a = angles + heading
    return x + lidar * np.cos(a), y + lidar * np.sin(a)


class BombDetection(NamedTuple):
    x: float            # estimated centre, world frame
    y: float
    grasped: bool
    range: float        # centre distance from the drone, px
    bearing: float      # relative to the heading, rad


def bomb_detections(x: float, y: float, heading: float,
                    semantic: Sequence[SemanticHit],
                    lidar: Optional[np.ndarray], angles: np.ndarray
                    ) -> List[BombDetection]:
    """Bomb detections of this step, with world and relative coordinates."""
    hits = sorted((h for h in semantic if h.kind == "BOMB"), key=lambda h: h.angle)
    groups: List[List[SemanticHit]] = []
    for h in hits:
        if groups and abs(h.angle - groups[-1][-1].angle) < 0.25 and \
                abs(h.distance - groups[-1][-1].distance) < 15.0:
            groups[-1].append(h)
        else:
            groups.append([h])
    if len(groups) > 1 and abs((groups[0][0].angle + 2 * math.pi) - groups[-1][-1].angle) < 0.25 \
            and abs(groups[0][0].distance - groups[-1][-1].distance) < 15.0:
        groups[0] = groups[-1] + groups[0]
        groups.pop()
    out = []
    for g in groups:
        surface = min(h.distance for h in g) - P.SEMANTIC_BIAS
        sx = sum(math.cos(h.angle) for h in g)
        sy = sum(math.sin(h.angle) for h in g)
        bearing = math.atan2(sy, sx)
        rng = surface
        if lidar is not None:
            diff = np.abs((angles - bearing + math.pi) % (2 * math.pi) - math.pi)
            sel = (diff < math.radians(9.0)) & (np.abs(lidar - surface) < 9.0)
            if np.any(sel):
                bearing = math.atan2(np.sum(np.sin(angles[sel])), np.sum(np.cos(angles[sel])))
                rng = float(np.min(lidar[sel]))
        center = rng + P.BOMB_RADIUS
        a = heading + bearing
        out.append(BombDetection(x + center * math.cos(a), y + center * math.sin(a),
                                 any(h.grasped for h in g), center, bearing))
    return out


def entity_mask(semantic: Optional[Sequence[SemanticHit]], lidar: np.ndarray,
                angles: np.ndarray) -> np.ndarray:
    """Lidar rays whose return matches a semantic entity detection."""
    mask = np.zeros(len(lidar), dtype=bool)
    if not semantic:
        return mask
    for h in semantic:
        surface = h.distance - P.SEMANTIC_BIAS
        diff = np.abs((angles - h.angle + math.pi) % (2 * math.pi) - math.pi)
        width = math.radians(7.0) + (0.12 if h.kind == "DISPOSAL_CENTER" else 0.0)
        mask |= (diff < width) & (np.abs(lidar - surface) < 14.0)
    return mask


def near_points_mask(ex: np.ndarray, ey: np.ndarray,
                     points: Sequence[Tuple[float, float]], radius: float) -> np.ndarray:
    mask = np.zeros(ex.shape, dtype=bool)
    for px, py in points:
        mask |= (ex - px) ** 2 + (ey - py) ** 2 < radius * radius
    return mask


class DisposalCenterModel:
    """Surface points of the disposal center and a robust bounding box."""

    MAX_POINTS = 600

    def __init__(self) -> None:
        self.points: List[Tuple[float, float]] = []
        self.box: Optional[Tuple[float, float, float, float]] = None
        self._dirty = False
        self.shared_box: Optional[Tuple[float, float, float, float]] = None

    def add(self, px: float, py: float) -> None:
        if len(self.points) >= self.MAX_POINTS:
            self.points[len(self.points) // 2:] = self.points[len(self.points) // 2 + 1:]
        self.points.append((px, py))
        self._dirty = True

    def observe(self, x: float, y: float, heading: float,
                semantic: Optional[Sequence[SemanticHit]]) -> None:
        if not semantic:
            return
        for h in semantic:
            if h.kind == "DISPOSAL_CENTER":
                d = h.distance - P.SEMANTIC_BIAS + 2.0
                a = heading + h.angle
                self.add(x + d * math.cos(a), y + d * math.sin(a))

    def bbox(self) -> Optional[Tuple[float, float, float, float]]:
        if self._dirty and len(self.points) >= 4:
            arr = np.array(self.points)
            lo = np.percentile(arr, 2, axis=0)
            hi = np.percentile(arr, 98, axis=0)
            self.box = (float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]))
            self._dirty = False
        if self.box is None:
            return self.shared_box
        return self.box

    @property
    def known(self) -> bool:
        return self.bbox() is not None

    def nearest_surface(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """Closest observed surface point (falls back to the box)."""
        if self.points:
            arr = np.asarray(self.points)
            i = int(np.argmin((arr[:, 0] - x) ** 2 + (arr[:, 1] - y) ** 2))
            return float(arr[i, 0]), float(arr[i, 1])
        box = self.bbox()
        if box is None:
            return None
        return min(max(x, box[0]), box[2]), min(max(y, box[1]), box[3])

    def distance(self, x: float, y: float) -> float:
        box = self.bbox()
        if box is None:
            return float("inf")
        dx = max(box[0] - x, 0.0, x - box[2])
        dy = max(box[1] - y, 0.0, y - box[3])
        return math.hypot(dx, dy)

    def inside(self, x: np.ndarray, y: np.ndarray, margin: float) -> np.ndarray:
        box = self.bbox()
        if box is None:
            return np.zeros(np.shape(x), dtype=bool)
        return ((x >= box[0] - margin) & (x <= box[2] + margin) &
                (y >= box[1] - margin) & (y <= box[3] + margin))
