"""Reachable approach positions in front of the disposal center (blue, B2).

The semantic sensor does not report walls, but walls stop its rays: a
disposal-center surface point detected from position ``o`` proves that the
straight segment from ``o`` to that point was free when it was seen. A
stand-off position is therefore taken on that segment, a fixed distance out
from the surface point (``STANDOFF``), instead of outward from the center's
middle: the latter can fall behind a wall that runs a few pixels from a face
of the center, which is how the Phase 2 Map03 carriers ended up pushing into
walls.

``ApproachSet`` keeps a bounded set of such witnessed stand-offs, one per
grid cell (the newest wins). Before they are used as planning goals they are
re-checked against the current occupancy grid (the stand-off cell is not an
obstacle and the segment to the surface point crosses no occupied cell, which
discards observations made with a pose error that the map later
contradicts), and areas where an approach has failed are excluded for a
while, with the exclusion growing on repeated failures.

The module is pure Python and uses only the agent's own estimates and map.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .grid import OccupancyGrid

Cell = Tuple[int, int]

STANDOFF = 36.0            # stand-off distance from the detected surface point, px
SURFACE_STOP = 12.0        # last part of a segment ignored (the center itself is mapped)
MAX_WITNESSES = 400        # bounded memory: oldest witnesses are dropped beyond this
PENALTY_RADIUS = 56.0
PENALTY_STEPS = 600
PENALTY_MAX_STEPS = 2400
MAX_PENALTIES = 32


def segments_clear(grid: OccupancyGrid, x0: np.ndarray, y0: np.ndarray,
                   x1: np.ndarray, y1: np.ndarray, stop_short: float = 0.0) -> np.ndarray:
    """Vectorized ``OccupancyGrid.clear_along`` for many segments at once.

    Same result as calling ``clear_along`` per segment: each segment is
    sampled at the same points (its own count, every half cell), and a
    segment no longer than ``stop_short`` is clear.
    """
    x0, y0 = np.asarray(x0, float), np.asarray(y0, float)
    x1, y1 = np.asarray(x1, float), np.asarray(y1, float)
    if x0.size == 0:
        return np.zeros(0, dtype=bool)
    length = np.hypot(x1 - x0, y1 - y0)
    short = length <= stop_short
    if stop_short > 0.0:
        k = np.where(short, 1.0, (length - stop_short) / np.maximum(length, 1e-12))
        x1, y1 = x0 + k * (x1 - x0), y0 + k * (y1 - y0)
    n = (np.maximum(np.abs(x1 - x0), np.abs(y1 - y0)) / (grid.cell * 0.5)).astype(np.int64) + 2
    idx = np.arange(int(n.max()))[None, :]
    inside = idx < n[:, None]
    last = idx == (n - 1)[:, None]
    # The arithmetic of numpy.linspace (index times step plus start, last
    # sample exactly at the end), so that sampled cells agree bit for bit.
    xs = idx * ((x1 - x0) / (n - 1))[:, None] + x0[:, None]
    ys = idx * ((y1 - y0) / (n - 1))[:, None] + y0[:, None]
    xs = np.where(last, x1[:, None], xs)
    ys = np.where(last, y1[:, None], ys)
    cells = grid._cells(xs.reshape(-1), ys.reshape(-1)).reshape(xs.shape)
    occ = grid.occupied().reshape(-1)
    blocked = inside & (cells >= 0) & occ[np.maximum(cells, 0)]
    return ~np.any(blocked, axis=1) | short


class ApproachSet:
    """Witnessed stand-off positions of the disposal center, with penalties."""

    def __init__(self) -> None:
        # cell -> (standoff x, standoff y, surface x, surface y, step)
        self.witnesses: Dict[Cell, Tuple[float, float, float, float, int]] = {}
        # (x, y, radius, until_step, level)
        self.penalties: List[Tuple[float, float, float, int, int]] = []
        self.version = 0           # incremented when penalties change

    def __len__(self) -> int:
        return len(self.witnesses)

    # -- recording -------------------------------------------------------------
    def observe(self, grid: OccupancyGrid, ox: float, oy: float,
                points: Sequence[Tuple[float, float]], step: int) -> None:
        """Record stand-offs witnessed from ``(ox, oy)`` for detected points."""
        for px, py in points:
            dx, dy = ox - px, oy - py
            dist = math.hypot(dx, dy)
            if dist < 1e-6:
                continue
            # Closer than the stand-off distance, the drone's own position is
            # the witnessed free point (the ray only proves the segment).
            k = min(STANDOFF, dist) / dist
            sx, sy = px + k * dx, py + k * dy
            self.witnesses[grid.to_cell(sx, sy)] = (sx, sy, px, py, step)
        if len(self.witnesses) > MAX_WITNESSES + 50:   # trimmed in batches
            newest = sorted(self.witnesses.items(), key=lambda kv: -kv[1][4])
            self.witnesses = dict(newest[:MAX_WITNESSES])

    # -- penalties ---------------------------------------------------------------
    def penalize(self, x: float, y: float, step: int) -> int:
        """Exclude witnesses near ``(x, y)``; repeated failures there last longer.

        Returns the exclusion length in steps.
        """
        level = 0
        for px, py, _, until, lvl in self.penalties:
            if math.hypot(px - x, py - y) < PENALTY_RADIUS:
                level = max(level, lvl + 1)
        duration = min(PENALTY_STEPS * (2 ** level), PENALTY_MAX_STEPS)
        self.penalties.append((x, y, PENALTY_RADIUS, step + duration, level))
        self.penalties = self.penalties[-MAX_PENALTIES:]
        self.version += 1
        return duration

    def penalized_at(self, x: float, y: float, step: int) -> bool:
        """Whether ``(x, y)`` lies in an area where an approach failed recently."""
        return bool(self._penalized(np.array([x]), np.array([y]), step)[0])

    def _penalized(self, xs: np.ndarray, ys: np.ndarray, step: int) -> np.ndarray:
        out = np.zeros(xs.shape, dtype=bool)
        for px, py, rad, until, _ in self.penalties:
            if until >= step:
                out |= (xs - px) ** 2 + (ys - py) ** 2 < rad * rad
        return out

    # -- use -----------------------------------------------------------------------
    def valid(self, grid: OccupancyGrid, step: int, ignore_penalties: bool = False
              ) -> List[Tuple[float, float, float, float]]:
        """Witnesses that the current map supports (and not penalized)."""
        if not self.witnesses:
            return []
        arr = np.array([w[:4] for w in self.witnesses.values()], dtype=float)
        sx, sy, px, py = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
        cells = grid._cells(sx, sy)
        occ = grid.occupied().reshape(-1)
        ok = (cells >= 0) & ~occ[np.maximum(cells, 0)]
        ok &= segments_clear(grid, sx, sy, px, py, stop_short=SURFACE_STOP)
        if not ignore_penalties:
            ok &= ~self._penalized(sx, sy, step)
        return [tuple(row) for row in arr[ok]]

    def sources(self, grid: OccupancyGrid, step: int) -> Tuple[List[Cell], bool]:
        """Planning goal cells and whether penalties had to be ignored.

        When every supported witness is penalized, all supported witnesses
        are returned again rather than none (an approach is always tried).
        """
        good = self.valid(grid, step)
        relaxed = False
        if not good:
            good = self.valid(grid, step, ignore_penalties=True)
            relaxed = bool(good)
        return sorted({grid.to_cell(w[0], w[1]) for w in good}), relaxed

    def at_standoff(self, grid: OccupancyGrid, x: float, y: float, step: int,
                    radius: float = 20.0) -> Optional[Tuple[float, float]]:
        """Surface point of the nearest supported stand-off within ``radius``."""
        best, best_d = None, radius
        for sx, sy, px, py in self.valid(grid, step):
            d = math.hypot(sx - x, sy - y)
            if d < best_d:
                best, best_d = (px, py), d
        return best


def radial_sources(grid: OccupancyGrid, points: Sequence[Tuple[float, float]],
                   box: Tuple[float, float, float, float], offset: float = 32.0
                   ) -> List[Cell]:
    """Phase 2 stand-offs (outward from the center's middle), map-checked.

    Used only when no witnessed stand-off exists (for example when the center
    is known from a teammate's box alone). A stand-off whose cell is an
    obstacle or whose segment to its surface point crosses an occupied cell
    is dropped; if nothing remains, the unchecked set is returned.
    """
    cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    pts = list(points[::5]) or [(cx, cy)]
    sx, sy, px, py = [], [], [], []
    for qx, qy in pts:
        dx, dy = qx - cx, qy - cy
        norm = math.hypot(dx, dy) or 1.0
        sx.append(qx + offset * dx / norm)
        sy.append(qy + offset * dy / norm)
        px.append(qx)
        py.append(qy)
    sx, sy = np.array(sx), np.array(sy)
    cells = grid._cells(sx, sy)
    occ = grid.occupied().reshape(-1)
    ok = (cells >= 0) & ~occ[np.maximum(cells, 0)]
    ok &= segments_clear(grid, sx, sy, np.array(px), np.array(py), stop_short=SURFACE_STOP)
    chosen = [i for i in range(len(sx)) if ok[i]] or list(range(len(sx)))
    return sorted({grid.to_cell(sx[i], sy[i]) for i in chosen})
