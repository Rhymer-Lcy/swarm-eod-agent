"""Coarse occupancy grid and grid planning.

The grid stores log-odds on square cells (``params.CELL`` px). A lidar scan
marks the cells of ray end points as occupied and the cells the rays crossed
as free; each cell is updated at most once per scan. Rays known to end on a
non-wall object (drone, bomb, disposal center) are passed in as "carve only":
they clear space up to the object but add no obstacle.

Planning uses geodesic cost fields: ``skimage.graph.MCP_Geometric`` when it
can be imported (it is compiled and computes a full field on the largest
supplied map in a few milliseconds), otherwise a breadth-first wavefront
built from OpenCV dilations. Costs are positive on traversable cells and
negative (impassable) on inflated obstacles.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from . import params as P

try:  # compiled Dijkstra on grids; optional
    from skimage.graph import MCP_Geometric as _MCP
except Exception:  # pragma: no cover - depends on the evaluator image
    _MCP = None

Cell = Tuple[int, int]
UNKNOWN_COST = 3.0         # default per-cell cost of unobserved cells


class OccupancyGrid:
    def __init__(self, size_area: Tuple[int, int], cell: float = P.CELL) -> None:
        self.width, self.height = int(size_area[0]), int(size_area[1])
        self.cell = float(cell)
        self.nx = int(math.ceil(self.width / self.cell))
        self.ny = int(math.ceil(self.height / self.cell))
        self.logodds = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.observed = np.zeros((self.ny, self.nx), dtype=bool)
        self.version = 0
        self._occ_cache: Optional[Tuple[int, np.ndarray]] = None
        steps = np.arange(1, int(P.LIDAR_RANGE / (self.cell * 0.5)) + 1)
        self._samples = steps * (self.cell * 0.5)

    # -- indexing -----------------------------------------------------------
    def to_cell(self, x: float, y: float) -> Cell:
        col = int((x + self.width / 2.0) // self.cell)
        row = int((self.height / 2.0 - y) // self.cell)
        return min(max(row, 0), self.ny - 1), min(max(col, 0), self.nx - 1)

    def to_world(self, row: int, col: int) -> Tuple[float, float]:
        return ((col + 0.5) * self.cell - self.width / 2.0,
                self.height / 2.0 - (row + 0.5) * self.cell)

    def _cells(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        cols = np.floor((xs + self.width / 2.0) / self.cell).astype(np.int64)
        rows = np.floor((self.height / 2.0 - ys) / self.cell).astype(np.int64)
        ok = (cols >= 0) & (cols < self.nx) & (rows >= 0) & (rows < self.ny)
        return np.where(ok, rows * self.nx + cols, -1)

    # -- updates ---------------------------------------------------------------
    def integrate_scan(self, x: float, y: float, heading: float,
                       ranges: np.ndarray, angles: np.ndarray,
                       hit_valid: np.ndarray, max_free: float = P.FREE_RANGE,
                       max_hit: float = P.LIDAR_NO_HIT, ray_stride: int = 2) -> None:
        """Update the grid with one lidar scan taken at pose (x, y, heading).

        ``hit_valid`` marks rays whose return is a wall candidate; other rays
        only carve free space up to their return.
        """
        idx = np.arange(0, len(ranges), ray_stride)
        r = ranges[idx]
        a = angles[idx] + heading
        is_hit = (r < max_hit) & hit_valid[idx]
        free_len = np.where(r < max_hit, r - 0.75 * self.cell, max_free)
        free_len = np.minimum(free_len, max_free)
        t = self._samples[None, :]
        mask = t < free_len[:, None]
        px = x + t * np.cos(a)[:, None]
        py = y + t * np.sin(a)[:, None]
        # Boolean scratch masks give each touched cell exactly once (the same
        # sorted unique cells as np.unique, without sorting).
        n_cells = self.nx * self.ny
        free_mask = np.zeros(n_cells, dtype=bool)
        cells = self._cells(px[mask], py[mask])
        free_mask[cells[cells >= 0]] = True
        free_cells = np.flatnonzero(free_mask)
        flat = self.logodds.reshape(-1)
        obs = self.observed.reshape(-1)
        flat[free_cells] += P.LOGODDS_FREE
        obs[free_cells] = True
        hx = x + r[is_hit] * np.cos(a[is_hit])
        hy = y + r[is_hit] * np.sin(a[is_hit])
        hit_mask = np.zeros(n_cells, dtype=bool)
        cells = self._cells(hx, hy)
        hit_mask[cells[cells >= 0]] = True
        hit_cells = np.flatnonzero(hit_mask)
        flat[hit_cells] += P.LOGODDS_HIT - P.LOGODDS_FREE * free_mask[hit_cells]
        obs[hit_cells] = True
        np.clip(self.logodds, P.LOGODDS_MIN, P.LOGODDS_MAX, out=self.logodds)
        self.version += 1

    def mark_box(self, x0: float, y0: float, x1: float, y1: float,
                 value: float = P.LOGODDS_MAX) -> None:
        """Force an axis-aligned world box to a log-odds value (e.g. obstacle)."""
        r0, c0 = self.to_cell(x0, y1)
        r1, c1 = self.to_cell(x1, y0)
        self.logodds[r0:r1 + 1, c0:c1 + 1] = value
        self.observed[r0:r1 + 1, c0:c1 + 1] = True
        self.version += 1

    # -- classification ------------------------------------------------------
    def occupied(self) -> np.ndarray:
        """Occupied cells (read-only, cached per grid ``version``).

        Code that writes ``logodds`` directly must increment ``version``.
        """
        cache = self._occ_cache
        if cache is None or cache[0] != self.version:
            occ = self.logodds > P.OCCUPIED_THRESHOLD
            occ.flags.writeable = False
            cache = (self.version, occ)
            self._occ_cache = cache
        return cache[1]

    def free(self) -> np.ndarray:
        return self.logodds < P.FREE_THRESHOLD

    def unknown(self) -> np.ndarray:
        return ~self.occupied() & ~self.free()

    def frontier(self) -> np.ndarray:
        """Free cells with an unknown 4-neighbour that is not next to a wall.

        Unknown slivers between carved free space and an observed wall are
        not frontiers: there is nothing behind them to explore.
        """
        near_wall = cv2.dilate(self.occupied().astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        unk = (self.unknown() & ~near_wall).astype(np.uint8)
        kernel = np.array([[0, 1, 0], [1, 0, 1], [0, 1, 0]], np.uint8)
        near_unknown = cv2.dilate(unk, kernel) > 0
        border = np.zeros_like(near_unknown)
        border[[0, -1], :] = True
        border[:, [0, -1]] = True
        return self.free() & near_unknown & ~border

    def clear_along(self, x: float, y: float, x1: float, y1: float,
                    stop_short: float = 0.0) -> bool:
        """True when the segment crosses no occupied cell.

        ``stop_short`` ignores the last part of the segment (for targets that
        lie on an obstacle surface).
        """
        if stop_short > 0.0:
            length = math.hypot(x1 - x, y1 - y)
            if length <= stop_short:
                return True
            k = (length - stop_short) / length
            x1, y1 = x + k * (x1 - x), y + k * (y1 - y)
        n = int(max(abs(x1 - x), abs(y1 - y)) / (self.cell * 0.5)) + 2
        xs = np.linspace(x, x1, n)
        ys = np.linspace(y, y1, n)
        cells = self._cells(xs, ys)
        cells = cells[cells >= 0]
        return not np.any(self.occupied().reshape(-1)[cells])


class Planner:
    """Cost maps and geodesic fields on an ``OccupancyGrid``."""

    def __init__(self, grid: OccupancyGrid) -> None:
        self.grid = grid
        r = int(math.ceil(P.INFLATE_RADIUS / grid.cell))
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        self._near = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 5, 2 * r + 5))
        self._cost_cache: Dict[tuple, np.ndarray] = {}
        self.extra_blocked: List[Tuple[float, float, float, int]] = []  # x, y, radius, until_step

    def cost(self, allow_unknown: bool, step: int = 0,
             unknown_cost: float = UNKNOWN_COST) -> np.ndarray:
        g = self.grid
        key = (g.version, allow_unknown, step // 20, unknown_cost)
        cached = self._cost_cache.get(key)
        if cached is not None:
            return cached
        occ = g.occupied().astype(np.uint8)
        for bx, by, rad, until in self.extra_blocked:
            if until >= step:
                r, c = g.to_cell(bx, by)
                k = int(math.ceil(rad / g.cell))
                occ[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1] = 1
        self.extra_blocked = [b for b in self.extra_blocked if b[3] >= step]
        inflated = cv2.dilate(occ, self._kernel) > 0
        near = cv2.dilate(occ, self._near) > 0
        cost = np.ones(occ.shape, dtype=np.float64)
        cost[near] = 2.5
        unknown = g.unknown()
        if allow_unknown:
            cost[unknown] = np.maximum(cost[unknown], unknown_cost)
        else:
            cost[unknown] = -1.0
        cost[inflated] = -1.0
        # A few variants (allow_unknown, unknown_cost) of the current grid.
        self._cost_cache = {k: v for k, v in self._cost_cache.items() if k[:3] == key[:3]}
        self._cost_cache[key] = cost
        return cost

    def field(self, sources: Sequence[Cell], allow_unknown: bool,
              step: int = 0, unknown_cost: float = UNKNOWN_COST) -> np.ndarray:
        """Geodesic cost-to-go from the nearest source; inf where unreachable.

        ``unknown_cost`` is the per-cell cost of unobserved cells when they
        are allowed (free cells cost 1, cells near obstacles 2.5).
        """
        cost = self.cost(allow_unknown, step, unknown_cost).copy()
        sources = [s for s in sources]
        if not sources:
            return np.full(cost.shape, np.inf)
        occupied = self.grid.occupied()
        k = int(math.ceil(P.INFLATE_RADIUS / self.grid.cell))
        for r, c in sources:
            # A goal inside the inflation margin (a bomb near a wall, the
            # standoff of the disposal center) gets an expensive approach
            # corridor through inflated, not occupied, cells.
            r0, r1 = max(r - k, 0), min(r + k + 1, cost.shape[0])
            c0, c1 = max(c - k, 0), min(c + k + 1, cost.shape[1])
            patch = cost[r0:r1, c0:c1]
            blocked = (patch < 0) & ~occupied[r0:r1, c0:c1]
            patch[blocked] = 4.0
            if cost[r, c] < 0:
                cost[r, c] = 1.0
        if _MCP is not None:
            mcp = _MCP(np.where(cost < 0, np.inf, cost), fully_connected=True)
            cum, _ = mcp.find_costs(sources)
            cum = np.asarray(cum, dtype=np.float64)
            cum[~np.isfinite(cum)] = np.inf
            return cum
        return _wavefront(cost, sources)

    @staticmethod
    def descend(field: np.ndarray, start: Cell, max_len: int = 400) -> List[Cell]:
        """Steepest descent on a field from ``start`` towards a source."""
        path = [start]
        r, c = start
        ny, nx = field.shape
        if not np.isfinite(field[r, c]):
            # The start lies inside an inflation margin (a drone close to a
            # wall): begin from the cheapest reachable cell nearby instead
            # of giving up on planning.
            found = None
            for rad in range(1, 5):
                r0, r1 = max(r - rad, 0), min(r + rad + 1, ny)
                c0, c1 = max(c - rad, 0), min(c + rad + 1, nx)
                window = field[r0:r1, c0:c1]
                if np.isfinite(window).any():
                    k = int(np.argmin(np.where(np.isfinite(window), window, np.inf)))
                    found = (r0 + k // (c1 - c0), c0 + k % (c1 - c0))
                    break
            if found is None:
                return []
            r, c = found
            path.append(found)
        for _ in range(max_len):
            best = (field[r, c], r, c)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if (dr or dc) and 0 <= rr < ny and 0 <= cc < nx and field[rr, cc] < best[0]:
                        best = (field[rr, cc], rr, cc)
            if best[1] == r and best[2] == c:
                break
            r, c = best[1], best[2]
            path.append((r, c))
        return path


def _wavefront(cost: np.ndarray, sources: Sequence[Cell]) -> np.ndarray:
    """Breadth-first geodesic distance (8-connected) using OpenCV dilations."""
    free = (cost > 0).astype(np.uint8)
    dist = np.full(cost.shape, np.inf)
    front = np.zeros(cost.shape, np.uint8)
    for r, c in sources:
        front[r, c] = 1
        free[r, c] = 1
    visited = front.copy()
    dist[front > 0] = 0.0
    kernel = np.ones((3, 3), np.uint8)
    level = 0
    while True:
        level += 1
        nxt = cv2.dilate(front, kernel) & free & (1 - visited)
        if not nxt.any():
            break
        dist[nxt > 0] = level
        visited |= nxt
        front = nxt
    return dist
