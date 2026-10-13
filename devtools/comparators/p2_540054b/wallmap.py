"""Wall evidence and the submitted exploration map (red team).

Evidence: every validated lidar return within ``MAP_RANGE`` adds one count
to a 1 px grid at the return point, pushed 1.5 px further along the ray so
that it falls inside the wall rather than on its face. Returns from drones,
bombs and the disposal center are filtered out before they get here.

Sharing: a drone's own counts are encoded as a compressed sparse snapshot;
teammates keep the newest snapshot of every source and sum them (each source
once) when they need the team map.

Rendering (``render``): the summed counts are smoothed, and a pixel is drawn
when its smoothed evidence is high in absolute terms and relative to the
local peak, which yields a band around the evidence ridge whose width
follows the observed spread. The arena border is the one wall whose position
is known up to its thickness: the border runs along the map edge, so a
border segment for which evidence was observed is drawn from the edge to the
observed inner face, and only between its outermost observed positions plus
``SPAN_MARGIN``. Unobserved border segments are not drawn, so a gap in the
border is never filled more than ``SPAN_MARGIN`` px beyond observed returns.
Inputs are the map dimensions, observed lidar returns and the estimated
pose; no wall coordinates of any supplied map are used. The disposal center
is removed from the result; it is not a wall.
"""

from __future__ import annotations

import math
import zlib
from typing import Dict, Iterable, Optional, Tuple

import cv2
import numpy as np

MAP_RANGE = 170.0
INSIDE_OFFSET = 1.5
SEGMENT = 40           # border segment length in px
SPAN_MARGIN = 12       # px drawn beyond the outermost evidence of a segment


class WallEvidence:
    def __init__(self, size_area: Tuple[int, int]) -> None:
        self.width, self.height = int(size_area[0]), int(size_area[1])
        self.counts = np.zeros((self.height, self.width), dtype=np.uint16)
        self.total = 0

    def add_returns(self, x: float, y: float, heading: float, ranges: np.ndarray,
                    angles: np.ndarray, valid: np.ndarray) -> None:
        use = valid & (ranges < MAP_RANGE)
        if not np.any(use):
            return
        r = ranges[use] + INSIDE_OFFSET
        a = angles[use] + heading
        cols = np.round(x + r * np.cos(a) + self.width / 2.0).astype(np.int64)
        rows = np.round(self.height / 2.0 - (y + r * np.sin(a))).astype(np.int64)
        ok = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        flat = rows[ok] * self.width + cols[ok]
        view = self.counts.reshape(-1)
        np.add.at(view, flat, 1)
        np.minimum(view, 60000, out=view)
        self.total += int(ok.sum())

    def snapshot(self) -> bytes:
        idx = np.flatnonzero(self.counts).astype(np.uint32)
        vals = self.counts.reshape(-1)[idx].astype(np.uint16)
        header = np.array([self.width, self.height, idx.size], dtype=np.uint32)
        return zlib.compress(header.tobytes() + idx.tobytes() + vals.tobytes(), 3)


def decode_snapshot(blob: bytes, shape: Tuple[int, int]) -> Optional[np.ndarray]:
    try:
        raw = zlib.decompress(blob)
        header = np.frombuffer(raw[:12], dtype=np.uint32)
        w, h, n = int(header[0]), int(header[1]), int(header[2])
        if (h, w) != shape:
            return None
        idx = np.frombuffer(raw[12:12 + 4 * n], dtype=np.uint32)
        vals = np.frombuffer(raw[12 + 4 * n:12 + 6 * n], dtype=np.uint16)
        grid = np.zeros(h * w, dtype=np.float32)
        grid[idx] = vals
        return grid.reshape(h, w)
    except Exception:
        return None


def merge(own: WallEvidence, snapshots: Iterable[bytes]) -> np.ndarray:
    total = own.counts.astype(np.float32)
    for blob in snapshots:
        grid = decode_snapshot(blob, total.shape)
        if grid is not None:
            total += grid
    return total


def _border_bands(evidence: np.ndarray, out: np.ndarray, min_hits: float = 3.0,
                  depth: int = 16) -> None:
    """Draw observed border segments from the map edge to their inner face."""
    h, w = evidence.shape
    sides = (
        ("top", evidence[:depth, :], lambda s, e, t: out.__setitem__((slice(0, t), slice(s, e)), 1)),
        ("bottom", evidence[h - depth:, :][::-1, :],
         lambda s, e, t: out.__setitem__((slice(h - t, h), slice(s, e)), 1)),
        ("left", evidence[:, :depth].T, lambda s, e, t: out.__setitem__((slice(s, e), slice(0, t)), 1)),
        ("right", evidence[:, w - depth:][:, ::-1].T,
         lambda s, e, t: out.__setitem__((slice(s, e), slice(w - t, w)), 1)),
    )
    for _, strip, draw in sides:
        length = strip.shape[1]
        profile = strip.sum(axis=1)
        if profile.sum() < min_hits:
            continue
        # Inner face: the deepest row that still holds a solid share of hits.
        cumulative = np.cumsum(profile) / profile.sum()
        face = int(np.searchsorted(cumulative, 0.9)) + 1
        thickness = int(min(max(face, 4), depth - 2))
        for start in range(0, length, SEGMENT):
            end = min(start + SEGMENT, length)
            if strip[:, start:end].sum() >= min_hits:
                # Draw only between the first and last observed position of
                # the segment (plus a margin), so that a gap in the border is
                # never filled more than SPAN_MARGIN px beyond the evidence.
                seen = np.flatnonzero(strip[:, start:end].sum(axis=0) > 0)
                lo = max(start, start + int(seen[0]) - SPAN_MARGIN)
                hi = min(end, start + int(seen[-1]) + 1 + SPAN_MARGIN)
                draw(lo, hi, thickness)


def render(evidence: np.ndarray, *, dc_box: Optional[Tuple[float, float, float, float]] = None,
           sigma: float = 2.2, abs_threshold: float = 0.55, rel_threshold: float = 0.33,
           border_depth: int = 16, border: bool = True) -> np.ndarray:
    """Binary wall map (uint8, H x W) from summed evidence counts.

    ``border=False`` disables the border-band rendering (mapping ablation);
    the agents always use the default.
    """
    h, w = evidence.shape
    smooth = cv2.GaussianBlur(evidence.astype(np.float32), (0, 0), sigma)
    peak = cv2.dilate(smooth, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)))
    walls = (smooth >= abs_threshold) & (smooth >= rel_threshold * peak)
    out = walls.astype(np.uint8)
    # Observed border segments are added at their exact position; interior
    # walls that abut the border keep their own evidence band.
    if border:
        _border_bands(evidence, out, depth=border_depth)
    # Remove small isolated specks (isolated false returns).
    n, labels, stats, _ = cv2.connectedComponentsWithStats(out, connectivity=8)
    small = stats[:, cv2.CC_STAT_AREA] < 12
    small[0] = False
    out[small[labels]] = 0
    if dc_box is not None:
        # The center is not a wall, but where it touches the border the
        # border itself still is: keep the border band.
        x0, y0, x1, y1 = dc_box
        c0 = max(int(math.floor(x0 + w / 2.0)) - 3, border_depth)
        c1 = min(int(math.ceil(x1 + w / 2.0)) + 3, w - border_depth)
        r0 = max(int(math.floor(h / 2.0 - y1)) - 3, border_depth)
        r1 = min(int(math.ceil(h / 2.0 - y0)) + 3, h - border_depth)
        if c1 > c0 and r1 > r0:
            out[r0:r1, c0:c1] = 0
    return out


def wall_weights(wall_map: np.ndarray, scale: int, border: int = 10) -> np.ndarray:
    """Down-sampled wall weights for coverage planning (border walls 0.5)."""
    h, w = wall_map.shape
    weights = wall_map.astype(np.float32)
    weights[:border, :] *= 0.5
    weights[h - border:, :] *= 0.5
    weights[:, :border] *= 0.5
    weights[:, w - border:] *= 0.5
    small = cv2.resize(weights, (w // scale, h // scale), interpolation=cv2.INTER_AREA)
    return small * (scale * scale)
