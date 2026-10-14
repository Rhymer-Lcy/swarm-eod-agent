"""Team coverage summary shared over the radio.

Each drone keeps a coarse bitmap of the arena in square blocks
(``BLOCK`` px): a block is covered once a drone has searched at least half
of its non-occupied grid cells (``BrainBase.searched``). Drones broadcast the bitwise OR
of everything they know (their own coverage and what they heard), packed one
bit per block, every ``PERIOD`` steps. The largest supplied map (1660 x 1122
px) needs 52 x 36 blocks, i.e. 234 bytes.

Merging is a bitwise OR, so the summary behaves like a grow-only set: a
delayed, duplicated or out-of-order message can only add blocks that some
drone did cover; a stale message lacks blocks but never removes any; a
disconnected drone keeps working with its own coverage and catches up on
reconnection; a failed drone's coverage persists in the others. Coverage is
never retracted, which is correct because searched cells stay searched (the
bombs of a round do not move unless a drone carries them).

Messages are validated: the payload must be ``(blocks_x, blocks_y, bytes)``
with the receiver's own block dimensions and the exact byte length.
"""

from __future__ import annotations

import math
from typing import Any, Optional, Tuple

import numpy as np

BLOCK = 32.0
PERIOD = 10
MIN_FRACTION = 0.5


class TeamCoverage:
    def __init__(self, size_area: Tuple[int, int], cell: float, ny: int, nx: int) -> None:
        self.k = max(1, int(round(BLOCK / cell)))
        self.ny, self.nx = int(ny), int(nx)
        self.by = int(math.ceil(self.ny / self.k))
        self.bx = int(math.ceil(self.nx / self.k))
        self.blocks = np.zeros((self.by, self.bx), dtype=bool)
        self.n_bytes = (self.by * self.bx + 7) // 8
        self._mask: Optional[np.ndarray] = None
        self.merged_messages = 0
        self.rejected_messages = 0

    # ------------------------------------------------------------------
    def update_own(self, searched: np.ndarray, occupied: np.ndarray) -> None:
        """Add the blocks this drone has searched itself.

        The share is taken over the block's cells that exist and are not
        occupied (wall cells are never searched, edge blocks are partial).
        """
        shape = (self.by * self.k, self.bx * self.k)
        hit = np.zeros(shape, dtype=np.float32)
        room = np.zeros(shape, dtype=np.float32)
        hit[:self.ny, :self.nx] = searched & ~occupied
        room[:self.ny, :self.nx] = ~occupied
        hits = hit.reshape(self.by, self.k, self.bx, self.k).sum(axis=(1, 3))
        rooms = room.reshape(self.by, self.k, self.bx, self.k).sum(axis=(1, 3))
        new = (rooms > 0) & (hits >= MIN_FRACTION * rooms)
        if np.any(new & ~self.blocks):
            self.blocks |= new
            self._mask = None

    def payload(self) -> Tuple[int, int, bytes]:
        return self.bx, self.by, np.packbits(self.blocks.reshape(-1)).tobytes()

    def merge(self, item: Any) -> bool:
        """OR a teammate's summary in; False (and nothing changes) if invalid."""
        if not (isinstance(item, tuple) and len(item) == 3 and item[0] == self.bx
                and item[1] == self.by and isinstance(item[2], bytes)
                and len(item[2]) == self.n_bytes):
            self.rejected_messages += 1
            return False
        bits = np.unpackbits(np.frombuffer(item[2], dtype=np.uint8))[:self.by * self.bx]
        other = bits.reshape(self.by, self.bx).astype(bool)
        if np.any(other & ~self.blocks):
            self.blocks |= other
            self._mask = None
        self.merged_messages += 1
        return True

    def cell_mask(self) -> np.ndarray:
        """Covered blocks expanded to grid cells (read-only, cached)."""
        if self._mask is None:
            mask = np.repeat(np.repeat(self.blocks, self.k, axis=0), self.k, axis=1)
            mask = np.ascontiguousarray(mask[:self.ny, :self.nx])
            mask.flags.writeable = False
            self._mask = mask
        return self._mask

    @property
    def fraction(self) -> float:
        return float(self.blocks.mean()) if self.blocks.size else 0.0
