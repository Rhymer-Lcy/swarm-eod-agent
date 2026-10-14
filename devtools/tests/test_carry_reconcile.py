"""Carry-episode accounting reconciled with the official disposal count (Phase 4)."""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.simlab.carry_episodes import reconcile  # noqa: E402


def _ep(outcome, end, dist=30.0, gain=None):
    g = gain if gain is not None else (1.0 if outcome == "disposed" else 0.0)
    return {"outcome": outcome, "end": end, "end_distance_to_center": dist, "reward_gain": g}


def test_last_carry_cut_off_by_termination_is_a_disposal():
    eps = [_ep("disposed", 100 * i) for i in range(1, 10)] + [_ep("held_at_end", 1000, 5.0)]
    r = reconcile(eps, official_disposed=10, total=10, last_step=1000)
    assert r["consistent"] and r["disposed_logged"] == 9
    assert r["disposed_at_termination"] == 1 and r["held_unresolved"] == 0


def test_genuine_unresolved_carries_stay_failures():
    eps = [_ep("disposed", 100)] + [_ep("held_at_end", 5000, 80.0) for _ in range(3)]
    r = reconcile(eps, official_disposed=1, total=4, last_step=5000)
    assert r["consistent"] and r["disposed_at_termination"] == 0 and r["held_unresolved"] == 3


def test_shared_disposal_counts_once():
    """Two drones holding one bomb when it is disposed split the reward
    (0.5 each); measured in round S2-2 (B3, P4PocketV2)."""
    eps = [_ep("disposed", 100 * i) for i in range(1, 9)] + \
        [_ep("disposed", 3420, gain=0.5), _ep("disposed", 3420, gain=0.5),
         _ep("held_at_end", 5001, 76.9)]
    r = reconcile(eps, official_disposed=9, total=10, last_step=5001)
    assert r["consistent"] and r["disposed_logged"] == 9
    assert r["shared_disposal_episodes"] == 2 and r["held_unresolved"] == 1


def test_unexplainable_difference_is_flagged_not_forced():
    eps = [_ep("disposed", 100), _ep("disposed", 200)]
    r = reconcile(eps, official_disposed=1, total=2, last_step=300)
    assert r["consistent"] is False and r["disposed_at_termination"] == 0
