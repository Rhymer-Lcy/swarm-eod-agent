"""Shared exploration: coverage summaries and target commitment (Phase 2)."""

from __future__ import annotations

import itertools
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from swarm_rescue.solutions.eod import params as P  # noqa: E402
from swarm_rescue.solutions.eod.comms import make_message  # noqa: E402
from swarm_rescue.solutions.eod.coverage import TeamCoverage  # noqa: E402
from swarm_rescue.solutions.eod.grid import OccupancyGrid  # noqa: E402

SIZE = (400, 300)


def _cov(size=SIZE):
    g = OccupancyGrid(size)
    return TeamCoverage(size, g.cell, g.ny, g.nx), g


def _searched(g, x0, x1, y0, y1):
    s = np.zeros((g.ny, g.nx), dtype=bool)
    r0, c0 = g.to_cell(x0, y1)
    r1, c1 = g.to_cell(x1, y0)
    s[r0:r1 + 1, c0:c1 + 1] = True
    return s


def test_payload_is_compact_on_every_supplied_map_size():
    for size in ((1250, 800), (1200, 900), (1113, 750), (1660, 1122), (1200, 1200)):
        cov, _ = _cov(size)
        bx, by, blob = cov.payload()
        assert len(blob) == cov.n_bytes == (bx * by + 7) // 8
        assert len(blob) <= 240


def test_own_coverage_uses_free_cells_only():
    cov, g = _cov()
    occupied = np.zeros((g.ny, g.nx), dtype=bool)
    occupied[:, :2] = True                              # a wall strip
    s = _searched(g, -200, -136, -150, 150)             # left 64 px
    cov.update_own(s, occupied)
    assert cov.blocks[:, :2].all() and not cov.blocks[:, 2:].any()
    assert cov.cell_mask().shape == (g.ny, g.nx)
    assert not cov.cell_mask().flags.writeable


def test_merge_is_order_independent_idempotent_and_duplicate_safe():
    parts = []
    expected = None
    for x0 in (-200, -100, 0, 100):
        cov, g = _cov()
        cov.update_own(_searched(g, x0, x0 + 64, -150, 0), np.zeros((g.ny, g.nx), bool))
        parts.append(cov.payload())
        expected = cov.blocks.copy() if expected is None else expected | cov.blocks
    results = []
    for order in itertools.permutations(parts):
        cov, _ = _cov()
        for item in order + order:                      # every message twice
            assert cov.merge(item)
        results.append(cov.blocks.copy())
    assert all(np.array_equal(results[0], r) for r in results)
    assert np.array_equal(results[0], expected)         # the union of the senders' blocks
    assert len(results) == 24 and expected.sum() >= 4


def test_stale_and_delayed_messages_never_remove_coverage():
    cov, g = _cov()
    other, _ = _cov()
    other.update_own(_searched(g, -200, -100, -150, 150), np.zeros((g.ny, g.nx), bool))
    early = other.payload()
    other.update_own(_searched(g, -200, 200, -150, 150), np.zeros((g.ny, g.nx), bool))
    late = other.payload()
    cov.merge(late)
    full = cov.blocks.sum()
    cov.merge(early)                                    # delayed, older summary
    assert cov.blocks.sum() == full


def test_invalid_coverage_messages_are_rejected():
    cov, _ = _cov()
    bx, by, blob = cov.payload()
    for bad in [None, (bx, by), (bx + 1, by, blob), (bx, by, blob[:-1]), (bx, by, "x" * len(blob)),
                (bx, by, list(blob)), [bx, by, blob]]:
        assert not cov.merge(bad)
    assert cov.rejected_messages == 7 and not cov.blocks.any()


def _red_brain(ident, n=3):
    from swarm_rescue.solutions.eod.red import PlaceBrain
    return PlaceBrain(ident, n, SIZE, 2000, 90.0)


def test_disconnected_drone_works_alone_and_catches_up_on_reconnection():
    a, b = _red_brain(0), _red_brain(1)
    b.searched[:, :] = False
    b.searched[:, : b.grid.nx // 2] = True
    b._coverage_tick(9)                                 # (9 + 1) % 10 == 0
    assert b.coverage.blocks.any()
    assert not a.coverage.blocks.any()                  # no contact yet
    msg = make_message(1, 9, (0.0, 0.0), cv=b._coverage_message(9))
    a._read_coverage(msg)                               # reconnection
    assert np.array_equal(a.coverage.blocks, b.coverage.blocks)
    # Drone 1 fails: its coverage stays with drone 0.
    del b
    a._read_coverage(make_message(2, 30, (0.0, 0.0), cv=None))
    assert a.coverage.blocks.any()


def test_exploration_skips_blocks_covered_by_teammates():
    brain = _red_brain(0)
    g = brain.grid
    g.logodds[:, :] = P.LOGODDS_MIN
    g.version += 1
    brain.home = (0.0, 0.0)
    brain.est.x = brain.est.y = 0.0
    brain.est.initialized = True
    other, _ = _cov()
    other.update_own(_searched(g, -200, 0, -150, 150), np.zeros((g.ny, g.nx), bool))
    brain.coverage.merge(other.payload())
    for now in (300, 400):
        tx, _ = brain._choose_explore_target(now)
        assert tx > 0.0                                 # right half only
    # Fallback: everything covered -> own knowledge decides again.
    other.update_own(_searched(g, -200, 200, -150, 150), np.zeros((g.ny, g.nx), bool))
    brain.coverage.merge(other.payload())
    assert brain._choose_explore_target(500) is not None


def test_target_commitment_expires_yields_and_drops_covered_targets():
    brain = _red_brain(2)
    g = brain.grid
    g.logodds[:, :] = P.LOGODDS_MIN
    g.version += 1
    brain.est.x = brain.est.y = 0.0
    brain.state = "explore"
    brain.target = (150.0, 100.0)
    brain.goal_step = 100
    brain.target_budget = 80
    assert not brain._explore_due(105, 40)              # committed beyond the old 40-step period
    assert not brain._explore_due(150, 40)
    assert brain._explore_due(181, 40)                  # budget expired
    assert brain.diag["target_expired"] == 1
    # A lower identifier with a target nearby: yield.
    brain.goal_step = 200
    brain.team.update(make_message(1, 199, (100.0, 0.0), tg=(160.0, 110.0)), now=200)
    assert brain._explore_due(201, 40) and brain.diag["target_yields"] == 1
    # A higher identifier does not make us yield.
    brain.team.members.clear()
    brain.team.update(make_message(5, 199, (100.0, 0.0), tg=(160.0, 110.0)), now=200)
    assert not brain._explore_due(201, 40)
    # The target block becomes covered by the team: re-choose at the next check.
    brain.team.members.clear()
    other, _ = _cov()
    other.update_own(_searched(g, 100, 200, 50, 150), np.zeros((g.ny, g.nx), bool))
    brain.coverage.merge(other.payload())
    assert brain._explore_due(210, 40) and brain.diag["target_covered"] == 1


def test_blue_keeps_phase1_exploration_rules():
    from swarm_rescue.solutions.eod.blue import RescueBrain
    brain = RescueBrain(0, 3, SIZE, 2000)
    assert not brain.SHARE_COVERAGE and not brain.COMMIT_TARGETS
    brain.est.x = brain.est.y = 0.0
    brain.state = "explore"
    brain.target = (150.0, 100.0)
    brain.goal_step = 100
    assert not brain._explore_due(140, 40) and brain._explore_due(141, 40)
    assert brain._coverage_message(9) is None


def test_red_brains_share_coverage_offline():
    from devtools.simlab.offline_sim import OfflineWorld
    from devtools.tests.test_eod_agent import _arena
    from swarm_rescue.solutions.eod.red import PlaceBrain
    fx = _arena()
    brains = []

    def factory(i):
        brains.append(PlaceBrain(i, 3, (600, 400), 900, 60.0))
        return brains[-1]

    world = OfflineWorld(fx, "place", 3, factory, seed=3, bombs_per_drone=1, max_steps=900)
    world.run()
    assert world.submitted is not None
    assert all(b.coverage.merged_messages > 0 for b in brains)
    assert all(b.coverage.rejected_messages == 0 for b in brains)
    own_only = [b.searched.mean() for b in brains]
    team = [b.coverage.fraction for b in brains]
    assert min(team) >= 0.8 * max(team)                 # consistent team picture
    assert max(team) > 0.0 and math.isfinite(sum(own_only))
