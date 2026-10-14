"""B3 (Phase 4): carriers search for a disposal center they do not know.

The defect (P4CorridorV1, real simulator round V-C2 and offline P4-A014): the
frozen B2 sent a carrier that does not know the center straight to its start
area, where it waited; when every drone had picked up a bomb before anyone
saw a hidden center, the team never found it (0 of 10 disposed). B3 changes
only that branch. These tests run the frozen B2 (``p3_067818a``) on the same
inputs: it must fail where B3 succeeds, and behave identically wherever the
center is known.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from devtools.comparators.p3_067818a import blue as b2_blue  # noqa: E402
from devtools.simlab import delivery_fixtures as DF  # noqa: E402
from swarm_rescue.solutions.eod import blue as b3_blue  # noqa: E402
from swarm_rescue.solutions.eod import params as P  # noqa: E402


def _hidden_run(module, n=2, seed=1, steps=3000):
    """``n`` drones start in the far corner of ``hidden``, each already holding
    a bomb (as when every drone grasped one before anyone saw the center);
    the center was never observed. Returns (disposed, brains, world)."""
    from devtools.simlab.offline_sim import OfflineWorld
    fx = DF.fixture("hidden", steps + 50)
    brains = []

    def factory(i):
        brains.append(module.RescueBrain(i, n, (DF.W, DF.H), steps + 50))
        return brains[-1]

    bombs = [(fx["start_area"][0] + 30.0 * i, fx["start_area"][1] - 60.0) for i in range(n)]
    world = OfflineWorld(fx, "rescue", n, factory, seed=seed, bombs=bombs,
                         max_steps=steps + 50, carry_ratio_sigma=0.36)
    world.step()
    for i, d in enumerate(world.drones):
        b = world.bombs[i]
        b.x = d.x + P.CARRY_OFFSET * math.cos(d.heading)
        b.y = d.y + P.CARRY_OFFSET * math.sin(d.heading)
        b.holder, d.carrying, d.grasp_angle = i, i, 0.0
    assert not any(br.dc.known for br in brains)
    while world.step() and world.disposed < n and world.step_index < steps:
        pass
    return world.disposed, brains, world


@pytest.mark.parametrize("seed", [1, 2])
def test_frozen_b2_never_finds_a_hidden_center_while_every_drone_carries(seed):
    disposed, brains, world = _hidden_run(b2_blue, seed=seed)
    assert disposed == 0
    assert not any(b.dc.known for b in brains)
    assert all(d.carrying is not None for d in world.drones)


@pytest.mark.parametrize("seed", [1, 2, 3, 4])
def test_b3_carriers_find_the_hidden_center(seed):
    """Every B3 carrier finds the center and at least one delivers in each
    seed (attempt P4-A015: 2, 1, 1, 1 of 2 within 3000 steps; B2: 0). The
    remaining carriers know the center but have no route to it: in their own
    lidar maps this fixture's 70 px corridor is closed by the planner's
    obstacle inflation (P4-A016: 8-14 of 14 centre-line cells untraversable,
    none occupied), because walls mapped under pose error come out thicker
    than drawn; with ideal walls it would be open (next test). B3 does not
    address this."""
    disposed, brains, _ = _hidden_run(b3_blue, seed=seed)
    assert disposed >= 1, [b.diag for b in brains]
    assert all(b.dc.known for b in brains)
    assert all(b.diag["center_searches"] > 0 for b in brains)


@pytest.mark.parametrize("width,open_", [(54, False), (70, True), (86, True)])
def test_planner_inflation_on_an_ideally_mapped_corridor(width, open_):
    """Characterization (shared by B1, B2 and B3): with 24 px inflation on
    8 px cells and ideal 6 px walls, a straight corridor is untraversable for
    the planner below about 56 px between wall faces, depends on the cell
    alignment up to 62 px and is open from 64 px (P4-A016 probe over eight
    alignments). A drone is 30 px wide, so 30-56 px passages that it could
    fly through are never planned through; noisier maps close wider ones."""
    from swarm_rescue.solutions.eod.grid import OccupancyGrid, Planner
    g = OccupancyGrid((400, 400))
    g.logodds[:, :] = P.LOGODDS_MIN
    g.observed[:, :] = True
    for x0 in (-width / 2 - 6.0, width / 2):          # 6 px walls, faces width apart
        for y in np.arange(-150.0, 150.0, 1.0):
            for dx in np.arange(0.0, 6.0, 1.0):
                r, c = g.to_cell(x0 + dx, y)
                g.logodds[r, c] = P.LOGODDS_MAX
    g.version += 1
    cost = Planner(g).cost(True, 0)
    row = g.to_cell(0.0, 0.0)[0]
    across = [cost[row, g.to_cell(x, 0.0)[1]] > 0 for x in np.arange(-width / 2, width / 2, 4.0)]
    assert any(across) is open_, (width, across)


def _teleport_known_center(module, seed):
    """The Phase 3 closed-loop carrier (center seen from two sides first)."""
    from devtools.tests import test_blue_delivery as TB
    return TB._teleport_carrier(module, (18.0, -75.0), seed=seed)


@pytest.mark.parametrize("seed", [1, 2])
def test_b3_is_command_identical_to_b2_when_the_center_is_known(seed):
    w2, brain2, cmds2, took2 = _teleport_known_center(b2_blue, seed)
    w3, brain3, cmds3, took3 = _teleport_known_center(b3_blue, seed)
    assert brain3.diag["center_searches"] == 0
    assert (w2.disposed, took2) == (w3.disposed, took3)
    assert cmds2 == cmds3
    d2, d3 = w2.drones[0], w3.drones[0]
    assert (d2.x, d2.y, d2.heading) == (d3.x, d3.y, d3.heading)   # same trajectory end, bit for bit
    diag3 = {k: v for k, v in brain3.diag.items() if k != "center_searches"}
    assert brain2.diag == diag3


def test_seeking_carrier_follows_a_planned_path_not_a_straight_line():
    """With the center unknown and the start area behind a wall, B2 aims at
    the start area through the wall; B3's target is a path point the map
    allows."""
    for module, through_wall in ((b2_blue, True), (b3_blue, False)):
        brain = module.RescueBrain(0, 1, (DF.W, DF.H), 3000)
        g = brain.grid
        g.logodds[:, :] = P.LOGODDS_MIN
        g.observed[:, :] = True
        r, c = g.to_cell(0.0, 0.0)
        g.logodds[r - 1:r + 2, c - 20:c + 21] = P.LOGODDS_MAX   # a 330 px wall at y = 0
        g.version += 1
        est = brain.est
        est.x, est.y, est.heading = 0.0, -60.0, math.pi / 2
        est.initialized = True
        est.heading_var = math.radians(2.0) ** 2
        brain.home = (0.0, 120.0)                                 # behind the wall
        brain.searched[:, :] = True                               # nothing left to search
        from swarm_rescue.solutions.eod.motion import LocalView
        from swarm_rescue.solutions.eod.observation import Observation
        obs = Observation(step=5, walltime=0.0, identifier=0, n_drones=1,
                          size_area=(DF.W, DF.H), max_steps=3000, max_walltime=600.0,
                          gps=None, compass=None, odometer=None,
                          lidar=np.full(P.LIDAR_RAYS, 290.0),
                          lidar_angles=np.linspace(-math.pi, math.pi, P.LIDAR_RAYS),
                          semantic=[], grasping=True, health=50, inventory=0)
        brain.carried = None
        brain._carry(obs, LocalView(est.heading, None, obs.lidar_angles), [])
        blocked = not g.clear_along(est.x, est.y, *brain.target)
        assert blocked is through_wall, (module.__name__, brain.target)
