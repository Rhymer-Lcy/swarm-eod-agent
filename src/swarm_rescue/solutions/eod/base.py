"""Navigation core shared by the blue and red brains.

Holds the pose estimator, the coarse occupancy grid and planner, the layer
of cells already searched with the semantic sensor, what is known about
teammates, and the behaviours both roles need: lidar integration with
entity masking, frontier/search-target selection with team spreading, path
following on geodesic fields, and stuck recovery.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from . import params as P
from .comms import TeamTable
from .estimator import PoseEstimator
from .grid import OccupancyGrid, Planner
from .motion import LocalView, StuckMonitor, drive, lookahead_point
from .observation import Command, Observation
from .perception import DisposalCenterModel, entity_mask, lidar_endpoints, near_points_mask

REPLAN_PERIOD = 25
FLEET_MAX_AGE = 200


class BrainBase:
    SEARCH_RANGE = 160.0
    FREE_RANGE = P.FREE_RANGE

    def __init__(self, identifier: int, n_drones: int, size_area: Tuple[int, int],
                 max_steps: Optional[int]) -> None:
        self.id = int(identifier)
        self.n = max(1, int(n_drones))
        self.size = (int(size_area[0]), int(size_area[1]))
        self.max_steps = max_steps or 5000
        self.est = PoseEstimator()
        self.grid = OccupancyGrid(self.size)
        self.planner = Planner(self.grid)
        self.searched = np.zeros((self.grid.ny, self.grid.nx), dtype=bool)
        self.dc = DisposalCenterModel()
        self.team = TeamTable()
        self.fleet: Dict[int, Tuple[float, float, int, bool]] = {}
        self.home: Optional[Tuple[float, float]] = None
        self.return_area: Optional[Tuple[float, float]] = None
        self.state = "init"
        self.target: Optional[Tuple[float, float]] = None
        self.path: List[Tuple[float, float]] = []
        self.path_goal: Optional[Tuple[float, float]] = None
        self.path_step = -10_000
        self.goal_step = -10_000
        self.recover_until = -1
        self.recover_dir = 0.0
        self.stuck = StuckMonitor()
        self.msg_out = None
        self.diag: Dict[str, int] = {"stuck": 0, "replans": 0}
        self._me_field: Optional[np.ndarray] = None
        self._me_field_step = -10_000
        self._me_field_cell: Tuple[int, int] = (-100, -100)

    # -- framework-facing -------------------------------------------------
    def outgoing_message(self):
        return self.msg_out

    def _begin(self, obs: Observation) -> bool:
        """Estimator update; False until the pose is initialized."""
        self.est.update(obs.gps, obs.compass, obs.odometer, grasping=obs.grasping)
        if not self.est.initialized:
            return False
        if self.home is None:
            self.home = (self.est.x, self.est.y)
        if obs.in_return_area:
            self.return_area = (self.est.x, self.est.y)
        return True

    def _finish(self, obs: Observation, cmd: Command) -> Command:
        self.est.set_command(cmd.forward, cmd.lateral, cmd.rotation, obs.grasping)
        return cmd

    # -- fleet gossip -------------------------------------------------------
    def _read_fleet(self, msg: dict) -> None:
        sender = msg["id"]
        self.fleet[sender] = (float(msg["p"][0]), float(msg["p"][1]), int(msg["t"]),
                              bool(msg.get("bz", False)))
        for item in msg.get("tp", ()):
            if isinstance(item, tuple) and len(item) == 5 and item[0] != self.id:
                known = self.fleet.get(int(item[0]))
                if known is None or known[2] < int(item[3]):
                    self.fleet[int(item[0])] = (float(item[1]), float(item[2]),
                                                int(item[3]), bool(item[4]))

    def _fleet_digest(self, now: int, busy: bool) -> tuple:
        fleet = [(self.id, round(self.est.x, 1), round(self.est.y, 1), now, busy)]
        fleet += [(k, round(v[0], 1), round(v[1], 1), v[2], v[3])
                  for k, v in self.fleet.items() if now - v[2] < FLEET_MAX_AGE]
        return tuple(fleet)

    # -- perception -------------------------------------------------------------
    def _lidar_mask(self, obs: Observation, extra_points=()
                    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Lidar end points and two masks of returns that are not walls.

        ``transient``: returns on drones and bombs, which must neither be
        mapped nor block planning. ``center``: returns on the disposal
        center, a solid obstacle for navigation that is not a wall.
        """
        est = self.est
        lidar = obs.lidar
        ex, ey = lidar_endpoints(est.x, est.y, est.heading, lidar, obs.lidar_angles)
        transient = entity_mask(obs.semantic, lidar, obs.lidar_angles,
                                kinds=("DRONE", "BOMB", "OTHER"))
        transient |= near_points_mask(ex, ey, self.team.positions(obs.step, 4), 24.0)
        if extra_points:
            transient |= near_points_mask(ex, ey, list(extra_points), 20.0)
        center = entity_mask(obs.semantic, lidar, obs.lidar_angles, kinds=("DISPOSAL_CENTER",))
        center |= self.dc.inside(ex, ey, 8.0)
        center &= ~transient
        return ex, ey, transient, center

    def _integrate(self, obs: Observation, transient: np.ndarray) -> None:
        """Navigation grid update: walls and the disposal center are obstacles."""
        est = self.est
        self.grid.integrate_scan(est.x, est.y, est.heading, obs.lidar, obs.lidar_angles,
                                 ~transient, max_free=self.FREE_RANGE)
        self._mark_searched(obs.lidar, obs.lidar_angles)

    def _mark_searched(self, lidar: np.ndarray, angles: np.ndarray) -> None:
        est = self.est
        idx = np.arange(0, len(lidar), 3)
        reach = np.minimum(lidar[idx], self.SEARCH_RANGE)
        t = np.arange(0.0, self.SEARCH_RANGE + 1.0, self.grid.cell)[None, :]
        mask = t <= reach[:, None]
        a = angles[idx] + est.heading
        px = (est.x + t * np.cos(a)[:, None])[mask]
        py = (est.y + t * np.sin(a)[:, None])[mask]
        cells = self.grid._cells(px, py)
        self.searched.reshape(-1)[cells[cells >= 0]] = True

    def _others(self, obs: Observation) -> List[Tuple[float, float]]:
        pts = self.team.positions(obs.step, 3)
        if obs.semantic:
            for hit in obs.semantic:
                if hit.kind == "DRONE":
                    d = hit.distance - P.SEMANTIC_BIAS + 10.0
                    a = self.est.heading + hit.angle
                    pts.append((self.est.x + d * math.cos(a), self.est.y + d * math.sin(a)))
        return pts

    # -- geodesic field from the drone (claims, exploration and paths) ---------
    def _field_from_me(self, now: int) -> np.ndarray:
        cell = self.grid.to_cell(self.est.x, self.est.y)
        moved = max(abs(cell[0] - self._me_field_cell[0]),
                    abs(cell[1] - self._me_field_cell[1]))
        if self._me_field is None or now - self._me_field_step > REPLAN_PERIOD or moved > 3:
            self._me_field = self.planner.field([cell], allow_unknown=True, step=now)
            self._me_field_step = now
            self._me_field_cell = cell
            self.diag["replans"] += 1
        return self._me_field

    # -- exploration target --------------------------------------------------
    def _choose_explore_target(self, now: int, disperse_until: int = 250
                               ) -> Optional[Tuple[float, float]]:
        g = self.grid
        candidates = (g.free() & ~self.searched) | g.frontier()
        if not candidates.any():
            return None
        field = self._field_from_me(now)
        reach = np.isfinite(field) & candidates
        if not reach.any():
            return None
        density = cv2.blur((~self.searched).astype(np.float32), (7, 7))
        rows, cols = np.nonzero(reach)
        score = field[rows, cols] * g.cell - 120.0 * density[rows, cols]
        wx = (cols + 0.5) * g.cell - g.width / 2.0
        wy = g.height / 2.0 - (rows + 0.5) * g.cell
        for _, tgt in self.team.targets(now):
            d = np.hypot(wx - tgt[0], wy - tgt[1])
            score += np.where(d < 220.0, (220.0 - d) * 2.0, 0.0)
        for k, (ox, oy, t, _) in self.fleet.items():
            if now - t < 30:
                d = np.hypot(wx - ox, wy - oy)
                score += np.where(d < 150.0, 150.0 - d, 0.0)
        if now < disperse_until and self.home is not None:
            sector = 2.0 * math.pi * self.id / self.n
            ang = np.arctan2(wy - self.home[1], wx - self.home[0])
            score += 80.0 * np.abs((ang - sector + math.pi) % (2 * math.pi) - math.pi)
        i = int(np.argmin(score))
        return float(wx[i]), float(wy[i])

    # -- path following -------------------------------------------------------
    def _path_target(self, obs: Observation, goal: Tuple[float, float]) -> Tuple[float, float]:
        est = self.est
        now = obs.step
        if not self.path or now - self.path_step > REPLAN_PERIOD or self.path_goal is None or \
                math.hypot(self.path_goal[0] - goal[0], self.path_goal[1] - goal[1]) > 20.0:
            field = self._field_from_me(now)
            # Descending the field from the goal leads back to the drone.
            cells = self.planner.descend(field, self.grid.to_cell(goal[0], goal[1]))
            cells.reverse()
            self.path = [self.grid.to_world(r, c) for r, c in cells] + [goal]
            self.path_goal = goal
            self.path_step = now
        return lookahead_point(self.path, est.x, est.y, P.WAYPOINT_LOOKAHEAD,
                               lambda px, py: self.grid.clear_along(est.x, est.y, px, py))

    def _goto(self, obs: Observation, view: LocalView, others, goal: Tuple[float, float],
              cruise: float, heading_gain: float = 0.5, stop: bool = False) -> Command:
        est = self.est
        target = self._path_target(obs, goal)
        cmd = Command()
        from .motion import rotation_toward
        cmd.rotation = rotation_toward(est.heading,
                                       math.atan2(target[1] - est.y, target[0] - est.x),
                                       gain=heading_gain)
        stop_here = stop and math.hypot(goal[0] - est.x, goal[1] - est.y) < 70.0
        aim = goal if stop_here else target
        f, l, _ = drive(est, aim, view, cruise=cruise, carrying=obs.grasping,
                        others=others, stop_at_target=stop_here)
        cmd.forward, cmd.lateral = f, l
        return cmd

    # -- recovery ---------------------------------------------------------------
    def _check_stuck(self, obs: Observation, view: LocalView, cmd: Command,
                     block_target: bool) -> None:
        moving = math.hypot(cmd.forward, cmd.lateral) > 0.3
        if self.stuck.update(self.est.x, self.est.y, 3.0 if moving else 0.0):
            self.diag["stuck"] += 1
            self.recover_until = obs.step + 20
            if view.ok and view.dist.size:
                i = int(np.argmin(view.dist))
                self.recover_dir = math.atan2(-view.py[i], -view.px[i]) + \
                    (0.6 if self.id % 2 else -0.6)
            else:
                self.recover_dir = self.est.heading + math.pi
            if block_target and self.target is not None:
                self.planner.extra_blocked.append((self.target[0], self.target[1], 16.0,
                                                   obs.step + 200))
            self.path = []
            self.stuck.reset()

    def _recover_command(self, obs: Observation, view: LocalView, others) -> Command:
        est = self.est
        aim = (est.x + 60.0 * math.cos(self.recover_dir), est.y + 60.0 * math.sin(self.recover_dir))
        f, l, _ = drive(est, aim, view, cruise=3.0, carrying=obs.grasping, others=others,
                        stop_at_target=False)
        return Command(forward=f, lateral=l, grasper=1 if obs.grasping else 0)
