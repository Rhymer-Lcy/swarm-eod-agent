"""Blue (rescue) brain, candidate B0.

Every drone runs the same decentralised policy:

* explore: go to the cheapest reachable cell that has not yet been searched
  with the semantic sensor, spreading away from teammates' targets (an
  identifier-based sector bias disperses the team at the start);
* share: bomb sightings, fleet positions, claims and deliveries are gossiped
  in every message, so information relays over several hops; the newest
  observation of a bomb wins;
* claim: a drone takes a free bomb only when no unoccupied teammate is
  clearly closer and no explicit claim with a lower cost exists (ties: lower
  identifier); claims are refreshed every step and lapse when not renewed;
* fetch and grasp: travel along the geodesic path; once the bomb is in view,
  steer on the live relative detection (immune to the slowly varying GPS
  error), face it and hold the grasper on until it attaches;
* carry and deliver: follow the geodesic field to the disposal center and
  drive the side on which the bomb was grasped into the nearest observed
  surface point of the center; a detachment next to the center counts as a
  delivery, anywhere else as a drop that is re-queued.

Missing sensors, missing messages or an unknown disposal center degrade the
behaviour (dead reckoning, local decisions, return towards the start area)
instead of stopping it.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from . import params as P
from .base import BrainBase
from .comms import make_message, valid_messages
from .geometry import wrap
from .motion import LocalView, drive, lookahead_point, rotation_toward
from .observation import Command, Observation
from .perception import BombDetection, bomb_detections, entity_mask

BOMB_MERGE = 35.0
APPROACH_RANGE = 140.0
GRASP_TIMEOUT = 140
CLAIM_TIMEOUT = 40
CLAIM_PERIOD = 8
EXPLORE_PERIOD = 40
DELIVER_RANGE = 100.0
DELIVER_TIMEOUT = 160
FLEET_MAX_AGE = 200


class Bomb:
    __slots__ = ("x", "y", "status", "seen", "fail_until")

    def __init__(self, x: float, y: float, status: int, seen: int) -> None:
        self.x, self.y = x, y
        self.status = status          # 0 free, 1 carried, 2 gone
        self.seen = seen              # step of the newest information
        self.fail_until = -1          # own grasp failures: skip until this step


class BombTable:
    """Known bombs; the newest information about each one wins."""

    def __init__(self) -> None:
        self.items: List[Bomb] = []

    def find(self, x: float, y: float, radius: float = BOMB_MERGE,
             status: Optional[int] = None) -> Optional[Bomb]:
        best, best_d = None, radius
        for b in self.items:
            if status is not None and b.status != status:
                continue
            d = math.hypot(b.x - x, b.y - y)
            if d < best_d:
                best, best_d = b, d
        return best

    def observe_free(self, x: float, y: float, step: int) -> Bomb:
        b = self.find(x, y)
        if b is None:
            b = Bomb(x, y, 0, step)
            self.items.append(b)
        else:
            b.x, b.y = 0.5 * (b.x + x), 0.5 * (b.y + y)
            b.status, b.seen = 0, step
        return b

    def observe_grasped(self, x: float, y: float, step: int) -> None:
        b = self.find(x, y, status=0)
        if b is not None:
            b.status, b.seen = 1, step

    def merge_remote(self, x: float, y: float, status: int, seen: int) -> None:
        b = self.find(x, y)
        if b is None:
            self.items.append(Bomb(x, y, status, seen))
        elif seen > b.seen:
            b.x, b.y, b.status, b.seen = x, y, status, seen

    def free(self, now: int) -> List[Bomb]:
        return [b for b in self.items if b.status == 0 and b.fail_until < now]

    def digest(self, limit: int = 40) -> Tuple:
        items = sorted(self.items, key=lambda b: -b.seen)[:limit]
        return tuple((round(b.x, 1), round(b.y, 1), b.status, b.seen) for b in items)


class RescueBrain(BrainBase):
    def __init__(self, identifier: int, n_drones: int, size_area: Tuple[int, int],
                 max_steps: Optional[int]) -> None:
        super().__init__(identifier, n_drones, size_area, max_steps)
        self.bombs = BombTable()
        self.claims: Dict[int, Tuple[float, float, float, int]] = {}  # id -> bx, by, cost, step
        self.claim: Optional[Bomb] = None
        self.claim_cost = math.inf
        self.claim_step = -10_000
        self.claim_eval_step = -10_000
        self.approach_start = -1
        self.last_rel: Optional[Tuple[float, float]] = None   # range, bearing of the claim
        self.carried: Optional[Bomb] = None
        self.carried_from: Optional[Tuple[float, float]] = None
        self.grasp_side = 0.0
        self.deliver_start = -1
        self.deliver_aim: Optional[Tuple[float, float]] = None
        self.bad_aims: List[Tuple[float, float]] = []
        self.was_grasping = False
        self.disposals: List[Tuple[float, float, int]] = []
        self.detections: List[BombDetection] = []
        self._dc_field: Optional[np.ndarray] = None
        self._dc_field_step = -10_000
        self.diag.update({"grasps": 0, "disposed": 0, "drops": 0, "grasp_timeouts": 0,
                          "deliver_retries": 0, "claims": 0, "yields": 0})

    # ------------------------------------------------------------------ API
    def step(self, obs: Observation) -> Command:
        if not self._begin(obs):
            return Command()
        self._read_messages(obs)
        self._perceive(obs)
        cmd = self._decide(obs)
        self.was_grasping = obs.grasping
        self._finish(obs, cmd)
        self._compose_message(obs)
        return cmd

    def diagnostics(self) -> Dict[str, Any]:
        out = dict(self.diag)
        out.update({"state": self.state, "bombs_known": len(self.bombs.items),
                    "bombs_free": sum(b.status == 0 for b in self.bombs.items),
                    "bombs_gone": sum(b.status == 2 for b in self.bombs.items),
                    "dc_points": len(self.dc.points),
                    "est_disturbances": self.est.disturbance_count})
        return out

    @property
    def busy(self) -> bool:
        return self.carried is not None or self.claim is not None

    # ------------------------------------------------------------- messages
    def _read_messages(self, obs: Observation) -> None:
        now = obs.step
        for msg in valid_messages(obs.messages, self.id):
            sender = msg["id"]
            self.team.update(msg, now)
            self._read_fleet(msg)
            for item in msg.get("b", ()):
                if isinstance(item, tuple) and len(item) == 4:
                    self.bombs.merge_remote(float(item[0]), float(item[1]), int(item[2]),
                                            int(item[3]))
            claim = msg.get("cl")
            if isinstance(claim, tuple) and len(claim) == 3:
                self.claims[sender] = (float(claim[0]), float(claim[1]), float(claim[2]),
                                       int(msg["t"]))
            elif sender in self.claims:
                del self.claims[sender]
            for item in msg.get("rc", ()):
                if isinstance(item, tuple) and len(item) == 5 and item[0] != self.id \
                        and item[0] != sender:
                    known = self.claims.get(int(item[0]))
                    if known is None or known[3] < int(item[4]):
                        self.claims[int(item[0])] = (float(item[1]), float(item[2]),
                                                     float(item[3]), int(item[4]))
            box = msg.get("dc")
            if isinstance(box, tuple) and len(box) == 4 and self.dc.box is None:
                self.dc.shared_box = tuple(float(v) for v in box)
            for item in msg.get("dp", ()):
                if isinstance(item, tuple) and len(item) == 3:
                    self.bombs.merge_remote(float(item[0]), float(item[1]), 2, int(item[2]))
        for sender in list(self.claims):
            if now - self.claims[sender][3] > CLAIM_TIMEOUT:
                del self.claims[sender]

    def _compose_message(self, obs: Observation) -> None:
        now = obs.step
        claim = None
        if self.claim is not None:
            claim = (round(self.claim.x, 1), round(self.claim.y, 1), round(self.claim_cost, 1))
        relayed = tuple((k, round(v[0], 1), round(v[1], 1), round(v[2], 1), v[3])
                        for k, v in self.claims.items() if now - v[3] < CLAIM_TIMEOUT // 2)
        box = self.dc.box
        self.msg_out = make_message(
            self.id, now, (self.est.x, self.est.y),
            r="B", s=self.state, c=bool(obs.grasping), bz=self.busy,
            tg=(round(self.target[0], 1), round(self.target[1], 1)) if self.target else None,
            b=self.bombs.digest(), cl=claim, rc=relayed, tp=self._fleet_digest(now, self.busy),
            dc=tuple(round(v, 1) for v in box) if box else None,
            dp=tuple(self.disposals[-6:]))

    # ------------------------------------------------------------ perception
    def _perceive(self, obs: Observation) -> None:
        est = self.est
        self.dc.observe(est.x, est.y, est.heading, obs.semantic)
        self.detections = []
        if obs.semantic is not None:
            self.detections = bomb_detections(est.x, est.y, est.heading, obs.semantic,
                                              obs.lidar, obs.lidar_angles)
            fresh = []
            for det in self.detections:
                if self.dc.distance(det.x, det.y) < 6.0:
                    continue
                if det.grasped:
                    self.bombs.observe_grasped(det.x, det.y, obs.step)
                else:
                    fresh.append(self.bombs.observe_free(det.x, det.y, obs.step))
            self._expire_missing(obs, fresh)
        if obs.lidar is not None and (obs.step + self.id) % P.SCAN_PERIOD == 0 \
                and est.steps_without_gps < 60:
            _, _, invalid = self._lidar_mask(
                obs, [(b.x, b.y) for b in self.bombs.items if b.status != 2])
            self._integrate(obs, invalid)

    def _expire_missing(self, obs: Observation, fresh: List[Bomb]) -> None:
        """A free bomb close by and in sight that is not detected is gone."""
        x, y = self.est.x, self.est.y
        for b in self.bombs.items:
            if b.status != 0 or b in fresh:
                continue
            if math.hypot(b.x - x, b.y - y) < 100.0 and obs.step - b.seen > 10 \
                    and self.grid.clear_along(x, y, b.x, b.y):
                b.status, b.seen = 2, obs.step

    # -------------------------------------------------------------- decision
    def _decide(self, obs: Observation) -> Command:
        est = self.est
        valid = None
        if obs.lidar is not None:
            valid = ~entity_mask(obs.semantic, obs.lidar, obs.lidar_angles)
        view = LocalView(est.heading, obs.lidar, obs.lidar_angles, valid=valid)
        others = self._others(obs)
        if obs.grasping and not self.was_grasping:
            self._on_grasp(obs)
        if self.was_grasping and not obs.grasping:
            self._on_release(obs)
        if obs.step < self.recover_until:
            return self._recover_command(obs, view, others)
        if obs.grasping:
            cmd = self._carry(obs, view, others)
        else:
            self._update_claim(obs)
            cmd = self._fetch(obs, view, others) if self.claim is not None else \
                self._explore(obs, view, others)
        if self.state != "approach":
            self._check_stuck(obs, view, cmd,
                              block_target=not obs.grasping and self.claim is None)
        return cmd

    # -- grasp events -----------------------------------------------------
    def _on_grasp(self, obs: Observation) -> None:
        self.diag["grasps"] += 1
        x, y = self.est.x, self.est.y
        near = [b for b in self.bombs.items if b.status == 0
                and math.hypot(b.x - x, b.y - y) < 50.0]
        if near:
            target = min(near, key=lambda b: (b is not self.claim,
                                              math.hypot(b.x - x, b.y - y)))
        else:
            target = self.bombs.observe_free(
                x + P.GRASP_REACH * math.cos(self.est.heading),
                y + P.GRASP_REACH * math.sin(self.est.heading), obs.step)
        if self.last_rel is not None and target is self.claim:
            self.grasp_side = self.last_rel[1]
        else:
            self.grasp_side = wrap(math.atan2(target.y - y, target.x - x) - self.est.heading)
        target.status, target.seen = 1, obs.step
        self.carried = target
        self.carried_from = (target.x, target.y)
        self.claim = None
        self.path = []
        self.deliver_start = -1
        self.deliver_aim = None

    def _on_release(self, obs: Observation) -> None:
        a = self.est.heading + self.grasp_side
        fx = self.est.x + P.CARRY_OFFSET * math.cos(a)
        fy = self.est.y + P.CARRY_OFFSET * math.sin(a)
        carried = self.carried
        if self.dc.distance(fx, fy) < 45.0:
            self.diag["disposed"] += 1
            src = self.carried_from or (fx, fy)
            if carried is not None:
                carried.status, carried.seen = 2, obs.step
            self.disposals.append((round(src[0], 1), round(src[1], 1), obs.step))
        else:
            self.diag["drops"] += 1
            if carried is not None:
                carried.x, carried.y, carried.status, carried.seen = fx, fy, 0, obs.step
            else:
                self.bombs.observe_free(fx, fy, obs.step)
        self.carried = None
        self.carried_from = None
        self.state = "search"
        self.path = []

    # -- claims -----------------------------------------------------------
    def _update_claim(self, obs: Observation) -> None:
        now = obs.step
        if self.claim is not None:
            if self.claim.status != 0 or self.claim.fail_until >= now:
                self.claim = None
            elif self._outranked(self.claim, self.claim_cost):
                self.diag["yields"] += 1
                self.claim = None
        if self.claim is not None and now - self.claim_step < 50:
            return
        if now - self.claim_eval_step < CLAIM_PERIOD and self.claim is None:
            return
        self.claim_eval_step = now
        free = self.bombs.free(now)
        if not free:
            self.claim = None
            return
        field = self._field_from_me(now)
        rivals = [(k, v[0], v[1]) for k, v in self.fleet.items()
                  if not v[3] and now - v[2] < FLEET_MAX_AGE]
        best, best_cost = None, math.inf
        for b in free:
            r, c = self.grid.to_cell(b.x, b.y)
            euclid = math.hypot(b.x - self.est.x, b.y - self.est.y)
            cost = field[r, c] * self.grid.cell if np.isfinite(field[r, c]) else 1.5 * euclid
            if b is self.claim:
                cost *= 0.85
            rival = min((1.25 * math.hypot(b.x - rx, b.y - ry) for _, rx, ry in rivals),
                        default=math.inf)
            if cost > 1.2 * rival + 40.0:
                continue
            if cost < best_cost and not self._outranked(b, cost):
                best, best_cost = b, cost
        if best is not self.claim:
            self.diag["claims"] += 1 if best is not None else 0
            self.path = []
            self.approach_start = -1
            self.last_rel = None
        self.claim, self.claim_cost = best, best_cost
        self.claim_step = now

    def _outranked(self, bomb: Bomb, cost: float) -> bool:
        for other, (bx, by, ocost, _) in self.claims.items():
            if math.hypot(bx - bomb.x, by - bomb.y) < BOMB_MERGE:
                if ocost < cost - 1e-6 or (abs(ocost - cost) <= 1e-6 and other < self.id):
                    return True
        return False

    # -- behaviours -------------------------------------------------------
    def _fetch(self, obs: Observation, view: LocalView, others) -> Command:
        est = self.est
        b = self.claim
        det = None
        for d in self.detections:
            if not d.grasped and math.hypot(d.x - b.x, d.y - b.y) < 45.0 and \
                    (det is None or d.range < det.range):
                det = d
        if det is not None:
            b.x, b.y, b.seen = det.x, det.y, obs.step
            self.last_rel = (det.range, det.bearing)
        cmd = Command()
        dist = det.range if det is not None else math.hypot(b.x - est.x, b.y - est.y)
        if det is not None and dist < APPROACH_RANGE:
            if self.state != "approach":
                self.state = "approach"
                self.approach_start = obs.step
            if obs.step - self.approach_start > GRASP_TIMEOUT:
                self.diag["grasp_timeouts"] += 1
                b.fail_until = obs.step + 400
                self.claim = None
                self.state = "search"
                return cmd
            bearing_world = est.heading + det.bearing
            aim = (est.x + det.range * math.cos(bearing_world),
                   est.y + det.range * math.sin(bearing_world))
            cmd.rotation = rotation_toward(est.heading, bearing_world)
            facing = abs(det.bearing) < 0.35
            cruise = (2.5 if dist < 60.0 else 6.0) if facing else 1.0
            cmd.grasper = 1 if dist < 90.0 else 0
            f, l, _ = drive(est, aim, view, cruise=cruise, carrying=False, others=others,
                            stop_at_target=False, repel_walls=dist > 45.0)
            cmd.forward, cmd.lateral = f, l
            self.target = aim
            return cmd
        self.state = "goto"
        target = self._path_target(obs, (b.x, b.y))
        heading = math.atan2(b.y - est.y, b.x - est.x) if dist < 220.0 else \
            math.atan2(target[1] - est.y, target[0] - est.x)
        cmd.rotation = rotation_toward(est.heading, heading, gain=0.6)
        f, l, _ = drive(est, target, view, cruise=P.MAX_SPEED * 0.85, carrying=False,
                        others=others, stop_at_target=False)
        cmd.forward, cmd.lateral = f, l
        self.target = (b.x, b.y)
        return cmd

    def _carry(self, obs: Observation, view: LocalView, others) -> Command:
        est = self.est
        cmd = Command(grasper=1)
        if not self.dc.known:
            self.state = "carry"
            target = self.home
        else:
            if self.dc.distance(est.x, est.y) < DELIVER_RANGE:
                deliver = self._deliver(obs, view)
                if deliver is not None:
                    return deliver
            self.state = "carry"
            target = self._path_target_dc(obs)
        heading = math.atan2(target[1] - est.y, target[0] - est.x) - self.grasp_side
        cmd.rotation = rotation_toward(est.heading, heading, gain=0.6)
        f, l, _ = drive(est, target, view, cruise=P.CARRY_SPEED, carrying=True,
                        others=others, stop_at_target=False)
        cmd.forward, cmd.lateral = f, l
        self.target = target
        return cmd

    def _deliver(self, obs: Observation, view: LocalView) -> Optional[Command]:
        est = self.est
        if self.deliver_aim is None or obs.step - self.deliver_start > DELIVER_TIMEOUT:
            if self.deliver_aim is not None:
                self.diag["deliver_retries"] += 1
                self.bad_aims.append(self.deliver_aim)
            self.deliver_aim = self._choose_aim()
            self.deliver_start = obs.step
        aim = self.deliver_aim
        if aim is None:
            return None
        if math.hypot(aim[0] - est.x, aim[1] - est.y) > 50.0 and \
                not self.grid.clear_along(est.x, est.y, aim[0], aim[1], stop_short=24.0):
            return None
        self.state = "deliver"
        bearing = math.atan2(aim[1] - est.y, aim[0] - est.x)
        desired = bearing - self.grasp_side
        cmd = Command(grasper=1)
        cmd.rotation = rotation_toward(est.heading, desired, gain=1.0)
        aligned = abs(wrap(desired - est.heading)) < 0.3
        f, l, _ = drive(est, aim, view, cruise=P.CARRY_SPEED if aligned else 0.8,
                        carrying=True, others=[], stop_at_target=False, repel_walls=False)
        cmd.forward, cmd.lateral = f, l
        self.target = aim
        return cmd

    def _choose_aim(self) -> Optional[Tuple[float, float]]:
        est = self.est
        pts = [p for p in self.dc.points
               if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 25.0 for q in self.bad_aims)]
        if not pts:
            self.bad_aims = []
            pts = self.dc.points
        if not pts:
            return None
        arr = np.asarray(pts)
        i = int(np.argmin((arr[:, 0] - est.x) ** 2 + (arr[:, 1] - est.y) ** 2))
        return float(arr[i, 0]), float(arr[i, 1])

    def _explore(self, obs: Observation, view: LocalView, others) -> Command:
        est = self.est
        now = obs.step
        if (self.state != "explore" or self.target is None or now - self.goal_step > EXPLORE_PERIOD
                or math.hypot(self.target[0] - est.x, self.target[1] - est.y) < 30.0):
            self.target = self._choose_explore_target(now)
            self.goal_step = now
            self.path = []
        self.state = "explore"
        if self.target is None:
            return self._idle(obs, view, others)
        return self._goto(obs, view, others, self.target, cruise=P.MAX_SPEED * 0.85)

    def _idle(self, obs: Observation, view: LocalView, others) -> Command:
        self.state = "idle"
        spot = self.return_area or self.home
        if spot is None:
            return Command()
        aim = (spot[0] + 45.0 * math.cos(2.4 * self.id), spot[1] + 45.0 * math.sin(2.4 * self.id))
        target = self._path_target(obs, aim)
        f, l, _ = drive(self.est, target, view, cruise=4.0, carrying=False, others=others)
        return Command(forward=f, lateral=l)

    def _path_target_dc(self, obs: Observation) -> Tuple[float, float]:
        est = self.est
        now = obs.step
        if self._dc_field is None or now - self._dc_field_step > 60:
            box = self.dc.bbox()
            cells = []
            if box is not None:
                cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
                for px, py in self.dc.points[::5] or [(cx, cy)]:
                    dx, dy = px - cx, py - cy
                    norm = math.hypot(dx, dy) or 1.0
                    cells.append(self.grid.to_cell(px + 32.0 * dx / norm, py + 32.0 * dy / norm))
            self._dc_field = self.planner.field(sorted(set(cells)), allow_unknown=True, step=now)
            self._dc_field_step = now
        cells = self.planner.descend(self._dc_field, self.grid.to_cell(est.x, est.y), max_len=12)
        if len(cells) >= 2:
            path = [self.grid.to_world(r, c) for r, c in cells]
            return lookahead_point(path, est.x, est.y, P.WAYPOINT_LOOKAHEAD,
                                   lambda px, py: self.grid.clear_along(est.x, est.y, px, py))
        surface = self.dc.nearest_surface(est.x, est.y)
        return surface if surface is not None else (est.x, est.y)
