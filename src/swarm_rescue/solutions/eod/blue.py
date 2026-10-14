"""Blue (rescue) brain, candidate B4-S (B2 in Phase 3, B1 in Phase 2, B0 and B0.1 in Phase 1).

B4-S (Phase 5) differs from B2 (frozen as the package ``p3_067818a``) in two
places. (1) Where B2's carrier has no exit, B4-S looks for one: with the
center unknown it searches for it along planned paths instead of waiting at
its start area; with the center known but no verified approach, when B2's
goals are unreachable or an approach on them failed, it visits view positions
around the known box for a while, excluding those from which no face was seen,
and then returns to B2's goals. Everywhere else the delivery is B2's
(``_carry``, ``_seek``). (2) A drone fetching a bomb keeps its grasper off
near a bomb another drone holds (``_grasp_guard``). (3) A teammate's center
box is accepted only when finite and well ordered.

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
* carry and deliver (B2): follow the geodesic field to stand-off positions
  witnessed by the drone's own detections of the center (``delivery.py``:
  each lies on a segment along which the center was seen, so never behind a
  wall), in which unobserved cells cost more than for unloaded travel (a
  carrying drone is slow, so a route that flips when a wall appears is
  expensive). The terminal push drives the side on which the bomb was
  grasped into the center only with a line-of-sight certificate at every
  distance: a surface point detected in this step (walls stop semantic rays)
  whose segment the map does not block, or the surface point of the
  witnessed stand-off the drone stands on. Without progress near the center
  the approach area is excluded for a while and the route re-planned. A
  detachment next to the center counts as a delivery, anywhere else as a
  drop that is re-queued. Surface points enter the stored model only while
  the heading estimate is trusted.
* wait: idle drones wait at a reachable spot away from the center, so that
  they do not block deliveries.

Missing sensors, missing messages or an unknown disposal center degrade the
behaviour (dead reckoning, local decisions, seeking the center while carrying,
then the start area along planned paths) instead of stopping it.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from . import params as P
from .base import TARGET_BUDGET_MAX, TARGET_BUDGET_MIN, BrainBase
from .comms import make_message, numeric_tuple, valid_messages
from .delivery import (MAX_PENALTIES, PENALTY_RADIUS, PENALTY_STEPS, STANDOFF, SURFACE_STOP,
                       ApproachSet, radial_sources, segments_clear)
from .geometry import wrap
from .motion import LocalView, drive, lookahead_point, rotation_toward
from .observation import Command, Observation
from .perception import BombDetection, bomb_detections, entity_mask, live_surface_points

BOMB_MERGE = 35.0
APPROACH_RANGE = 140.0
GRASP_TIMEOUT = 140
CLAIM_TIMEOUT = 40
CLAIM_PERIOD = 8
EXPLORE_PERIOD = 40
DELIVER_RANGE = 100.0
FLEET_MAX_AGE = 200
DIGEST_MEMORY = 4096      # remembered digest items before the set is reset
DC_RECORD_HEADING_SIGMA = math.radians(8.0)
CARRY_UNKNOWN_COST = 10.0  # carry routes prefer observed free space (default 3)
LIVE_AIM_RANGE = 120.0
TERMINAL_STOP = 14.0       # the last part of a terminal segment lies on the mapped center
DC_FIELD_PERIOD = 60       # steps between refreshes of the center's field
APPROACH_PATIENCE = 160    # steps near the center without progress before a re-plan
PROGRESS_RANGE = 150.0     # distance to the center within which progress is watched
PROGRESS_EPS = 5.0         # px of distance gain that count as progress
PROGRESS_MOVE = 80.0       # a drone that moved this far is travelling, not stuck
IDLE_DC_CLEARANCE = 130.0  # idle drones wait at least this far from the center
IDLE_PERIOD = 100
# Approach seeking (B4-S, Phase 5): a carrier without a verified approach looks
# for one instead of waiting. Every value is derived from an existing constant.
SEEK_VIEW_OFFSET = 2.0 * STANDOFF          # 72 px out from a face: beyond the stand-off ring, well
                                           # inside LIVE_AIM_RANGE, so a visible face gives live points
SEEK_VIEW_SPACING = SEEK_VIEW_OFFSET       # a face seen from that distance spans about twice it
SEEK_ARRIVE = 30.0                         # reached, as for exploration targets (base._explore_due)
SEEK_MARK_RADIUS = PENALTY_RADIUS          # a view or search goal that failed is excluded like a
SEEK_MARK_STEPS = PENALTY_STEPS            # failed approach (delivery.py)
SEEK_MAX_MARKS = MAX_PENALTIES
SEEK_BUDGET_MIN = TARGET_BUDGET_MIN        # travel-time budget: unloaded bounds rescaled to the
SEEK_BUDGET_MAX = int(TARGET_BUDGET_MAX * P.MAX_SPEED / P.CARRY_SPEED)   # carrying speed
SEEK_RECHECK = DC_FIELD_PERIOD             # steps before seeking again when nothing was left
ESCAPE_STEPS = PENALTY_STEPS               # view seeking after B2's goals gave no exit, then B2 again
SEEK_CHECK_PERIOD = 10                     # steps between re-evaluations of the verified approach
MIN_OWN_POINTS = 4                         # own points before an own box exists (perception.bbox)
GRASP_GUARD_BOMB = P.GRASP_REACH + 2.0 * P.BOMB_RADIUS   # a held bomb's centre this close is reachable
GRASP_GUARD_CARRIER = (P.GRASP_REACH + P.CARRY_OFFSET + P.BOMB_RADIUS
                       + 2.0 * P.GPS_NOISE)              # a carrier's reported position, GPS error added


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
        self.was_grasping = False
        self.disposals: List[Tuple[float, float, int]] = []
        self.detections: List[BombDetection] = []
        self._digest_seen: set = set()
        self.dc_live: List[Tuple[float, float]] = []
        self.approach = ApproachSet()
        self._dc_field: Optional[np.ndarray] = None
        self._dc_field_step = -10_000
        self._dc_field_version = -1
        self._relaxed = False      # every supported stand-off is penalized
        # Progress watch near the center: best distance, its step and position.
        self.progress: Optional[Tuple[float, int, float, float]] = None
        self._idle_spot: Optional[Tuple[int, Tuple[float, float]]] = None
        # B4-S approach seeking: current goal, its kind ("view" around a known box or
        # "search" for an unknown center), when it was chosen, its travel budget, the
        # last step a center surface was in sight, failed goals (bounded), and when
        # seeking may resume after nothing was left to seek.
        self._seek_goal: Optional[Tuple[float, float]] = None
        self._seek_kind = ""
        self._seek_step = -10_000
        self._seek_budget = 0
        self._seen_dc_step = -10_000
        self._seek_marks: List[Tuple[float, float, int]] = []
        self._seek_resume = -1
        self._escape_until = -1      # view seeking replaces B2's unverified goals until then
        self._verified: Optional[Tuple[int, int, int, bool]] = None   # step, witnesses, version, result
        self.diag.update({"grasps": 0, "disposed": 0, "drops": 0, "grasp_timeouts": 0,
                          "claims": 0, "yields": 0, "live_aims": 0, "witness_aims": 0,
                          "approach_failures": 0, "relaxed_sources": 0,
                          "radial_fallbacks": 0, "terminal_blocked": 0,
                          "seek_goals": 0, "seek_steps": 0, "seek_views_empty": 0,
                          "seek_expired": 0, "seek_exhausted": 0, "escapes": 0,
                          "grasp_guards": 0})

    # ------------------------------------------------------------------ API
    def step(self, obs: Observation) -> Command:
        if not self._begin(obs):
            return Command()
        self._read_messages(obs)
        self._perceive(obs)
        self._coverage_tick(obs.step)
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
                    "approach_witnesses": len(self.approach),
                    "est_disturbances": self.est.disturbance_count,
                    "est_carry_odometer_steps": self.est.carry_odometer_steps,
                    "est_carry_model_steps": self.est.carry_model_steps})
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
            self._read_coverage(msg)
            for item in msg.get("b", ()):
                # Digests repeat every step; an item already applied is skipped
                # (re-applying it could only recreate stale records).
                if item in self._digest_seen or not numeric_tuple(item, 4):
                    continue
                self._digest_seen.add(item)
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
            # B4-S: only a finite, well-ordered box is accepted (B2 accepted any
            # 4-tuple; a non-finite one made the center "known" and later failed
            # in planning on every step).
            if numeric_tuple(box, 4) and box[0] < box[2] and box[1] < box[3] \
                    and self.dc.box is None:
                self.dc.shared_box = tuple(float(v) for v in box)
            for item in msg.get("dp", ()):
                if item in self._digest_seen or not numeric_tuple(item, 3):
                    continue
                self._digest_seen.add(item)
                self.bombs.merge_remote(float(item[0]), float(item[1]), 2, int(item[2]))
        if len(self._digest_seen) > DIGEST_MEMORY:
            self._digest_seen.clear()
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
            dp=tuple(self.disposals[-6:]), cv=self._coverage_message(now))

    # ------------------------------------------------------------ perception
    def _perceive(self, obs: Observation) -> None:
        est = self.est
        self.dc_live = live_surface_points(est.x, est.y, est.heading, obs.semantic)
        # Surface points recorded during a large heading error would persist
        # as outliers of the center model; record only with a trusted heading.
        if est.heading_sigma < DC_RECORD_HEADING_SIGMA:
            self.dc.observe(est.x, est.y, est.heading, obs.semantic)
            if self.dc_live:
                self.approach.observe(self.grid, est.x, est.y, self.dc_live, obs.step)
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
            _, _, transient, _ = self._lidar_mask(
                obs, [(b.x, b.y) for b in self.bombs.items if b.status != 2])
            self._integrate(obs, transient)

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
        self.progress = None
        self._seek_goal = None       # a new carry starts without a stale seek goal

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
            cmd.grasper = 1 if dist < 90.0 and not self._grasp_guard(obs) else 0
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
        """Carry the held bomb (B4-S).

        Center known: B2's delivery, unchanged (progress watch, certified
        terminal push, carry field to verified stand-offs or, without them, to
        B2's radial goals), except where B2 has no exit. Without a verified
        approach (``_verified_approach``), if B2's goals cannot be reached
        (B2 waits at its start area) or an approach on them fails (B2 returns
        to the same side), the carrier seeks a view of a face for
        ``ESCAPE_STEPS`` (``_seek``) and then tries B2's goals again; a face in
        sight allows a certified terminal push meanwhile. Center unknown: B2
        drove straight to its start area and waited; B4-S searches for the
        center (``_seek``) and, with nothing left to search, goes to its start
        area along a planned path and searches again later.
        """
        est = self.est
        cmd = Command(grasper=1)
        now = obs.step
        if self.dc_live:
            self._seen_dc_step = now
        target = None
        escape = False
        tried = False                                   # terminal push already tried this step
        if self.dc.known:
            verified = self._verified_approach(now)
            if verified or now >= self._escape_until:
                # B2's delivery, unchanged: verified stand-offs, or B2's radial
                # goals (through unknown cells they draw the carrier round the
                # walls of a partly known center; Phase 4 Risk A).
                self._seek_goal = None
                dist = self.dc.distance(est.x, est.y)
                failures = self.diag["approach_failures"]
                self._watch_progress(obs, dist)
                if dist < DELIVER_RANGE:
                    tried = True
                    deliver = self._deliver(obs, view)
                    if deliver is not None:
                        return deliver
                self.state = "carry"
                target = self._path_target_dc(obs)
                if not verified and (not self._dc_route() or
                                     self.diag["approach_failures"] > failures):
                    # B2 has no exit here: no route to its unverified goals
                    # (it would wait at its start area), or an approach on
                    # them failed (it would come back to the same side).
                    self._escape_until = now + ESCAPE_STEPS
                    self.diag["escapes"] += 1
                    escape = True
            else:
                escape = True
            if escape:
                if not tried and self.dc_live and \
                        self.dc.distance(est.x, est.y) < DELIVER_RANGE:
                    deliver = self._deliver(obs, view)  # certified by a live point
                    if deliver is not None:
                        return deliver
                seek = self._seek(obs)
                if seek is not None:
                    target = seek
                elif target is None:                    # nothing to seek: B2's goals
                    self.state = "carry"
                    target = self._path_target_dc(obs)
        else:
            target = self._seek(obs)
            if target is None:                          # nothing left to search (for now)
                self.state = "carry"
                target = self._path_target(obs, self.home) if self.home is not None \
                    else (est.x, est.y)
        heading = math.atan2(target[1] - est.y, target[0] - est.x) - self.grasp_side
        cmd.rotation = rotation_toward(est.heading, heading, gain=0.6)
        f, l, _ = drive(est, target, view, cruise=P.CARRY_SPEED, carrying=True,
                        others=others, stop_at_target=False)
        cmd.forward, cmd.lateral = f, l
        # While seeking, teammates are told the seek goal (spreading), not the
        # next path point.
        self.target = self._seek_goal if self._seek_goal is not None else target
        return cmd

    # -- B4-S: grasp guard, verified approach, approach seeking ------------------
    def _grasp_guard(self, obs: Observation) -> bool:
        """Keep the grasper off near a bomb that another drone holds.

        The framework lets a second drone grasp a bomb already held (one bomb
        per drone, but no check of the bomb's holders); the semantic sensor
        flags a held bomb as grasped for every other drone, and teammates
        report whether they carry. A drone fetching its own claim therefore
        does not close its grasper while a held bomb or a carrying teammate is
        within grasping reach, so it cannot capture a teammate's bomb on the
        way (Phase 4 S2-2, two drones on one bomb). The guard only delays a
        grasp; it never changes a claim.
        """
        est = self.est
        held = any(d.grasped and d.range < GRASP_GUARD_BOMB for d in self.detections)
        if not held:
            for _, m in self.team.recent(obs.step, 3):
                if m.carrying and math.hypot(m.x - est.x, m.y - est.y) < GRASP_GUARD_CARRIER:
                    held = True
                    break
        if held:
            self.diag["grasp_guards"] += 1
        return held

    def _verified_approach(self, now: int) -> bool:
        """Whether the carrier holds approach evidence B2's delivery can trust.

        Verified: a supported, unpenalized witnessed stand-off, or, with an
        own box (at least ``MIN_OWN_POINTS`` own surface points), a radial
        stand-off that passes the map check and lies outside every penalized
        area. A teammate's box alone, a handful of own points, or goals that
        survive only as B2's unchecked fallback are not verified. Re-evaluated
        every ``SEEK_CHECK_PERIOD`` steps or when the witnesses change.
        """
        if not self.dc.known:
            return False
        key = (len(self.approach), self.approach.version)
        cached = self._verified
        if cached is not None and (cached[1], cached[2]) == key and \
                now - cached[0] < SEEK_CHECK_PERIOD:
            return cached[3]
        ok = bool(self.approach.valid(self.grid, now))
        if not ok and self.dc.box is not None and len(self.dc.points) >= MIN_OWN_POINTS:
            ok = self._own_radial_ok(now)
        self._verified = (now, key[0], key[1], ok)
        return ok

    def _dc_route(self) -> bool:
        """Whether B2's current carry field reaches the drone (a route exists)."""
        field = self._dc_field
        if field is None:
            return False
        return bool(self.planner.descend(field, self.grid.to_cell(self.est.x, self.est.y),
                                         max_len=1))

    def _own_radial_ok(self, now: int) -> bool:
        """A map-checked, unpenalized radial stand-off from the own box exists."""
        box = self.dc.box
        cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
        pts = np.asarray(self.dc.points[::5], dtype=float)
        dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
        norm = np.maximum(np.hypot(dx, dy), 1e-9)
        sx, sy = pts[:, 0] + 32.0 * dx / norm, pts[:, 1] + 32.0 * dy / norm
        cells = self.grid._cells(sx, sy)
        occ = self.grid.occupied().reshape(-1)
        ok = (cells >= 0) & ~occ[np.maximum(cells, 0)]
        ok &= segments_clear(self.grid, sx, sy, pts[:, 0], pts[:, 1], stop_short=SURFACE_STOP)
        ok &= ~self.approach._penalized(sx, sy, now)
        return bool(ok.any())

    def _seek(self, obs: Observation) -> Optional[Tuple[float, float]]:
        """Path point towards the current seek goal, or None if nothing is left.

        Center unknown: a reachable unsearched or frontier cell (``search``).
        Center known from a box but no verified approach: a view position
        ``SEEK_VIEW_OFFSET`` out from a face of the box (``view``). A goal is
        kept until reached, until its travel budget (carrying speed) expires
        or until the kind changes. A view position reached or abandoned
        without any center surface in sight since it was chosen, and a goal
        whose budget expired, are excluded for ``SEEK_MARK_STEPS``
        (negative evidence: walls stop semantic rays, so a face that is not
        seen from there is not approachable from there). With no candidate
        left, seeking pauses for ``SEEK_RECHECK`` steps.
        """
        now = obs.step
        est = self.est
        if now < self._seek_resume:
            return None
        kind = "view" if self.dc.known else "search"
        goal = self._seek_goal
        if goal is not None:
            reached = math.hypot(goal[0] - est.x, goal[1] - est.y) < SEEK_ARRIVE
            expired = now - self._seek_step > self._seek_budget
            if kind != self._seek_kind or reached or expired:
                unseen = self._seen_dc_step < self._seek_step
                if expired:
                    self.diag["seek_expired"] += 1
                if (self._seek_kind == "view" and unseen) or expired:
                    if self._seek_kind == "view" and unseen:
                        self.diag["seek_views_empty"] += 1
                    self._mark(goal, now)
                goal = None
        if goal is None:
            choice = self._choose_view(now) if kind == "view" else self._choose_search(now)
            if choice is None:
                self._seek_goal = None
                self._seek_resume = now + SEEK_RECHECK
                self.diag["seek_exhausted"] += 1
                return None
            goal, budget = choice
            self._seek_goal, self._seek_kind = goal, kind
            self._seek_step, self._seek_budget = now, budget
            self.diag["seek_goals"] += 1
            self.path = []
        self.state = "seek"
        self.diag["seek_steps"] += 1
        return self._path_target(obs, goal)

    def _mark(self, goal: Tuple[float, float], now: int) -> None:
        marks = [m for m in self._seek_marks if m[2] >= now]
        marks.append((goal[0], goal[1], now + SEEK_MARK_STEPS))
        self._seek_marks = marks[-SEEK_MAX_MARKS:]

    def _marked(self, xs: np.ndarray, ys: np.ndarray, now: int) -> np.ndarray:
        out = np.zeros(xs.shape, dtype=bool)
        for mx, my, until in self._seek_marks:
            if until >= now:
                out |= (xs - mx) ** 2 + (ys - my) ** 2 < SEEK_MARK_RADIUS ** 2
        return out

    def _seek_budget_for(self, cost_px: float) -> int:
        travel = cost_px / (0.5 * P.CARRY_SPEED)
        return int(min(SEEK_BUDGET_MAX, max(SEEK_BUDGET_MIN, travel + 40)))

    def _choose_view(self, now: int) -> Optional[Tuple[Tuple[float, float], int]]:
        """Cheapest reachable view position around the known box (deterministic)."""
        box = self.dc.bbox()
        x0, y0, x1, y1 = box
        off, step = SEEK_VIEW_OFFSET, SEEK_VIEW_SPACING
        pts = []
        for x in np.arange(x0, x1 + 1e-6, step).tolist() + [x1]:
            pts += [(x, y1 + off), (x, y0 - off)]
        for y in np.arange(y0, y1 + 1e-6, step).tolist() + [y1]:
            pts += [(x1 + off, y), (x0 - off, y)]
        d = off / math.sqrt(2.0)
        pts += [(x0 - d, y0 - d), (x0 - d, y1 + d), (x1 + d, y0 - d), (x1 + d, y1 + d)]
        arr = np.unique(np.round(np.array(pts), 1), axis=0)
        g = self.grid
        inside = (np.abs(arr[:, 0]) < g.width / 2.0 - g.cell) & \
            (np.abs(arr[:, 1]) < g.height / 2.0 - g.cell)
        arr = arr[inside]
        if not arr.size:
            return None
        cells = g._cells(arr[:, 0], arr[:, 1])
        cost = self.planner.cost(True, now, CARRY_UNKNOWN_COST).reshape(-1)
        ok = (cells >= 0) & (cost[np.maximum(cells, 0)] > 0)
        ok &= ~self._marked(arr[:, 0], arr[:, 1], now)
        ok &= ~self.approach._penalized(arr[:, 0], arr[:, 1], now)
        arr, cells = arr[ok], cells[ok]
        if not arr.size:
            return None
        field = self.planner.field([g.to_cell(self.est.x, self.est.y)], allow_unknown=True,
                                   step=now, unknown_cost=CARRY_UNKNOWN_COST).reshape(-1)
        reach = np.isfinite(field[cells])
        arr, cells = arr[reach], cells[reach]
        if not arr.size:
            return None
        score = field[cells] * g.cell
        for _, tgt in self.team.targets(now):
            dd = np.hypot(arr[:, 0] - tgt[0], arr[:, 1] - tgt[1])
            score = score + np.where(dd < 2.0 * SEEK_VIEW_OFFSET, 2.0 * SEEK_VIEW_OFFSET - dd, 0.0)
        order = np.lexsort((arr[:, 1], arr[:, 0], score))    # ties: lower x, then lower y
        i = int(order[0])
        return (float(arr[i, 0]), float(arr[i, 1])), self._seek_budget_for(float(field[cells[i]] * g.cell))

    def _choose_search(self, now: int) -> Optional[Tuple[Tuple[float, float], int]]:
        """Reachable unsearched or frontier cell for a carrier without a center.

        The candidate set and score are those of the unloaded exploration
        (``BrainBase._choose_explore_target``: geodesic cost, unsearched
        density, teammates' targets and positions), without the start-up
        sector bias and with failed goals excluded.
        """
        g = self.grid
        searched = self.searched
        candidates = (g.free() & ~searched) | g.frontier()
        if not candidates.any():
            return None
        field = self._field_from_me(now)
        reach = np.isfinite(field) & candidates
        if not reach.any():
            return None
        rows, cols = np.nonzero(reach)
        wx = (cols + 0.5) * g.cell - g.width / 2.0
        wy = g.height / 2.0 - (rows + 0.5) * g.cell
        keep = ~self._marked(wx, wy, now)
        if not keep.any():
            return None
        rows, cols, wx, wy = rows[keep], cols[keep], wx[keep], wy[keep]
        density = cv2.blur((~searched).astype(np.float32), (7, 7))
        score = field[rows, cols] * g.cell - 120.0 * density[rows, cols]
        for _, tgt in self.team.targets(now):
            d = np.hypot(wx - tgt[0], wy - tgt[1])
            score += np.where(d < 220.0, (220.0 - d) * 2.0, 0.0)
        for _, (ox, oy, t, _) in self.fleet.items():
            if now - t < 30:
                d = np.hypot(wx - ox, wy - oy)
                score += np.where(d < 150.0, 150.0 - d, 0.0)
        i = int(np.argmin(score))                              # ties: first in row-major order
        return (float(wx[i]), float(wy[i])), self._seek_budget_for(float(field[rows[i], cols[i]] * g.cell))

    def _watch_progress(self, obs: Observation, dist: float) -> None:
        """Re-plan when a carrier near the center stops getting closer.

        Progress is a gain of ``PROGRESS_EPS`` in distance to the center; a
        drone that has moved ``PROGRESS_MOVE`` since the last progress is
        travelling (for example around a wall) and restarts the window.
        """
        est, now = self.est, obs.step
        if dist > PROGRESS_RANGE:
            self.progress = None
            return
        p = self.progress
        if p is None or dist < p[0] - PROGRESS_EPS or \
                math.hypot(est.x - p[2], est.y - p[3]) > PROGRESS_MOVE:
            self.progress = (dist, now, est.x, est.y)
            return
        if now - p[1] > APPROACH_PATIENCE:
            # The approach used here does not work: exclude stand-offs around
            # this position (longer after repeated failures) and re-plan.
            self.approach.penalize(est.x, est.y, now)
            self.diag["approach_failures"] += 1
            self.progress = (dist, now, est.x, est.y)
            self._dc_field = None
            self.path = []

    def _terminal_aim(self, obs: Observation) -> Optional[Tuple[float, float]]:
        """Aim of the terminal push, only with a line-of-sight certificate.

        Walls stop semantic rays, so a surface point detected in this step is
        in sight; it is used when the map does not block its segment either.
        Without such a detection, the drone must stand on a witnessed
        stand-off, whose surface point was seen from there. No distance
        exempts the check. Inside an area where an approach failed recently
        there is no terminal push (the field leads to another stand-off),
        unless every approach has failed (relaxed goals).
        """
        est = self.est
        if not self._relaxed and self.approach.penalized_at(est.x, est.y, obs.step):
            return None
        live = sorted(self.dc_live, key=lambda p: (p[0] - est.x) ** 2 + (p[1] - est.y) ** 2)
        for px, py in live[:3]:
            if math.hypot(px - est.x, py - est.y) >= LIVE_AIM_RANGE:
                break
            if self.grid.clear_along(est.x, est.y, px, py, stop_short=TERMINAL_STOP):
                self.diag["live_aims"] += 1
                return px, py
        aim = self.approach.at_standoff(self.grid, est.x, est.y, obs.step)
        if aim is not None and self.grid.clear_along(est.x, est.y, aim[0], aim[1],
                                                     stop_short=TERMINAL_STOP):
            self.diag["witness_aims"] += 1
            return aim
        return None

    def _deliver(self, obs: Observation, view: LocalView) -> Optional[Command]:
        est = self.est
        aim = self._terminal_aim(obs)
        if aim is None:
            self.diag["terminal_blocked"] += 1
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

    def _explore(self, obs: Observation, view: LocalView, others) -> Command:
        now = obs.step
        if self._explore_due(now, EXPLORE_PERIOD):
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
        if self.return_area is None:
            aim = self._idle_aim(obs.step, spot, aim)
        target = self._path_target(obs, aim)
        f, l, _ = drive(self.est, target, view, cruise=4.0, carrying=False, others=others)
        return Command(forward=f, lateral=l)

    def _idle_aim(self, now: int, spot: Tuple[float, float],
                  aim: Tuple[float, float]) -> Tuple[float, float]:
        """A waiting position that keeps clear of the disposal center.

        The start area lies close to the center on the supplied maps, so
        drones waiting there obstruct carriers. The chosen position is a
        known free, reachable cell at least ``IDLE_DC_CLEARANCE`` from the
        center, near the start area (cached for ``IDLE_PERIOD`` steps).
        """
        if not self.dc.known or self.dc.distance(*aim) >= IDLE_DC_CLEARANCE:
            return aim
        cached = self._idle_spot
        if cached is not None and now - cached[0] < IDLE_PERIOD:
            return cached[1]
        field = self._field_from_me(now)
        cost = self.planner.cost(False, now)
        best, best_score = aim, math.inf
        for radius in (60.0, 100.0, 140.0, 180.0, 220.0):
            for k in range(12):
                a = 2.4 * self.id + k * math.pi / 6.0
                x, y = spot[0] + radius * math.cos(a), spot[1] + radius * math.sin(a)
                if self.dc.distance(x, y) < IDLE_DC_CLEARANCE:
                    continue
                r, c = self.grid.to_cell(x, y)
                if cost[r, c] <= 0 or not np.isfinite(field[r, c]):
                    continue
                score = field[r, c] * self.grid.cell + 0.5 * math.hypot(x - aim[0], y - aim[1])
                if score < best_score:
                    best, best_score = (x, y), score
        self._idle_spot = (now, best)
        return best

    def _dc_sources(self, now: int) -> List[Tuple[int, int]]:
        """Goal cells of the carry field: witnessed stand-offs, else map-checked radial ones."""
        cells, relaxed = self.approach.sources(self.grid, now)
        self._relaxed = relaxed
        if relaxed:
            self.diag["relaxed_sources"] += 1
        if cells:
            return cells
        box = self.dc.bbox()
        if box is None:
            return []
        self.diag["radial_fallbacks"] += 1
        return radial_sources(self.grid, self.dc.points, box)

    def _path_target_dc(self, obs: Observation) -> Tuple[float, float]:
        est = self.est
        now = obs.step
        if self._dc_field is None or now - self._dc_field_step > DC_FIELD_PERIOD or \
                self._dc_field_version != self.approach.version:
            self._dc_field = self.planner.field(self._dc_sources(now), allow_unknown=True,
                                               step=now, unknown_cost=CARRY_UNKNOWN_COST)
            self._dc_field_step = now
            self._dc_field_version = self.approach.version
        cells = self.planner.descend(self._dc_field, self.grid.to_cell(est.x, est.y), max_len=12)
        if len(cells) >= 2:
            path = [self.grid.to_world(r, c) for r, c in cells]
            return lookahead_point(path, est.x, est.y, P.WAYPOINT_LOOKAHEAD,
                                   lambda px, py: self.grid.clear_along(est.x, est.y, px, py))
        if cells:
            # At a goal cell: hold here; the terminal push starts when a
            # certificate exists, and the progress watch re-plans if it never
            # does. Never push straight at a surface point through whatever
            # lies between.
            return est.x, est.y
        # No route to any goal in the current map: head for the start area,
        # which keeps the drone moving and mapping until a route appears.
        return self._path_target(obs, self.home) if self.home else (est.x, est.y)
