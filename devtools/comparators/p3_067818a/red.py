"""Red (place) brain, candidate R2 (R1 in Phase 2, R0 and R0.1 in Phase 1).

Schedule (fractions of the step limit T; the time score is full when the map
is submitted by 0.5 T):

* 0 - 0.25 T, explore: frontier/search-target exploration with team
  spreading; every validated lidar return within ``wallmap.MAP_RANGE`` adds
  wall evidence;
* 0.25 T - 0.40 T, place: each drone with inventory picks the reachable
  free spot with the highest marginal blast coverage of the walls on its
  current team map (border walls weigh 0.5; in R2, unknown regions of its
  occupancy grid enclosed by obstacles weigh as solid wall, because solid
  obstacles count with their whole area), net of travel cost and with a
  mild preference for spots far from the disposal center; spots are claimed
  over the radio and the claims are relayed with their age (R2), the lower
  identifier keeps a contested area, and a target is given up as soon as a
  known placed bomb lies within the overlap distance of it (R2); after
  placing, the drone resumes exploring;
* 0.40 T onwards, rally: everybody returns towards the start area so that
  the team's evidence snapshots reach the leader (the lowest identifier
  heard from recently); the leader submits once it holds a fresh snapshot of
  every teammate it knows, and in any case at 0.495 T;
* safety: an unplaced bomb is dropped where the drone is shortly before the
  rally; any drone submits if the wall-clock limit comes close (the leader at
  80 %, the others at 88 %), because a round without a submitted map scores
  nothing for the map.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from . import params as P
from .base import BrainBase
from .comms import make_message, numeric_tuple, valid_messages
from .motion import LocalView, drive
from .observation import Command, Observation
from .perception import bomb_detections, entity_mask
from .wallmap import WallEvidence, merge, render, wall_weights

T_PLACE = 0.25
T_RALLY = 0.40
T_SUBMIT = 0.47
T_DEADLINE = 0.495
SNAPSHOT_PERIOD = 50
PLACE_DECISION_PERIOD = 60
COVER_SCALE = 4
TRAVEL_WEIGHT = 0.3
FAR_BONUS = 0.12
CLAIM_TTL = 80             # steps a (possibly relayed) placement claim stays valid
OVERLAP = 0.8              # targets closer than this many blast radii overlap
SOLID_MAX_AREA = 70000.0   # px^2; larger enclosed unknown regions are not taken as solid


class PlaceBrain(BrainBase):
    SEARCH_RANGE = 130.0
    FREE_RANGE = 180.0
    SHARE_COVERAGE = True
    COMMIT_TARGETS = True
    COVERAGE_DROPS_FRONTIER = False    # red maps walls: unmapped space stays a target

    def __init__(self, identifier: int, n_drones: int, size_area: Tuple[int, int],
                 max_steps: Optional[int], max_walltime: Optional[float]) -> None:
        super().__init__(identifier, n_drones, size_area, max_steps)
        self.max_walltime = float(max_walltime) if max_walltime else 1e9
        self.evidence = WallEvidence(self.size)
        self.snapshots: Dict[int, Tuple[int, bytes]] = {}
        self.own_snapshot: Optional[Tuple[int, bytes]] = None
        self.relay_index = 0
        self.placed: List[Tuple[float, float]] = []
        self.place_claims: Dict[int, Tuple[float, float, int]] = {}
        self.place_target: Optional[Tuple[float, float]] = None
        self.place_decided = -10_000
        self.place_tries = 0
        self.prev_place_cmd = 0
        self.prev_inventory: Optional[int] = None
        self.phase = "explore"
        self.blast_radius = round(0.12 * min(self.size))
        T = self.max_steps
        self.t_place = int(T_PLACE * T)
        self.t_rally = int(T_RALLY * T)
        self.t_submit = int(T_SUBMIT * T)
        self.t_deadline = int(T_DEADLINE * T)
        self.submitted = False
        self.final_map: Optional[np.ndarray] = None
        self.diag.update({"placed": 0, "place_retries": 0, "emergency_drops": 0,
                          "snapshots_held": 0, "submit_reason": "", "targets_dropped": 0,
                          "solid_cells": 0})

    # ------------------------------------------------------------------ API
    def step(self, obs: Observation) -> Command:
        if not self._begin(obs):
            return Command()
        now = obs.step
        self._read_messages(obs)
        self._perceive(obs)
        self._coverage_tick(now)
        self._track_inventory(obs)
        cmd = self._decide(obs)
        if not self.submitted and self._should_submit(obs):
            self.final_map = self._build_map()
            self.submitted = True
            cmd.submit_map = True
        self.prev_place_cmd = cmd.place_bomb
        self._finish(obs, cmd)
        if (now + 7 * self.id) % SNAPSHOT_PERIOD == 0 or now >= self.t_rally and now % 10 == 0:
            self.own_snapshot = (now, self.evidence.snapshot())
        self._compose_message(obs)
        return cmd

    def exploration_map(self) -> Optional[np.ndarray]:
        if self.final_map is None:
            self.final_map = self._build_map()
        return self.final_map

    def diagnostics(self) -> Dict[str, Any]:
        out = dict(self.diag)
        out.update({"phase": self.phase, "evidence_hits": self.evidence.total,
                    "snapshots_held": len(self.snapshots), "known_placed": len(self.placed),
                    "est_disturbances": self.est.disturbance_count,
                    "team_coverage": round(self.coverage.fraction, 3),
                    "coverage_merged": self.coverage.merged_messages,
                    "coverage_rejected": self.coverage.rejected_messages})
        return out

    # ------------------------------------------------------------- messages
    def _read_messages(self, obs: Observation) -> None:
        now = obs.step
        for msg in valid_messages(obs.messages, self.id):
            sender = msg["id"]
            self.team.update(msg, now)
            self._read_fleet(msg)
            self._read_coverage(msg)
            for item in msg.get("pl", ()):
                if isinstance(item, tuple) and len(item) == 2:
                    self._add_placed(float(item[0]), float(item[1]))
            # Claims carry the step at which their owner made them, so a
            # relayed copy never outlives the owner's newer statement.
            sent = int(msg["t"])
            claim = msg.get("pc")
            if isinstance(claim, tuple) and len(claim) == 2:
                self._merge_claim(sender, float(claim[0]), float(claim[1]), sent)
            elif sender in self.place_claims and self.place_claims[sender][2] <= sent:
                del self.place_claims[sender]
            for item in msg.get("pr", ()):
                if numeric_tuple(item, 4) and int(item[0]) not in (self.id, sender):
                    self._merge_claim(int(item[0]), float(item[1]), float(item[2]), int(item[3]))
            snap = msg.get("sn")
            if isinstance(snap, tuple) and len(snap) == 3 and isinstance(snap[2], bytes):
                source, step = int(snap[0]), int(snap[1])
                if source != self.id and (source not in self.snapshots
                                          or self.snapshots[source][0] < step):
                    self.snapshots[source] = (step, snap[2])
            box = msg.get("dc")
            if isinstance(box, tuple) and len(box) == 4 and self.dc.box is None:
                self.dc.shared_box = tuple(float(v) for v in box)

    def _compose_message(self, obs: Observation) -> None:
        now = obs.step
        snap = None
        held = [(k, v) for k, v in self.snapshots.items()]
        if self.own_snapshot is not None and (not held or now % 2 == 0):
            snap = (self.id, self.own_snapshot[0], self.own_snapshot[1])
        elif held:
            self.relay_index = (self.relay_index + 1) % len(held)
            k, (step, blob) = held[self.relay_index]
            snap = (k, step, blob)
        box = self.dc.box
        self.msg_out = make_message(
            self.id, now, (self.est.x, self.est.y), r="R", s=self.phase,
            tg=(round(self.target[0], 1), round(self.target[1], 1)) if self.target else None,
            tp=self._fleet_digest(now, False),
            pl=tuple((round(x, 1), round(y, 1)) for x, y in self.placed[-30:]),
            pc=(round(self.place_target[0], 1), round(self.place_target[1], 1))
            if self.place_target else None,
            pr=tuple((k, round(v[0], 1), round(v[1], 1), v[2])
                     for k, v in self.place_claims.items() if now - v[2] < CLAIM_TTL),
            sn=snap, inv=int(obs.inventory),
            dc=tuple(round(v, 1) for v in box) if box else None,
            cv=self._coverage_message(now))

    # ------------------------------------------------------------ perception
    def _merge_claim(self, owner: int, x: float, y: float, step: int) -> None:
        known = self.place_claims.get(owner)
        if known is None or known[2] < step:
            self.place_claims[owner] = (x, y, step)

    def _add_placed(self, x: float, y: float) -> None:
        if all(math.hypot(x - a, y - b) > 30.0 for a, b in self.placed):
            self.placed.append((x, y))

    def _perceive(self, obs: Observation) -> None:
        est = self.est
        self.dc.observe(est.x, est.y, est.heading, obs.semantic)
        if obs.semantic is not None:
            for det in bomb_detections(est.x, est.y, est.heading, obs.semantic, obs.lidar,
                                       obs.lidar_angles):
                self._add_placed(det.x, det.y)
        if obs.lidar is None or (obs.step + self.id) % P.SCAN_PERIOD != 0:
            return
        if est.steps_without_gps >= 40:
            return
        _, _, transient, center = self._lidar_mask(obs, self.placed)
        self._integrate(obs, transient)
        if est.position_sigma <= 8.0:
            # Wall evidence excludes the disposal center as well.
            self.evidence.add_returns(est.x, est.y, est.heading, obs.lidar, obs.lidar_angles,
                                      ~(transient | center) & (obs.lidar < P.LIDAR_NO_HIT))

    def _track_inventory(self, obs: Observation) -> None:
        if self.prev_inventory is not None and obs.inventory < self.prev_inventory:
            self._add_placed(self.est.x, self.est.y)
            self.diag["placed"] += 1
            self.place_target = None
            self.place_tries = 0
        self.prev_inventory = obs.inventory

    # -------------------------------------------------------------- decision
    def _leader(self, now: int) -> int:
        alive = [k for k, v in self.fleet.items() if now - v[2] < 300]
        return min([self.id] + alive)

    def _should_submit(self, obs: Observation) -> bool:
        now = obs.step
        leader = self._leader(now) == self.id
        frac = obs.walltime / self.max_walltime if self.max_walltime else 0.0
        if frac >= (0.80 if leader else 0.88):
            self.diag["submit_reason"] = f"walltime {frac:.2f}"
            return True
        if not leader:
            return False
        if now >= self.t_deadline:
            self.diag["submit_reason"] = "deadline"
            return True
        if now >= self.t_submit:
            known = [k for k, v in self.fleet.items() if now - v[2] < 300]
            fresh = all(k in self.snapshots and self.snapshots[k][0] >= self.t_rally - 40
                        for k in known)
            if fresh:
                self.diag["submit_reason"] = "all snapshots fresh"
                return True
        return False

    def _decide(self, obs: Observation) -> Command:
        est = self.est
        now = obs.step
        valid = None
        if obs.lidar is not None:
            valid = ~entity_mask(obs.semantic, obs.lidar, obs.lidar_angles)
        view = LocalView(est.heading, obs.lidar, obs.lidar_angles, valid=valid)
        others = self._others(obs)
        if now < self.recover_until:
            return self._recover_command(obs, view, others)
        if obs.inventory > 0 and now >= self.t_rally - int(0.03 * self.max_steps):
            cmd = self._emergency_place(obs, view, others)
        elif obs.inventory > 0 and now >= self.t_place:
            self.phase = "place"
            cmd = self._place(obs, view, others)
        elif now >= self.t_rally:
            self.phase = "rally"
            cmd = self._rally(obs, view, others)
        else:
            self.phase = "explore"
            cmd = self._explore(obs, view, others)
        self._check_stuck(obs, view, cmd, block_target=self.phase == "explore")
        return cmd

    def _explore(self, obs: Observation, view: LocalView, others) -> Command:
        now = obs.step
        if self._explore_due(now, 40):
            if self.target is not None and self.state == "explore":
                self.diag["target_switches"] += 1
            self.target = self._choose_explore_target(now)
            self.goal_step = now
            self.path = []
        self.state = "explore"
        if self.target is None:
            return self._rally(obs, view, others)
        return self._goto(obs, view, others, self.target, cruise=P.MAX_SPEED * 0.8)

    def _rally(self, obs: Observation, view: LocalView, others) -> Command:
        self.state = "rally"
        spot = self.home or (self.est.x, self.est.y)
        aim = (spot[0] + 50.0 * math.cos(2.4 * self.id), spot[1] + 50.0 * math.sin(2.4 * self.id))
        self.target = aim
        return self._goto(obs, view, others, aim, cruise=P.MAX_SPEED * 0.8, stop=True)

    # -- placement ----------------------------------------------------------
    def _place(self, obs: Observation, view: LocalView, others) -> Command:
        est = self.est
        now = obs.step
        if self.place_target is not None:
            tx, ty = self.place_target
            near = OVERLAP * self.blast_radius
            # A bomb placed meanwhile next to the target makes it redundant
            # (gossiped placements arrive after the target was chosen).
            if any(math.hypot(bx - tx, by - ty) < near for bx, by in self.placed):
                self.diag["targets_dropped"] += 1
                self.place_target = None
        if self.place_target is not None:
            for other, (cx, cy, step) in self.place_claims.items():
                if other < self.id and now - step < CLAIM_TTL and \
                        math.hypot(cx - self.place_target[0], cy - self.place_target[1]) \
                        < OVERLAP * self.blast_radius:
                    self.place_target = None
                    break
        if now < self.t_place + 15 * self.id:
            # Staggered first decisions, so that earlier claims are known.
            return self._explore(obs, view, others)
        if self.place_target is None or now - self.place_decided > 4 * PLACE_DECISION_PERIOD:
            self.place_target = self._choose_place_target(now)
            self.place_decided = now
            self.place_tries = 0
        if self.place_target is None:
            return self._explore(obs, view, others)
        self.state = "place"
        self.target = self.place_target
        dist = math.hypot(self.place_target[0] - est.x, self.place_target[1] - est.y)
        cmd = self._goto(obs, view, others, self.place_target, cruise=P.MAX_SPEED * 0.8,
                         stop=True)
        if dist < 10.0 and self.prev_place_cmd == 0:
            cmd.place_bomb = 1
            self.place_tries += 1
            if self.place_tries > 3:
                # Repeated failure (separation or a stale target): shift.
                self.diag["place_retries"] += 1
                ang = 2.4 * self.place_tries
                self.place_target = (self.place_target[0] + 30.0 * math.cos(ang),
                                     self.place_target[1] + 30.0 * math.sin(ang))
        return cmd

    def _emergency_place(self, obs: Observation, view: LocalView, others) -> Command:
        self.phase = "place"
        est = self.est
        cmd = self._rally(obs, view, others)
        clear = all(math.hypot(est.x - a, est.y - b) > 30.0 for a, b in self.placed)
        if clear and self.prev_place_cmd == 0 and self.dc.distance(est.x, est.y) > 30.0:
            cmd.place_bomb = 1
            self.diag["emergency_drops"] += 1
        return cmd

    def _choose_place_target(self, now: int) -> Optional[Tuple[float, float]]:
        est = self.est
        g = self.grid
        wall_map = render(merge(self.evidence, (v[1] for v in self.snapshots.values())),
                          dc_box=self.dc.bbox())
        s = COVER_SCALE
        weights = wall_weights(wall_map, s)
        # Solid obstacles count with their whole area in the wall score, but
        # the lidar only sees their outline: enclosed unknown regions of the
        # occupancy grid weigh as solid wall here (placement only; the
        # submitted map is unchanged).
        solid = self._solid_interiors()
        if solid.any():
            k = int(self.grid.cell) // s
            up = np.kron(solid.astype(np.float32), np.ones((k, k), np.float32))
            up = up[:weights.shape[0], :weights.shape[1]] * float(s * s)
            weights[:up.shape[0], :up.shape[1]] = np.maximum(
                weights[:up.shape[0], :up.shape[1]], up)
            self.diag["solid_cells"] = int(solid.sum())
        r_small = self.blast_radius / s
        covered = np.zeros_like(weights, dtype=np.uint8)
        taken = list(self.placed) + [(v[0], v[1]) for k, v in self.place_claims.items()
                                     if now - v[2] < CLAIM_TTL]
        for bx, by in taken:
            cx = int((bx + self.size[0] / 2.0) / s)
            cy = int((self.size[1] / 2.0 - by) / s)
            cv2.circle(covered, (cx, cy), int(round(r_small)), 1, -1)
        weights[covered > 0] = 0.0
        k = int(math.ceil(r_small))
        disk = np.zeros((2 * k + 1, 2 * k + 1), np.float32)
        cv2.circle(disk, (k, k), int(round(r_small)), 1.0, -1)
        coverage = cv2.filter2D(weights, -1, disk, borderType=cv2.BORDER_CONSTANT)
        # Candidate centres: free coarse cells away from obstacles and the center.
        free = g.free()
        occ = g.occupied().astype(np.uint8)
        near = cv2.dilate(occ, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))) > 0
        cand = free & ~near
        field = self._field_from_me(now)
        cand &= np.isfinite(field)
        rows, cols = np.nonzero(cand)
        if rows.size == 0:
            return None
        wx = (cols + 0.5) * g.cell - g.width / 2.0
        wy = g.height / 2.0 - (rows + 0.5) * g.cell
        ok = np.ones(rows.size, dtype=bool)
        for bx, by in taken:
            # Blasts closer than one radius mostly overlap.
            ok &= np.hypot(wx - bx, wy - by) > self.blast_radius
        box = self.dc.bbox()
        if box is not None:
            dx = np.maximum(np.maximum(box[0] - wx, 0.0), wx - box[2])
            dy = np.maximum(np.maximum(box[1] - wy, 0.0), wy - box[3])
            dc_dist = np.hypot(dx, dy)
            ok &= dc_dist > 40.0
        else:
            dc_dist = np.hypot(wx - (self.home or (0.0, 0.0))[0], wy - (self.home or (0.0, 0.0))[1])
        if not ok.any():
            return None
        ci = np.clip(((wx + self.size[0] / 2.0) / s).astype(int), 0, coverage.shape[1] - 1)
        ri = np.clip(((self.size[1] / 2.0 - wy) / s).astype(int), 0, coverage.shape[0] - 1)
        cov = coverage[ri, ci]
        best_cov = float(cov.max()) if cov.size else 0.0
        if best_cov <= 0.0:
            return None
        score = cov - TRAVEL_WEIGHT * field[rows, cols] * g.cell \
            + FAR_BONUS * best_cov * np.minimum(dc_dist, 600.0) / 600.0
        score[~ok] = -np.inf
        i = int(np.argmax(score))
        if not np.isfinite(score[i]):
            return None
        return float(wx[i]), float(wy[i])

    def _solid_interiors(self) -> np.ndarray:
        """Unknown grid cells enclosed by occupied cells (interiors of solid
        obstacles, which the lidar cannot see into).

        A 4-connected region of unknown cells counts when none of its cells
        is 4-adjacent to a free cell or lies on the grid edge (so an outline
        with a gap, or unexplored space reachable from mapped free space, is
        never filled) and its area is at most ``SOLID_MAX_AREA``. Regions
        overlapping the observed disposal center (an obstacle for navigation,
        mapped from outside like a box, but not a wall) are excluded.
        """
        g = self.grid
        unknown = g.unknown().astype(np.uint8)
        out = np.zeros(unknown.shape, dtype=bool)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(unknown, connectivity=4)
        if n <= 1:
            return out
        cross = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], np.uint8)
        near_free = cv2.dilate(g.free().astype(np.uint8), cross) > 0
        open_labels = set(np.unique(labels[near_free & (unknown > 0)]).tolist())
        for edge in (labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]):
            open_labels.update(np.unique(edge).tolist())
        box = self.dc.bbox()
        if box is not None:
            r0, c0 = g.to_cell(box[0] - 16.0, box[3] + 16.0)
            r1, c1 = g.to_cell(box[2] + 16.0, box[1] - 16.0)
            open_labels.update(np.unique(labels[r0:r1 + 1, c0:c1 + 1]).tolist())
        max_cells = SOLID_MAX_AREA / (g.cell * g.cell)
        keep = [i for i in range(1, n) if i not in open_labels
                and stats[i, cv2.CC_STAT_AREA] <= max_cells]
        if keep:
            out = np.isin(labels, keep)
        return out

    # -- map ------------------------------------------------------------------
    def _build_map(self) -> np.ndarray:
        total = merge(self.evidence, (v[1] for v in self.snapshots.values()))
        self.diag["snapshots_held"] = len(self.snapshots)
        return render(total, dc_box=self.dc.bbox())
