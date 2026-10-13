"""GL-free kinematic approximation of the competition simulator.

Purpose: fast, deterministic logic tests of agent brains. It reproduces
what was measured or read from the supplied simulator:

* motion: the exact linear thrust/rotation model (``eod.params``), the
  simulator's thrust clipping, carrying with a third of the thrust gain and a
  speed cap, the reduced rotation response while carrying (0.39 of the
  command, the median measured in the grasp probe), contact with walls and
  the disposal center that stops motion along the contact normal;
* sensors: GPS and compass AR(1) noise (sigma 5 px / 4 deg, phi 0.98),
  odometer Gaussian noise (0.2 px, 8 deg, 1 deg), 181-ray lidar over walls,
  the disposal center, drones and bombs (sigma 2.5 px, 300 px range),
  35-ray semantic sensor over drones, bombs and the disposal center
  (200 px, distance + N(2.5, 1));
* mechanics: grasp within 29 px with the grasper on, carried bomb 10.6 px
  ahead on the grasp side, disposal when a carried bomb touches the center,
  release drift towards the spawn point, red placement rules (rising edge,
  24 px separation, inventory, no placement while grasping), one-step
  message latency within 250 px, health loss on contact with a cooldown.

It is an approximation (no rigid-body dynamics, circular bodies, simplified
contact), so results from it are development signals only and are never
reported as competition performance. Ground truth used here comes from the
walls-only fixtures and stays in development tooling.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence

import cv2
import numpy as np

from swarm_rescue.solutions.eod import params as P
from swarm_rescue.solutions.eod.observation import Command, Observation, SemanticHit

AR_PHI = 0.98
HEALTH_COOLDOWN_STEPS = 150
CARRY_ROTATION_RATIO = 0.39
DRONE_BODY = 13.0
SEMANTIC_RAYS = 35


@dataclass
class SimDrone:
    ident: int
    x: float
    y: float
    heading: float
    vx: float = 0.0
    vy: float = 0.0
    health: int = 50
    removed: bool = False
    carrying: Optional[int] = None
    grasp_angle: float = 0.0
    inventory: int = 0
    prev_place: int = 0
    last_hit_step: int = -10_000
    gps_noise: np.ndarray = field(default_factory=lambda: np.zeros(2))
    compass_noise: float = 0.0
    prev_x: float = 0.0
    prev_y: float = 0.0
    prev_heading: float = 0.0
    grasper_on: bool = False
    reward: float = 0.0


@dataclass
class SimBomb:
    x: float
    y: float
    origin_x: float
    origin_y: float
    holder: Optional[int] = None
    removed: bool = False


class OfflineWorld:
    def __init__(self, fixture: dict, mode: str, n_drones: int,
                 brain_factory: Callable[[int], Any], *, seed: int = 0,
                 bombs: Sequence[Sequence[float]] = (), bombs_per_drone: int = 2,
                 max_steps: Optional[int] = None,
                 gps_off: Optional[Callable[[int, int], bool]] = None) -> None:
        self.rng = np.random.default_rng(seed)
        self.mode = mode
        self.fixture = fixture
        self.walls = fixture["walls"].astype(np.uint8)
        self.h, self.w = self.walls.shape
        dc = fixture["disposal_center"]
        self.dc_box = (dc["x"] - dc["width"] / 2, dc["y"] - dc["height"] / 2,
                       dc["x"] + dc["width"] / 2, dc["y"] + dc["height"] / 2)
        solid = self.walls.copy()
        r0, c0 = self._px(self.dc_box[0], self.dc_box[3])
        r1, c1 = self._px(self.dc_box[2], self.dc_box[1])
        solid[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1] = 1
        self.solid = solid
        self.dist = cv2.distanceTransform((1 - solid).astype(np.uint8), cv2.DIST_L2,
                                          cv2.DIST_MASK_PRECISE)
        self.max_steps = max_steps or int(fixture["max_timestep_limit"])
        self.n = n_drones
        self.drones = [self._spawn(i) for i in range(n_drones)]
        self.bombs = [SimBomb(float(x), float(y), float(x), float(y)) for x, y in bombs]
        if mode == "place":
            for d in self.drones:
                d.inventory = bombs_per_drone
        self.brains = [brain_factory(i) for i in range(n_drones)]
        self.inbox: List[List[Any]] = [[] for _ in range(n_drones)]
        self.step_index = 1
        self.disposed = 0
        self.submitted: Optional[np.ndarray] = None
        self.submit_step: Optional[int] = None
        self.gps_off = gps_off
        self.lidar_angles = np.linspace(-math.pi, math.pi, P.LIDAR_RAYS)
        self.sem_angles = np.linspace(-math.pi, math.pi, SEMANTIC_RAYS)
        self.contacts = 0
        self.health_lost = 0

    # ------------------------------------------------------------------ setup
    def _spawn(self, i: int) -> SimDrone:
        sx, sy = self.fixture["start_area"]
        side = math.ceil(math.sqrt(self.n))
        x = sx - (side - 1) * 20.0 + (i % side) * 40.0
        y = sy - (side - 1) * 20.0 + (i // side) * 40.0
        heading = float(self.rng.uniform(-math.pi, math.pi))
        return SimDrone(i, x, y, heading, prev_x=x, prev_y=y, prev_heading=heading)

    def _px(self, x: float, y: float):
        return int(round(self.h / 2.0 - y)), int(round(x + self.w / 2.0))

    def _clear(self, x: float, y: float) -> float:
        r, c = self._px(x, y)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 0.0
        return float(self.dist[r, c])

    # ---------------------------------------------------------------- sensors
    def _raycast(self, x: float, y: float, angles: np.ndarray, max_range: float,
                 circles: List[tuple]) -> tuple:
        """Distance to the first solid pixel or circle; returns (dist, kind)."""
        n = len(angles)
        t = np.zeros(n)
        cos_a, sin_a = np.cos(angles), np.sin(angles)
        active = np.ones(n, dtype=bool)
        for _ in range(60):
            px = x + t * cos_a
            py = y + t * sin_a
            rows = np.round(self.h / 2.0 - py).astype(int)
            cols = np.round(px + self.w / 2.0).astype(int)
            inside = (rows >= 0) & (rows < self.h) & (cols >= 0) & (cols < self.w)
            d = np.zeros(n)
            d[inside] = self.dist[rows[inside], cols[inside]]
            hit = active & ((d < 0.75) | ~inside)
            active &= ~hit
            t = np.where(active, t + np.maximum(d - 0.5, 0.75), t)
            active &= t < max_range
            if not active.any():
                break
        t = np.minimum(t, max_range)
        kind = np.where(t < max_range, 0, -1)  # 0: wall/center, -1: nothing
        for idx, (cx, cy, rad) in enumerate(circles):
            ox, oy = cx - x, cy - y
            proj = ox * cos_a + oy * sin_a
            perp2 = ox * ox + oy * oy - proj * proj
            ok = (proj > 0) & (perp2 < rad * rad)
            dist_c = proj - np.sqrt(np.maximum(rad * rad - perp2, 0.0))
            better = ok & (dist_c < t) & (dist_c >= 0)
            t = np.where(better, dist_c, t)
            kind = np.where(better, idx + 1, kind)
        return t, kind

    def _inside_dc(self, x: float, y: float, margin: float = 0.0) -> bool:
        x0, y0, x1, y1 = self.dc_box
        return x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin

    def observe(self, d: SimDrone) -> Observation:
        rng = self.rng
        # GPS / compass AR(1) noise evolves every step.
        wn = P.GPS_NOISE * math.sqrt(1 - AR_PHI ** 2)
        d.gps_noise = AR_PHI * d.gps_noise + rng.normal(0, wn, 2)
        wc = P.COMPASS_NOISE * math.sqrt(1 - AR_PHI ** 2)
        d.compass_noise = AR_PHI * d.compass_noise + rng.normal(0, wc)
        gps_disabled = self.gps_off is not None and self.gps_off(d.ident, self.step_index)
        gps = None if gps_disabled else (d.x + d.gps_noise[0], d.y + d.gps_noise[1])
        compass = math.atan2(math.sin(d.heading + d.compass_noise),
                             math.cos(d.heading + d.compass_noise))
        dx, dy = d.x - d.prev_x, d.y - d.prev_y
        dist_travel = math.hypot(dx, dy) + rng.normal(0, 0.2)
        alpha = math.atan2(dy, dx) - d.prev_heading + rng.normal(0, math.radians(8))
        dtheta = d.heading - d.prev_heading + rng.normal(0, math.radians(1))
        odometer = (dist_travel, math.atan2(math.sin(alpha), math.cos(alpha)),
                    math.atan2(math.sin(dtheta), math.cos(dtheta)))
        circles, kinds, grasped_flags = [], [], []
        for other in self.drones:
            if other.ident != d.ident and not other.removed:
                circles.append((other.x, other.y, DRONE_BODY))
                kinds.append("DRONE")
                grasped_flags.append(False)
        for bi, b in enumerate(self.bombs):
            if not b.removed and d.carrying != bi:
                circles.append((b.x, b.y, P.BOMB_RADIUS))
                kinds.append("BOMB")
                grasped_flags.append(b.holder is not None)
        ang = d.heading + self.lidar_angles
        dist, _ = self._raycast(d.x, d.y, ang, P.LIDAR_RANGE, circles)
        lidar = dist + rng.normal(0, P.LIDAR_NOISE, len(dist))
        sem_dist, sem_kind = self._raycast(d.x, d.y, d.heading + self.sem_angles,
                                           P.SEMANTIC_RANGE, circles)
        semantic = []
        for i in range(SEMANTIC_RAYS):
            if sem_dist[i] >= P.SEMANTIC_RANGE:
                continue
            k = int(sem_kind[i])
            if k > 0:
                kind = kinds[k - 1]
                grasped = grasped_flags[k - 1]
            else:
                hx = d.x + sem_dist[i] * math.cos(d.heading + self.sem_angles[i])
                hy = d.y + sem_dist[i] * math.sin(d.heading + self.sem_angles[i])
                if not self._inside_dc(hx, hy, 2.0):
                    continue
                kind, grasped = "DISPOSAL_CENTER", False
            semantic.append(SemanticHit(float(sem_dist[i] + rng.normal(2.5, 1.0)),
                                        float(self.sem_angles[i]), kind, grasped))
        return Observation(
            step=self.step_index, walltime=self.step_index / 150.0, identifier=d.ident,
            n_drones=self.n, size_area=(self.w, self.h), max_steps=self.max_steps,
            max_walltime=float(self.fixture["max_walltime_limit"]), gps=gps,
            compass=compass, odometer=odometer, lidar=lidar,
            lidar_angles=self.lidar_angles, semantic=semantic,
            grasping=d.carrying is not None, health=d.health,
            inventory=d.inventory, messages=list(self.inbox[d.ident]))

    # ---------------------------------------------------------------- physics
    def _move(self, d: SimDrone, cmd: Command) -> None:
        f = max(-1.0, min(1.0, float(cmd.forward)))
        l = max(-1.0, min(1.0, float(cmd.lateral)))
        norm = math.hypot(f, l)
        if norm > 1.0:
            f, l = f / norm, l / norm
        r = max(-1.0, min(1.0, float(cmd.rotation)))
        c, s = math.cos(d.heading), math.sin(d.heading)
        ux, uy = f * c - l * s, f * s + l * c
        gain = P.CARRY_GAIN_FACTOR if d.carrying is not None else 1.0
        nx = d.x + P.POS_VEL * d.vx + P.POS_GAIN * gain * ux
        ny = d.y + P.POS_VEL * d.vy + P.POS_GAIN * gain * uy
        d.vx = P.VEL_DECAY * d.vx + P.VEL_GAIN * gain * ux
        d.vy = P.VEL_DECAY * d.vy + P.VEL_GAIN * gain * uy
        if d.carrying is not None:
            sp = math.hypot(d.vx, d.vy)
            if sp > P.CARRY_SPEED + 0.1:
                d.vx *= (P.CARRY_SPEED + 0.1) / sp
                d.vy *= (P.CARRY_SPEED + 0.1) / sp
        d.prev_x, d.prev_y, d.prev_heading = d.x, d.y, d.heading
        # Measured in the grasp probe: with a bomb on the pivot joint the drone
        # turns by only about 0.39 of the commanded rotation (median ratio).
        rot_gain = P.ROT_GAIN * (CARRY_ROTATION_RATIO if d.carrying is not None else 1.0)
        d.heading = math.atan2(math.sin(d.heading + rot_gain * r),
                               math.cos(d.heading + rot_gain * r))
        if self._clear(nx, ny) < DRONE_BODY:
            # Contact: move as far as possible, then cancel the normal speed.
            lo, hi = 0.0, 1.0
            for _ in range(8):
                mid = (lo + hi) / 2
                if self._clear(d.x + mid * (nx - d.x), d.y + mid * (ny - d.y)) >= DRONE_BODY:
                    lo = mid
                else:
                    hi = mid
            nx, ny = d.x + lo * (nx - d.x), d.y + lo * (ny - d.y)
            gx = self._clear(nx + 1, ny) - self._clear(nx - 1, ny)
            gy = self._clear(nx, ny + 1) - self._clear(nx, ny - 1)
            gn = math.hypot(gx, gy)
            if gn > 0:
                gx, gy = gx / gn, gy / gn
                vn = d.vx * gx + d.vy * gy
                if vn < 0:
                    d.vx -= vn * gx
                    d.vy -= vn * gy
            self._damage(d)
        d.x, d.y = nx, ny

    def _damage(self, d: SimDrone) -> None:
        self.contacts += 1
        if self.step_index - d.last_hit_step >= HEALTH_COOLDOWN_STEPS:
            d.health -= 1
            self.health_lost += 1
            d.last_hit_step = self.step_index
            if d.health <= 0:
                d.removed = True
                if d.carrying is not None:
                    self.bombs[d.carrying].holder = None
                    d.carrying = None

    def _drone_contacts(self) -> None:
        alive = [d for d in self.drones if not d.removed]
        for i, a in enumerate(alive):
            for b in alive[i + 1:]:
                dx, dy = b.x - a.x, b.y - a.y
                dist = math.hypot(dx, dy)
                if dist < 2 * DRONE_BODY and dist > 1e-6:
                    push = (2 * DRONE_BODY - dist) / 2
                    ux, uy = dx / dist, dy / dist
                    a.x -= push * ux
                    a.y -= push * uy
                    b.x += push * ux
                    b.y += push * uy
                    self._damage(a)
                    self._damage(b)
            if self.mode == "rescue":
                for bomb in self.bombs:
                    if bomb.removed or bomb.holder is not None:
                        continue
                    dx, dy = bomb.x - a.x, bomb.y - a.y
                    dist = math.hypot(dx, dy)
                    min_d = DRONE_BODY + P.BOMB_RADIUS
                    if 1e-6 < dist < min_d:
                        push = min_d - dist
                        # Bombs are three times heavier: the drone yields most.
                        a.x -= 0.75 * push * dx / dist
                        a.y -= 0.75 * push * dy / dist
                        bomb.x += 0.25 * push * dx / dist
                        bomb.y += 0.25 * push * dy / dist

    def _mechanics(self, d: SimDrone, cmd: Command) -> None:
        d.grasper_on = bool(cmd.grasper)
        if d.carrying is not None:
            bomb = self.bombs[d.carrying]
            if not cmd.grasper:
                bomb.holder = None
                d.carrying = None
            else:
                a = d.heading + d.grasp_angle
                bomb.x = d.x + P.CARRY_OFFSET * math.cos(a)
                bomb.y = d.y + P.CARRY_OFFSET * math.sin(a)
                x0, y0, x1, y1 = self.dc_box
                cx = min(max(bomb.x, x0), x1)
                cy = min(max(bomb.y, y0), y1)
                if math.hypot(bomb.x - cx, bomb.y - cy) <= P.BOMB_RADIUS + 1.0:
                    bomb.removed = True
                    bomb.holder = None
                    d.carrying = None
                    d.reward += 1
                    self.disposed += 1
        elif cmd.grasper and self.mode == "rescue":
            for bi, bomb in enumerate(self.bombs):
                if bomb.removed or bomb.holder is not None:
                    continue
                if math.hypot(bomb.x - d.x, bomb.y - d.y) <= P.GRASP_REACH:
                    d.carrying = bi
                    bomb.holder = d.ident
                    d.grasp_angle = math.atan2(bomb.y - d.y, bomb.x - d.x) - d.heading
                    break
        if self.mode == "place":
            rising = cmd.place_bomb == 1 and d.prev_place == 0
            d.prev_place = int(cmd.place_bomb)
            if rising and d.inventory > 0 and not cmd.grasper and d.carrying is None:
                if all(math.hypot(b.x - d.x, b.y - d.y) >= P.MIN_BOMB_SEPARATION
                       for b in self.bombs):
                    self.bombs.append(SimBomb(d.x, d.y, d.x, d.y))
                    d.inventory -= 1

    def _drift(self) -> None:
        for bomb in self.bombs:
            if bomb.removed or bomb.holder is not None:
                continue
            dx, dy = bomb.origin_x - bomb.x, bomb.origin_y - bomb.y
            dist = math.hypot(dx, dy)
            if dist > 10.0:
                bomb.x += 0.17 * dx / dist
                bomb.y += 0.17 * dy / dist

    # ---------------------------------------------------------------- loop
    def step(self) -> bool:
        outgoing = [brain.outgoing_message() for brain in self.brains]
        commands = []
        for d, brain in zip(self.drones, self.brains):
            obs = self.observe(d)
            cmd = brain.step(obs)
            if cmd.submit_map and self.submitted is None and self.mode == "place":
                self.submitted = brain.exploration_map()
                self.submit_step = self.step_index
            commands.append(cmd)
        if self.submitted is not None:
            return False
        for d, cmd in zip(self.drones, commands):
            if d.removed:
                continue
            self._move(d, cmd)
            self._mechanics(d, cmd)
        self._drone_contacts()
        self._drift()
        self.inbox = [[] for _ in range(self.n)]
        for sender, msg in zip(self.drones, outgoing):
            if msg is None or sender.removed:
                continue
            for rec in self.drones:
                if rec.ident != sender.ident and not rec.removed and \
                        math.hypot(rec.x - sender.x, rec.y - sender.y) < P.COMM_RANGE:
                    self.inbox[rec.ident].append(msg)
        self.step_index += 1
        if self.mode == "rescue" and self.bombs and \
                all(b.removed for b in self.bombs):
            return False
        return self.step_index <= self.max_steps

    def run(self) -> Dict[str, Any]:
        while self.step():
            pass
        return self.summary()

    def summary(self) -> Dict[str, Any]:
        health = sum(d.health if not d.removed else 0 for d in self.drones)
        out = {"steps": self.step_index - 1, "disposed": self.disposed,
               "bombs": len(self.bombs), "health_total": health,
               "health_lost": self.health_lost, "contacts": self.contacts,
               "removed": sum(d.removed for d in self.drones)}
        if self.mode == "rescue":
            limit = self.max_steps
            t = self.step_index - 1
            time_score = max(0.0, min(1.0, (limit - t) / (0.5 * limit))) * 100
            if self.disposed < len(self.bombs):
                time_score = 0.0
            disposal = 100.0 * self.disposed / max(1, len(self.bombs))
            health_score = 100.0 * health / (self.n * 50)
            out["proxy_score"] = 0.6 * disposal + 0.2 * health_score + 0.2 * time_score
        else:
            out["placed"] = len(self.bombs)
            out["submit_step"] = self.submit_step
        return out
