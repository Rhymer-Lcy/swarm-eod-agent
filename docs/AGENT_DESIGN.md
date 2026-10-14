# Agent design: interfaces, architecture and role policies

Dates are business dates in UTC+8. Phase 4 (2026-10-14) validated B2 and R2
without changing them: the agent package is byte-identical to `067818a`, and
the Phase 4 blue candidate B3 (section 4) was not adopted and is kept as the
frozen package `devtools/comparators/p4_ce09cb7`. Status as of Phase 3: the
blue candidate **B2** (rescue) is implemented under
`src/swarm_rescue/solutions/eod/` and selected by
`solutions/my_drone_eval.py`; the red candidate is described in section 5
and its Phase 3 status in [PHASE3_PLAN.md](PHASE3_PLAN.md). The Phase 2
candidates B1 and R1 are kept as the frozen comparator
`devtools/comparators/p2_540054b`, the Phase 1 candidates B0.1 and R0.1 as
`devtools/comparators/p1_ddc6897`. Every candidate was adopted on paired
comparisons with few rounds; none is competitive-proven. Measured results
and their limits are in [EXPERIMENTS.md](EXPERIMENTS.md), method selection in
[RESEARCH.md](RESEARCH.md).

## 1. Verified simulator and participant-interface facts

Every row was read from the supplied source (`src/swarm_rescue/`) or
measured with the scripted probes P1-P3 (`devtools/simlab/probe.py`, ledger
rounds 1-3). "Probe" rows are measurements on the supplied simulator, not
assumptions.

| Topic | Fact | Source |
|---|---|---|
| Control loop | Per step: every drone's `define_message_for_all()` is collected, then every `control()` runs, then one physics step (10 pymunk sub-steps of 0.1) is applied and messages are delivered. | `gui_sr.on_update`, `playground.step` |
| Message latency | A message returned by `define_message_for_all()` at step t is readable in `control()` at step t+1 by teammates within 250 px (measured range 252 px including motion). Messages are Python objects passed in-process. | P1; `communicator.py` |
| Command validation | Out-of-range `forward`/`lateral`/`rotation` raise `ValueError`; an unknown key raises `KeyError` (so a blue command must not contain `place_bomb`). Thrust is clipped per axis, then rescaled to unit norm. | `controller.py`, `agent.receive_commands`, `drone_base._apply_commands` |
| Translation | Exact (residual below 1e-4): `v' = 0.95 v + 0.57293 u`, `p' = p + 0.97729 v + 0.52908 u`, `u` the world-frame thrust vector; terminal speed 11.46 px/step, time constant 20 steps. | P1 (975 clean samples) |
| Rotation | `0.17591 * rotation` rad per step, linear in the command; positive is counter-clockwise. | P1 |
| Lateral sign | **Positive `lateral` moves the drone to its left** (+90 degrees). The participant guide (02, section 4) says "positive moves right"; the code and the probe agree on left (D15). | P1; `drone_base.py` |
| GPS / compass | AR(1) noise, phi 0.98, stationary sigma 5 px per axis and 4 degrees. | `drone_sensors.py`; P1 |
| Odometer | `(dist, alpha, theta)` per step with white noise 0.2 px, 8 degrees, 1 degree. | `drone_sensors.py`; P1 |
| Lidar | 181 rays from -pi to pi relative to the heading, counter-clockwise; range 300 px; no return reads 300 plus noise; range noise sigma 2.5 px. Returns on drones, bombs and the disposal center appear like walls. | `drone_lidar.py`; P1 |
| Semantic sensor | 35 rays, 200 px; walls filtered out of the output but they still stop the rays (a ray that hits a wall reports nothing), so every detection is in line of sight; distance to the detected surface plus N(2.5, 1); angle counter-clockwise relative to the heading. A drone does not see the bomb it holds. | `drone_semantic_sensor.py` (`_compute_raw_sensor` drops wall hits after ray casting); P1 |
| Grasp | With the grasper on, a bomb is grasped when its centre is about 29 px from the drone centre; it then rides about 10.6 px out on the side where it was grasped. | P3 |
| Carrying | Mass rises from 50 to 150; speed at full thrust about 3.1 px/step. **Rotation while carrying is about 0.39 of the commanded value** (median ratio, P3), with per-step deviations mostly below 3.5 degrees. | P3 |
| Disposal | A carried bomb is disposed when it touches the center. Driving the grasp side into the center works (frontal and side, 160 steps each in P3); reversing with the bomb trailing never disposed. A released bomb drifts back towards its spawn point at about 0.17 px/step. | P3; `disposal_center.py`, `bomb.py` |
| Placement (red) | Rising edge of `place_bomb`; fails silently with an empty inventory, with the grasper on, while holding a bomb, or within 24 px of another bomb; the bomb is placed at the drone's true position. | P1; `bomb_deployer.py` |
| Health | -1 per contact with a wall or drone, with a cooldown of 1 **wall-clock** second, so damage depends on simulation speed (D17). The disposal center causes no damage. | `drone_abstract.collide_wall` |
| Ground-truth guard | `true_*()` raise only inside `control()` in competition mode; `define_message_for_all()` runs outside the guard (D16). The static checker rejects any `true_*()` call in submitted code. Agent code here calls none. | `ground_truth_guard.py`, `submission_check.py` |
| Determinism | With `random` and `numpy.random` seeded before the map is built, two probe rounds reproduced every pose, sensor value and lidar ray bit for bit; health differed (wall-clock cooldown). Seeds do not align noise across different agents once their sensing differs. | P1/P2 |
| Disposal center | Lies 50-180 px from the drone start grid on all five supplied maps, so it is in semantic range at the start; this is used only as a fallback hint. | Fixtures |

## 2. Architecture

```
simulator --(official measured API)--> shell.py ---Observation---> brain (blue.py | red.py)
                                          ^                              |
                                          +-------------Command----------+
brain = BrainBase (base.py) + role policy
BrainBase: estimator.py, grid.py (+ planner), perception.py, motion.py, comms.py,
           coverage.py (enabled per role)
red only:  wallmap.py
```

* **Information boundary.** `shell.py` is the only module that touches the
  framework. It reads sensors through the measured API only and builds an
  `Observation` (`observation.py`); the brains never see simulator objects,
  so they run unchanged in tests and in the offline approximation. A brain
  exception is caught in the shell; the drone holds still for that step and
  the error is counted (no prints).
* **Data contracts.** `Observation` (step, wall time, identifier, team size,
  map size, limits, GPS, compass, odometer, lidar and angles, semantic hits,
  grasp state, health, inventory, messages, flags) and `Command` (forward,
  lateral, rotation, grasper, place_bomb, submit_map).
* **Per-step pipeline.** estimate pose; read messages; perceive (bombs,
  disposal center, lidar masking, map update every second step); decide;
  record the command for the next prediction; compose the outgoing message.
* **Compute.** Phase 3 (instrumented, same server): B2 59.6-74.1 steps per
  second with 10 drones across Map01-Map05 against 60.5-72.0 for B1 in the
  same configurations (median over nine pairs 71.8 against 68.2), controller
  0.82-1.14 ms per drone-step for B2 and 0.85-1.12 for B1. Measured on the remote server in Phase 2: B1 70-73 simulation
  steps per second with 10 drones on Map01 and Map02 and 66-68 on Map03-Map05
  (B0.1: 49-53), R1 85-99 with 5 drones (R0.1: 76-89); simulator included,
  the supplied examples run at 200-260. B1's controllers take about 0.87 ms
  per drone-step (B0.1: 1.37). A profile showed that the Phase 1 assumption
  (geodesic fields dominate) was wrong; see EXPERIMENTS.md, section 7.8.

## 3. Shared components

| Component | File | Design |
|---|---|---|
| Pose estimator | `estimator.py` | Dead reckoning with the exact motion model; GPS and compass correct with small gains (position: 0.03 + 0.006 x speed, raised after a disturbance). **Heading (B1, carry-aware):** an unloaded step uses the exact rotation model unless the odometer contradicts it by more than 3.5 degrees (contact), as in Phase 1. A step that may involve a bomb (held when the command was issued or held now, so grasp and release transitions included) or whose command is unknown uses the measured odometer rotation unconditionally (noise 1 degree per step); without an odometer, a carrying step is predicted with the probe's median ratio 0.39 and a variance from its 10-90 % spread. A heading variance is propagated and the compass gain is `P / (P + R)` (R = (4 degrees)^2), bounded to [0.004, 0.5]: for unloaded flight this equals Phase 1's running average exactly; while carrying it settles near 0.2. Gain choice: for a compass error that is AR(1) with phi 0.98 and an odometer random walk of 1 degree per step, the steady-state heading error of a complementary filter is flat at 4.1-4.7 degrees RMS for gains 0.05-0.3 and rises to 5.4 at 0.025 (closed form; a direct simulation of the filter gives 4.0-4.5 and 5.3), so the Kalman gain near 0.2 lies in that flat optimum. Commands are consumed once, so a step after a brain failure (zero command applied) is treated as unknown. Real rounds: carrying heading error p95 6.8-9.3 degrees (B0.1 in the same paired rounds: 37.0-50.9). |
| Coverage summary | `coverage.py` | Grow-only bitmap of 32 px blocks searched by the team (a block counts once half of its non-occupied cells are searched), packed one bit per block (at most 234 bytes on the supplied maps), broadcast every 10 steps, merged by bitwise OR, so delayed, duplicate or stale messages and disconnections are harmless; payloads with other dimensions or lengths are rejected. Enabled for red only (section 5). |
| Occupancy grid | `grid.py` | Log-odds on 8 px cells; per scan, end-point cells occupied and crossed cells free, once per cell; vectorized ray sampling on every second ray. Frontier cells exclude unknown slivers next to walls. |
| Planner | `grid.py` | Geodesic cost fields with `skimage.graph.MCP_Geometric` (compiled), OpenCV wavefront fallback when scikit-image is missing; obstacles inflated by 24 px; goals and starts inside the inflation margin get a short expensive corridor. |
| Motion | `motion.py` | Holonomic control: desired world velocity turned into thrust by one-step inversion of the model; speed capped by the measured braking distance to the lidar corridor clearance; short-range repulsion from walls and teammates; line-of-sight lookahead on the path; stuck monitor with recovery. |
| Perception | `perception.py` | Bomb centres from grouped semantic rays refined with matching lidar rays (error median 2.6 px with true pose); lidar returns on drones, bombs and the disposal center are masked; disposal-center surface points and a robust box. |
| Protocol | `comms.py` | Versioned dictionaries of immutable values; receivers copy what they need and never keep a reference to another drone's objects. |
| Navigation core | `base.py` | Search layer (cells seen within the semantic search range), exploration target choice with team spreading and an initial identifier-based sector bias, path following, recovery, fleet gossip. Per-role flags: `SHARE_COVERAGE` (teammates' covered blocks count as searched), `COVERAGE_DROPS_FRONTIER` (whether frontier cells in covered blocks are dropped too) and `COMMIT_TARGETS` (keep an exploration target until it is reached or covered, its travel-time budget of 60-400 steps expires, or a lower identifier targets the same area within 100 px). |

## 4. Blue candidate B2 (`blue.py`, `delivery.py`; B1 and B0.1 differences noted)

* **States:** `explore`, `goto`, `approach`, `carry`, `deliver`, `idle`,
  recovery.
* **Bomb table:** sightings, carried and disposed states; the newest
  information wins; grasped detections never create entries; a free bomb
  within 100 px in line of sight that is not detected for 10 steps is
  marked gone.
* **Allocation:** greedy decentralised claims. A drone claims the free bomb
  with the lowest geodesic cost unless an unoccupied teammate (positions
  gossiped over several hops) is clearly closer or a lower-cost claim exists
  (ties: lower identifier). Claims are broadcast and relayed; they lapse after
  40 steps without renewal.
* **Grasp:** inside 140 px the drone steers on the live relative detection
  (immune to GPS drift), faces the bomb, slows to 2.5 px/step inside 60 px
  and holds the grasper on. Timeout 140 steps, then the bomb is skipped for
  400 steps.
* **Delivery goals (B2).** Walls stop semantic rays, so a center surface point
  detected from position `o` proves that the segment from `o` to the point
  was free. For each such detection (heading standard deviation below 8
  degrees) the drone records a *witnessed stand-off* on that segment, 36 px
  out from the point (at the drone itself when it is closer), one per grid
  cell, at most 400, newest kept (`delivery.ApproachSet`). Before use, a
  witness must still be supported by the map: its cell is not occupied and
  the segment to its surface point crosses no occupied cell (12 px at the
  center end are ignored, because the center itself is mapped). The carry
  field (unobserved cells cost 10) is computed from the supported witnesses;
  only when none exists (for example a center known from a teammate's box
  alone) does it fall back to B1's stand-offs outward from the center's
  middle, filtered by the same map check. B1 used only those outward
  stand-offs, which near a corner can fall behind a wall running a few
  pixels from a face of the center; the planner then connects both sides of
  the wall through the goal cell (Map03, EXPERIMENTS.md section 7.6).
* **Terminal push (B2).** Within 100 px the drone drives its grasp side into
  the center only with a line-of-sight certificate, at every distance: a
  surface point detected in this step within 120 px whose segment the map
  does not block (14 px at the center end ignored), or the surface point of
  the supported witness the drone stands on (within 20 px). B1 skipped the
  line-of-sight test within 50 px. Without a certificate the drone follows
  the carry field, or holds at a goal cell; it never pushes towards a point
  through whatever lies between.
* **Progress and retries (B2).** Within 150 px of the center, a carrier that
  gains less than 5 px of distance to the center in 160 steps while staying
  within 80 px of where it last progressed has failed at that approach: the
  witnesses within 56 px of it are excluded for 600 steps (1200 and 2400
  after repeated failures there), the field is recomputed, and no terminal
  push starts inside an excluded area. If every supported witness is
  excluded, all are used again. This replaces B1's timeout, which re-aimed at
  other surface points on the same side.
* A detachment near the center counts as a delivery, elsewhere as a drop that
  is re-queued; the grasper stays on throughout carry, re-planning and
  recovery. With no observed center the drone returns towards its start
  position.
* **Waiting (B2).** An idle drone without a return area waits at a known
  free, reachable cell at least 130 px from the center near the start area
  (re-chosen every 100 steps), instead of within 45 px of its start
  position, which on the supplied maps lies 50-180 px from the center.
* **Messages:** state, target, busy flag, bomb digest (up to 40), own claim,
  relayed claims, fleet positions, disposal-center box, disposal records
  (pickup positions). B1 validates digest items (finite numbers of the
  expected length) and skips items it has already applied.
* **Shared exploration:** off for blue (`SHARE_COVERAGE` and `COMMIT_TARGETS`
  false); offline it delayed completion on Map01 (EXPERIMENTS.md, 7.9).
* **Candidate B3 (Phase 4, not adopted).** A carrier that does not know the
  center searches for it with the unloaded drones' target choice along
  planned paths, instead of B2's straight drive to the start area
  (`_seek_center`). It turned a 0/10 corridor round into 6/10 but disposed
  one bomb fewer than B2 in another pair, so B2 was retained under the
  preregistered rule (PHASE4_PLAN.md, addendum 3; EXPERIMENTS.md 9.7).

## 5. Red candidate R2 (`red.py`, `wallmap.py`; R1 and R0.1 differences noted)

* **Exploration (R1, unchanged in R2):** team coverage summaries and target commitment are on
  (`SHARE_COVERAGE`, `COMMIT_TARGETS`), with `COVERAGE_DROPS_FRONTIER` false:
  coverage is a semantic-search notion, so blocks covered by teammates stop
  being search targets, but unmapped space (frontier cells) stays a target
  because red must map the walls there. Dropping frontier cells too lowered
  the offline Map02 map score; keeping them did not (EXPERIMENTS.md, 7.9).

* **Schedule** (T = step limit): explore until 0.25 T; place until 0.40 T
  (decisions staggered by 15 steps per identifier); rally towards the start
  area from 0.40 T; the leader (lowest identifier heard within 300 steps)
  submits from 0.47 T once it holds a fresh snapshot from every known
  teammate, and in any case at 0.495 T. The time score is full when the map
  is submitted by 0.5 T.
* **Wall evidence:** validated lidar returns within 170 px add a count at the
  return point (pushed 1.5 px into the wall) on a 1 px grid, only when the
  pose estimate is trusted. Drones, bombs and the disposal center are
  excluded.
* **Map sharing:** compressed sparse snapshots of each drone's own counts,
  refreshed every 50 steps (every 10 steps after the rally), gossiped with one
  snapshot per message; the newest snapshot of each source is kept and the
  sources are summed once each.
* **Rendering:** Gaussian smoothing (sigma 2.2 px), then pixels above an
  absolute threshold and above 0.33 of the local peak; specks below 12 px
  removed; the observed disposal-center box removed except for the border
  band. **Border handling (conditionally approved by the owner):** the arena
  border runs along the map edge, so for each 40 px border segment in which
  evidence was observed, the band from the edge to the observed inner face is
  drawn, in R1 only between the segment's outermost observed positions plus
  12 px, so that a gap in the border is never filled beyond the evidence.
  Unobserved segments are not drawn. Inputs: map dimensions, observed lidar
  returns and the estimated pose; no supplied map's wall coordinates. Tests
  with synthetic boundary gaps and interior walls near the edge are in
  `devtools/tests/test_red_r1.py`; the ablation on real evidence (bands add
  6.8-29.6 map points on Map01 with no penalized pixel) is in EXPERIMENTS.md,
  section 7.7. Remaining ambiguity: whether every official map has a
  complete border frame is not documented by the organizer; the clipping
  limits the damage if one does not.
* **Placement:** greedy marginal coverage on the current team map at 4 px
  resolution (border walls weigh 0.5, blast radius `round(0.12 * min(W, H))`),
  excluding walls already covered by placed or claimed bombs; candidates are
  reachable free cells away from obstacles, the center and existing bombs
  (one blast radius apart); score = coverage - 0.3 x path length + 0.12 x
  best coverage x min(distance to center, 600)/600. Contested areas go to the
  lower identifier. Unplaced bombs are dropped where the drone is shortly
  before the rally.
* **Placement changes in R2** (each found by the Phase 3 offline diagnosis,
  EXPERIMENTS.md section 8.6):
  * *Target re-validation and relayed claims.* A target is given up as soon
    as a known placed bomb (own detections or gossip) lies within 0.8 blast
    radii of it; R1 kept a target for up to 240 steps after such a bomb
    became known. Claims are relayed in every message with the step at
    which their owner made them and stay valid for 80 steps; R1 heard a
    claim only within radio range of its owner and dated it on receipt.
  * *Solid obstacle interiors.* The wall score counts the whole area of a
    solid box, while the lidar maps only its outline. When choosing a target,
    R2 weighs as solid wall every 4-connected region of unknown cells of its
    own occupancy grid that no free cell touches, that does not reach the
    grid edge, that is at most 70,000 px^2 and that does not overlap the
    observed disposal center (whose interior is also unknown but is not a
    wall). Only the placement weights change; the submitted map does not.
    Offline, the inferred area was 95-98 % truth wall on Map04, 81-83 % on
    Map05 and 79 % on Map02; on maps without boxes only a few hundred
    pixels were inferred.
* **Safety:** any drone submits if the wall-clock limit comes close (the
  leader at 80 %, the others at 88 %), because a round without a submission
  scores 0 on the map.

## 6. Compliance boundary

* Agent code uses only the measured API listed in section 1. It calls no
  `true_*()` method, reads no `self.position`-style attribute, does not
  touch `_playground`, `sys.modules`, `gc`, `ctypes` or `inspect`, and does
  not print. `devtools/tests/test_eod_agent.py` checks this statically; the
  check was mutation-tested (each of three planted violations made it fail).
* The supplied checker passes in workspace mode and on a test package built
  like `create_zip_submission.sh` with the placeholder identity.
* No behaviour depends on the open discrepancies D1-D3. Wall-clock time is
  read only for the red safety submission, which protects against D1 rather
  than exploiting it.
* Evaluation-only tooling (truth logging, probes, fixtures, offline
  simulator) lives in `devtools/` and is never imported by agent code.

## 7. Known defects and limitations (prioritized)

Resolved in Phase 2: heading drift while carrying (B1 estimator), no sharing
of searched areas (red; blue keeps it off), exploration target churn (red),
disposal-center model outliers (live aim and gated recording), optimistic
carry routes (higher unknown cost). Resolved in Phase 3 (B2): delivery to a
partly walled disposal center (witnessed stand-offs, line-of-sight
certificate at every distance, progress-based re-planning; Map03 family 50 of
50 bombs against 38 of 50 for B1) and idle drones waiting next to the center.
Found in Phase 4 on synthetic custom geometry (EXPERIMENTS.md section 9) and
open, in priority order:

1. **A hidden disposal center is never searched for once every drone carries
   (blue, confirmed).** A carrier that does not know the center drives
   straight at its start area and waits; blue does not explore while
   carrying. With the center out of sight of the start area and at least as
   many bombs as drones, all ten drones carried a bomb and none ever saw the
   center (P4CorridorV1 and P4CorridorV2: 0 of 10). The supplied maps place
   the center in view of the start area. B3 addresses this and was not
   adopted; the next step is a B3-like search that does not cost deliveries
   elsewhere.
2. **The planner closes narrow passages (all blue and red versions).** With
   24 px obstacle inflation on 8 px cells, a passage between ideal walls is
   untraversable for the planner below about 56 px (up to 62 px depending
   on the cell alignment), although a drone is 30 px wide; under pose error
   walls are mapped thicker and wider passages close (a 70 px corridor in
   carriers' own maps).
3. **Radial fallback without an exit (blue).** With the center known only
   from a teammate's box, an own map already complete around it and the
   start area behind a walled side, the fallback's only goal is the enclosed
   middle and the carrier waits behind the wall (offline, `test_p4_risk_a.py`).
   With an incomplete map the same goal draws the carrier around the walls
   and it recovers; the terminal push never moved through a wall.
4. **Solid-interior inference takes sealed rooms as solid (red, measured).**
   Precision 0.47-0.48 on a synthetic map with two sealed rooms (94-99 % of
   the false area inside them) against 0.98 on Map04; the whole-run effect on
   the wall score was mixed (EXPERIMENTS.md 9.4).
5. **Witnesses are not shared (blue).** Each drone uses only stand-offs it
   witnessed itself; a carrier that saw the center from one side only
   routes to that side even when another side is nearer. All drones see the
   center from the start area on the supplied maps, so this has not failed
   in a round; it can lengthen carries.
6. **Worst-case controller latency (blue).** B2's maximum `control()` time
   was 29-33 ms per round against 27-31 ms for B1 (means equal); the
   source of the extra 2-3 ms has not been profiled.
7. **Bomb table duplicates (blue).** Records of moved or carried bombs can be
   recreated from gossip: 100-111 records for 10 bombs in offline Map03 runs.
   Digest de-duplication reduces it; the table is not yet consolidated.
8. **Compute margin.** B2 ran at 59.6-74.1 steps/s in the Phase 3 rounds on
   the server (B1 60.5-72.0 in the same configurations; the lowest values on
   Map04); a 7200-step round at 60 steps/s would take 120 s of 240 s there.
   The evaluator's speed is unknown.
9. **Red wall coverage still depends on what is mapped at placement time.**
   R2 raised the wall score in three real pairs (mean 54.61 to 78.81; Map04
   33.28 to 65.42), but on Map04 the team map held only about 60 % of the
   weighted walls when placement started (offline), and the schedule
   fractions are fixed. The solid-interior inference cannot tell a closed,
   unexplored room from a solid box; on the supplied maps without boxes it
   inferred only a few hundred pixels offline, but a map with sealed rooms
   would mislead it (measured in Phase 4, item 4). Emergency drops near the rally point still overlap
   occasionally.
10. Return-area support is minimal (idle drones go to the last position where
   `is_inside_return_area` was true); the supplied maps have no return area.
