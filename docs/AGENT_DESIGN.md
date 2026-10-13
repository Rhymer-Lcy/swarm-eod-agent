# Agent design: interfaces, architecture and role policies

Dates are business dates in UTC+8. Status as of 2026-10-13: candidates **B0.1**
(blue, rescue) and **R0.1** (red, place) are implemented under
`src/swarm_rescue/solutions/eod/` and selected by
`solutions/my_drone_eval.py`. They are first functional candidates, not
tuned or competitive-proven agents; measured results and their limits are in
[EXPERIMENTS.md](EXPERIMENTS.md), method selection in [RESEARCH.md](RESEARCH.md).

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
| Semantic sensor | 35 rays, 200 px; walls filtered out; distance to the detected surface plus N(2.5, 1); angle counter-clockwise relative to the heading. A drone does not see the bomb it holds. | `drone_semantic_sensor.py`; P1 |
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
BrainBase: estimator.py, grid.py (+ planner), perception.py, motion.py, comms.py
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
* **Compute.** Measured on the remote server: blue about 50 simulation steps
  per second with 10 drones, red 79-87 with 5 drones (simulator included;
  the supplied examples run at 200-260). The dominant cost is geodesic field
  computation, cached per drone for up to 25 steps.

## 3. Shared components

| Component | File | Design |
|---|---|---|
| Pose estimator | `estimator.py` | Dead reckoning with the exact motion model; the odometer flags model violations (contact, carrying) and then supplies the measured displacement and rotation; GPS and compass correct with running-average gains that settle at a floor (0.03 + 0.006 x speed for position, 0.004 for heading). Offline: heading RMS 1.4 degrees, position RMS 6.8 px. Real runs: see the defect in section 7. |
| Occupancy grid | `grid.py` | Log-odds on 8 px cells; per scan, end-point cells occupied and crossed cells free, once per cell; vectorized ray sampling on every second ray. Frontier cells exclude unknown slivers next to walls. |
| Planner | `grid.py` | Geodesic cost fields with `skimage.graph.MCP_Geometric` (compiled), OpenCV wavefront fallback when scikit-image is missing; obstacles inflated by 24 px; goals and starts inside the inflation margin get a short expensive corridor. |
| Motion | `motion.py` | Holonomic control: desired world velocity turned into thrust by one-step inversion of the model; speed capped by the measured braking distance to the lidar corridor clearance; short-range repulsion from walls and teammates; line-of-sight lookahead on the path; stuck monitor with recovery. |
| Perception | `perception.py` | Bomb centres from grouped semantic rays refined with matching lidar rays (error median 2.6 px with true pose); lidar returns on drones, bombs and the disposal center are masked; disposal-center surface points and a robust box. |
| Protocol | `comms.py` | Versioned dictionaries of immutable values; receivers copy what they need and never keep a reference to another drone's objects. |
| Navigation core | `base.py` | Search layer (cells seen within the semantic search range), exploration target choice with team spreading and an initial identifier-based sector bias, path following, recovery, fleet gossip. |

## 4. Blue candidate B0.1 (`blue.py`)

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
* **Delivery:** geodesic field from stand-off points outside the observed
  center; inside 100 px it drives the grasp side into the nearest observed
  surface point; a detachment near the center counts as a delivery, elsewhere
  as a drop that is re-queued. With no observed center the drone returns
  towards its start position.
* **Messages:** state, target, busy flag, bomb digest (up to 40), own claim,
  relayed claims, fleet positions, disposal-center box, disposal records
  (pickup positions).

## 5. Red candidate R0.1 (`red.py`, `wallmap.py`)

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
  band. **Border handling (owner review requested):** the arena border runs
  along the map edge, so for each 40 px border segment in which evidence was
  observed, the band from the edge to the observed inner face is drawn.
  Unobserved segments are not drawn. This uses the known position of the
  border, not unobserved fill.
* **Placement:** greedy marginal coverage on the current team map at 4 px
  resolution (border walls weigh 0.5, blast radius `round(0.12 * min(W, H))`),
  excluding walls already covered by placed or claimed bombs; candidates are
  reachable free cells away from obstacles, the center and existing bombs
  (one blast radius apart); score = coverage - 0.3 x path length + 0.12 x
  best coverage x min(distance to center, 600)/600. Contested areas go to the
  lower identifier. Unplaced bombs are dropped where the drone is shortly
  before the rally.
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

1. **Heading drift while carrying (blue, confirmed).** The estimator predicts
   rotation from the command, but a carrying drone turns by about 0.39 of it,
   and the per-step deviations stay mostly below the odometer check. Real
   rounds: heading error median 15-20 degrees and 95th percentile 47-57
   degrees while carrying, against medians of 1.5-6.3 degrees in the other
   states (round 10). Effects: wrong
   thrust directions, slow and wandering carries, mis-oriented deliveries,
   polluted disposal-center points. Proposed fix: use the odometer rotation
   while carrying and raise the compass gain floor then. An offline what-if
   with this change disposed 10/10 on Map01 for two seeds (current: 9 and 8);
   it is not yet validated on the real simulator.
2. **No sharing of searched areas (both roles).** Each drone tracks only its
   own coverage, so drones re-search areas teammates covered; R0.1 on Map01
   left the right-hand compartment unmapped in one of two seeds.
3. **Optimistic planning through unknown space while carrying.** Routes over
   unobserved cells flip once the wall is seen; at 3 px/step each flip costs
   hundreds of steps.
4. **Disposal-center model outliers.** Surface points recorded during large
   pose errors persist; delivery then aims at free space. The final approach
   should use live relative detections, as the grasp already does.
5. **Exploration target churn.** Targets are re-chosen every 40 steps, which
   produces wavy paths.
6. **Compute margin.** About 50 steps/s on the server for blue; a full
   7200-step round would take about 144 s of the 240 s limit, so a slower
   evaluator could hit the wall-clock limit on long rounds.
7. **Fixed red schedule.** Placement and submission times are fixed fractions
   of the step limit; mapping quality on short rounds (Map01) varies strongly
   between seeds.
8. Return-area support is minimal (idle drones go to the last position where
   `is_inside_return_area` was true); the supplied maps have no return area.
