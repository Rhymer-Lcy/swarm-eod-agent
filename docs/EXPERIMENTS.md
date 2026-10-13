# Experiments: methodology, benchmarks and results (Phases 1 to 3)

Dates are business dates in UTC+8. All rounds ran on the remote server
under the rules in [ENVIRONMENT.md](ENVIRONMENT.md), one process at a time,
NUMA node 0 only, on one GPU identified by its UUID (kept in the private
operations notes). Raw records stay in the ignored `_local/runs/`; this page
summarizes them.

## 1. Methodology

**Harness** (`devtools/simlab/runner.py`). Each round is one call of the
official `Launcher.one_round` with a generated one-round plan, so map
construction, the step and wall-clock limits, the competition-mode guard and
the official score managers are exactly those of the baseline. The harness
adds: seeding of `random` and `numpy.random` before the map is built;
selection of the agent class (`official` = the submission entry point; or a
named development class for baselines and probes); an evaluation-only truth
log; and a ledger with one line when a round starts and one when it ends, so
interrupted rounds remain visible. It refuses to start a round once the
phase budget is used (verified: a 17th attempt was refused).

**Scores.** Blue rows report the official blue score. Red rows report the
**partial** red score (map 50 %, wall 20 %, time 10 %); the bomb-difficulty
term (20 %) needs organizer reference agents and is not computed locally
(maximum partial score 80).

**Truth log.** `diagnostics.with_truth_log` records drone and bomb ground
truth, and a read-only copy of the agent's pose estimate, state and target,
from `define_message_for_all`, which the framework calls before each
`control()`. Nothing flows back to the agent. The pose estimate logged at
step t describes the state one physics step earlier than the logged truth;
error statistics below are aligned for that.

**Determinism.** Two probe rounds with the same seed reproduced every pose,
sensor value and lidar ray bit for bit; health differed because the
collision cooldown is wall-clock based. Seeds give identical initial headings
and initial noise across agents, but noise streams diverge as soon as two
agents sense differently, so comparisons across agents with the same seed are
matched in setup, not paired in noise.

**Partitions.** Development: Map01 and Map02 (offline tuning of both roles;
all real-simulator development of both roles used Map01). Held out from the
real simulator until their single evaluation round: Map03 (red; never used
for either role before), Map04 and Map05 (blue). Disclosure: blue was run in
the offline approximation on Map04 and Map05 during development (to check
behaviour and compute cost); no parameter was changed in response to those
runs, but the maps were not unseen in the strict sense. Bomb layouts: the
synthetic layout S1 (`devtools/simlab/layouts.py`, uniform, seed 1, 10
bombs; labelled synthetic) and layouts placed by red agents in earlier
rounds.

**Budget.** 16 complete rounds were authorized for this phase; 16 were used:
3 characterization probes, 3 baseline rounds, 10 candidate rounds. None was
interrupted; all ran from clean commits.

## 2. Run ledger

Generated with `python -m devtools.simlab.summarize <ledger> --phase phase1`.
`official` is the submission entry point at the commit of the run: B0/R0
(rounds 5, 7; `e9d7828`), B0.1/R0.1 (rounds 8-9: `1d72889`; rounds 10-16:
`410c43b`, which differs from `1d72889` only in development tooling).

| # | label | map | mode | agent | seed | steps | wall s | score | components | bombs | termination |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | p1-dynamics-probe | Map01 | place | DynamicsProbe | 11 | 421 | 2.94 | 11.01 | map 0.0, time 100.0, wall 5.0 | 3 placed | map_submitted |
| 2 | p2-determinism-probe | Map01 | place | DynamicsProbe | 11 | 421 | 2.89 | 11.01 | map 0.0, time 100.0, wall 5.0 | 3 placed | map_submitted |
| 3 | p3-grasp-probe | Map01 | rescue | GraspProbe | 12 | 2000 | 12.62 | 63.96 | disposal 75.0, health 94.8, time 0.0 | 3/4 disposed | step_limit |
| 4 | r-baseline-example-map01 | Map01 | place | MyDronePlaceExample | 21 | 511 | 1.94 | 16.79 | map 0.0, time 100.0, wall 34.0 | 10 placed | map_submitted |
| 5 | r0-map01 | Map01 | place | official | 21 | 940 | 11.21 | 47.96 | map 55.4, time 100.0, wall 51.4 | 10 placed | map_submitted |
| 6 | b-baseline-example-s1 | Map01 | rescue | MyDroneRescueExample | 22 | 2000 | 9.76 | 20.00 | disposal 0.0, health 100.0, time 0.0 | 0/10 disposed | step_limit |
| 7 | b0-s1 | Map01 | rescue | official | 22 | 2000 | 38.21 | 61.36 | disposal 70.0, health 96.8, time 0.0 | 7/10 disposed | step_limit |
| 8 | b0.1-s1 | Map01 | rescue | official | 22 | 2000 | 38.77 | 67.24 | disposal 80.0, health 96.2, time 0.0 | 8/10 disposed | step_limit |
| 9 | r0.1-map01 | Map01 | place | official | 21 | 940 | 10.76 | 55.03 | map 67.9, time 100.0, wall 55.3 | 10 placed | map_submitted |
| 10 | b0.1-vs-r0.1 | Map01 | rescue | official | 23 | 1731 | 33.76 | 84.18 | disposal 100.0, health 94.0, time 26.9 | 10/10 disposed | all_disposed |
| 11 | bex-vs-r0.1 | Map01 | rescue | MyDroneRescueExample | 23 | 2000 | 9.78 | 19.96 | disposal 0.0, health 99.8, time 0.0 | 0/10 disposed | step_limit |
| 12 | b0.1-vs-rex | Map01 | rescue | official | 24 | 517 | 10.7 | 99.84 | disposal 100.0, health 99.2, time 100.0 | 10/10 disposed | all_disposed |
| 13 | b0.1-heldout-map04 | Map04 | rescue | official | 25 | 1632 | 33.25 | 99.56 | disposal 100.0, health 97.8, time 100.0 | 10/10 disposed | all_disposed |
| 14 | r0.1-heldout-map03 | Map03 | place | official | 26 | 2350 | 29.87 | 77.57 | map 99.7, time 100.0, wall 88.6 | 10 placed | map_submitted |
| 15 | b0.1-heldout-map05 | Map05 | rescue | official | 27 | 1484 | 28.75 | 99.40 | disposal 100.0, health 97.0, time 100.0 | 10/10 disposed | all_disposed |
| 16 | r0.1-map01-seed28 | Map01 | place | official | 28 | 957 | 11.11 | 70.92 | map 90.6, time 100.0, wall 78.0 | 10 placed | map_submitted |

Layouts: rounds 3 (probe), 6-8 S1 on Map01; 10-11 the layout placed by R0.1
in round 9; 12 the layout placed by the supplied example in round 4; 13 and
15 S1 on Map04 and Map05. Red rounds used 5 drones with 2 bombs each, blue
rounds 10 drones (the supplied local plans' settings).

## 2a. Characterization (probes)

Rounds 1-3 measured the dynamics, sensor models, conventions, grasping,
carrying, disposal and placement rules; the results are tabulated in
[AGENT_DESIGN.md](AGENT_DESIGN.md), section 1. Two findings correct the
participant guide or common assumptions: positive lateral thrust moves the
drone to its **left**, and a carrying drone turns by only about 0.39 of the
commanded rotation.

## 3. Comparisons and their uncertainty

Each configuration was run once, except R0.1 on Map01 (two seeds). There
are no confidence intervals: one run per cell cannot separate the effect of
an algorithm change from run-to-run variation, and the two R0.1 Map01 runs
(partial 55.0 and 70.9; map 67.9 and 90.6) show that this variation is
large for red on Map01.

* **Against the supplied examples (same configuration and seed).** Blue on S1:
  the example disposed 0/10 (score 20.0), B0 7/10 (61.4), B0.1 8/10 (67.2).
  On R0.1's layout the example disposed 0/10 (20.0) and B0.1 10/10 (84.2).
  Red on Map01 seed 21: the example's random map scored 0 (partial 16.8), R0
  55.4 (48.0), R0.1 67.9 (55.0). The direction is the same in every
  configuration tested, but these are single runs.
* **Corrective iteration (B0 to B0.1, R0 to R0.1).** Same scenario and seed,
  one run each: 7/10 to 8/10 disposed (61.4 to 67.2); red map 55.4 to 67.9
  (partial 48.0 to 55.0). The changes fixed observed mechanisms (below); the
  score differences alone are within the variation seen between seeds and
  are not evidence of improvement on their own.
* **Held-out maps.** B0.1 cleared all bombs on Map04 (step 1632 of 5000,
  score 99.6) and Map05 (step 1484 of 7200, 99.4); R0.1 on Map03 scored map
  99.7, wall 88.6, time 100 (partial 77.6 of 80). One run each.
* **Cross-role.** B0.1 cleared the supplied example's layout in 517 steps
  (99.8) and R0.1's layout in 1731 steps (84.2). Under the scoring formula
  `difficulty = min(100, 2 x (100 - blue score))`, R0.1's layout would rate
  100 against an agent like the example and 31.6 against one like B0.1. The
  organizer's reference agents are unknown, so this only illustrates the
  range; it is not a bomb-difficulty score.

## 4. Failure analysis

1. **R0, disposal center as a frontier (round 5).** Lidar returns on the
   center were masked from the map and from navigation, so the center's
   interior stayed unknown and its edge an exploration frontier; drones
   scraped along it and the right-hand compartment was never mapped (map
   55.4, credit 0.555, penalty 0.001). Fixed in R0.1: center returns are
   navigation obstacles (still excluded from the wall map).
2. **B0, no path from a cell inside the inflation margin (round 7).** A drone
   within 24 px of a wall sits in an impassable cell, the field is infinite
   there, and the delivery fell back to driving straight at the center:
   carrying drones slid along the partitions; six of ten carries lasted
   668-1468 steps, three of them still unfinished at the step limit. Fixed in
   B0.1: path extraction starts from the nearest reachable cell, and delivery
   falls back to a planned path.
3. **Heading drift while carrying (rounds 10, 12, 13; open).** Aligned pose
   error over three B0.1 rounds: position median 6.2-6.8 px; heading median
   3.5-5.9 degrees but 95th percentile 42-49 degrees, concentrated in
   carrying (round 10: median 15.4 degrees in `carry`, 19.6 in `deliver`,
   1.5-6.3 in the other states). Cause: the carried bomb slows the rotation
   (P3: median ratio 0.39 of the command; deviations mostly below the 3.5
   degree odometer check). Consequences seen in the traces: a 1400-step carry
   in round 10, deliveries aimed at disposal-center points 55-93 px off.
4. **No sharing of searched areas (open).** In round 9 the five red drones
   flew about 2700-3000 px each in the first 500 steps, mostly over the same
   left and middle parts of the map: three never passed x = 100 and only one
   crossed briefly into the right compartment (x > 288, up to x = 340). The
   right compartment and the right border were missing from the submitted
   map.
5. **Optimistic routes through unknown cells while carrying (open).** Round
   10, drone 9: the planned route alternated between the two ends of a
   partition as the wall was observed.

## 5. Offline approximation

`devtools/simlab/offline_sim.py` reproduces the measured dynamics, the
source-code noise models and the probe-measured mechanics on the walls-only
truth fixtures, without OpenGL. It was used for unit-level integration tests
and fast tuning; it overestimated real performance until the carried-rotation
effect was added after round 15 (it then reproduced the real heading drift:
median 10.6 and 95th percentile 44.3 degrees while carrying). A what-if run
with a carry-aware heading prediction (odometer rotation while carrying; a
development script, not applied to the agent):

| Offline scenario (S1) | Current B0.1 | Carry-aware estimator |
|---|---|---|
| Map01, seed 1 | 9/10, 2000 steps, proxy 72.2 | 10/10, 1421 steps, proxy 90.5 |
| Map01, seed 2 | 8/10, 2000 steps, proxy 66.1 | 10/10, 1656 steps, proxy 85.2 |
| Map04, seed 1 | 10/10, 1280 steps, proxy 99.5 | 10/10, 971 steps, proxy 99.7 |
| Map04, seed 2 | 10/10, 1567 steps, proxy 99.1 | 10/10, 1269 steps, proxy 99.3 |

These are signals from an approximate simulator, not competition results;
the change needs real-simulator validation.

## 6. Reproducing a round

```bash
# on the server, from the clone (host-specific values in the private notes)
CPUSET=<node-0 CPUs> EGL_DEVICE=<n> PYTHON=<env python> \
  devtools/simlab/remote_run.sh 600 -m devtools.simlab.runner \
  --label b0.1-s1 --map Map01 --mode rescue --drones 10 \
  --bombs-file _local/layouts/synthetic_Map01_uniform_n10_s1.json \
  --agent official --seed 22
# layouts and fixtures
python -m devtools.simlab.fixtures            # needs an OpenGL context
python -m devtools.simlab.layouts --map Map01 --bombs 10 --seed 1
# offline approximation (no OpenGL)
python -m devtools.simlab.offline_eval --role blue --map Map01 \
  --layout _local/layouts/synthetic_Map01_uniform_n10_s1.json --seed 1
```

## 7. Phase 2 (2026-10-13): paired comparisons, parity and held-out rounds

The plan, its arms, seeds, metrics and decision rules were frozen in
[PHASE2_PLAN.md](PHASE2_PLAN.md) before the first Phase 2 round; two dated
addenda record the decisions taken during the phase. Phase 2 had its own
budget of 30 rounds; **29 were started, none interrupted, all from clean
commits; one reserve round was not used.** The 16 Phase 1 rounds are
unchanged and not counted here.

Code of the rounds: rounds 1-13 at `76f8a76` (B1 agent code identical to
`e701426`), rounds 14-27 at `8a08483`, rounds 28-29 at `494138a` (agent code
identical to `290c077` in both; blue behaviour there equals B1, see the plan's
addendum 1). B0.1 and R0.1 rows ran the frozen comparator package
`devtools/comparators/p1_ddc6897` (byte-identical to `ddc6897`).

### 7.1 Run ledger (Phase 2)

Generated with `python -m devtools.simlab.summarize <ledger> --phase phase2`;
the comparator class names are shown as B0.1 and R0.1. "official" is the
submission entry point at the round's commit (B1 or R1). Run modes are
defined in the plan, section 3.

| # | label | map | mode | agent | run mode | seed | steps | wall s | steps/s | score | components | bombs | termination |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | p2-B1-b01-map01-s1-seed31 | Map01 | rescue | B0.1 (comparator) | instrumented | 31 | 2000 | 38.72 | 51.7 | 66.88 | disposal 80.0, health 94.4, time 0.0 | 8/10 disposed | step_limit |
| 2 | p2-B1-b1-map01-s1-seed31 | Map01 | rescue | official | instrumented | 31 | 1551 | 21.8 | 71.1 | 87.82 | disposal 100.0, health 94.2, time 44.9 | 10/10 disposed | all_disposed |
| 3 | p2-B2-b01-map01-s1-seed32 | Map01 | rescue | B0.1 (comparator) | instrumented | 32 | 2000 | 39.35 | 50.8 | 65.96 | disposal 80.0, health 89.8, time 0.0 | 8/10 disposed | step_limit |
| 4 | p2-B2-b1-map01-s1-seed32 | Map01 | rescue | official | instrumented | 32 | 1591 | 22.23 | 71.6 | 87.22 | disposal 100.0, health 95.2, time 40.9 | 10/10 disposed | all_disposed |
| 5 | p2-B3-b01-map01-s1-seed33 | Map01 | rescue | B0.1 (comparator) | instrumented | 33 | 2000 | 38.24 | 52.3 | 60.64 | disposal 70.0, health 93.2, time 0.0 | 7/10 disposed | step_limit |
| 6 | p2-B3-b1-map01-s1-seed33 | Map01 | rescue | official | instrumented | 33 | 1463 | 20.77 | 70.4 | 89.82 | disposal 100.0, health 95.4, time 53.7 | 10/10 disposed | all_disposed |
| 7 | p2-B4-b01-map01-far-seed34 | Map01 | rescue | B0.1 (comparator) | instrumented | 34 | 1904 | 36.86 | 51.7 | 80.92 | disposal 100.0, health 95.0, time 9.6 | 10/10 disposed | all_disposed |
| 8 | p2-B4-b1-map01-far-seed34 | Map01 | rescue | official | instrumented | 34 | 1424 | 19.8 | 71.9 | 90.80 | disposal 100.0, health 96.4, time 57.6 | 10/10 disposed | all_disposed |
| 9 | p2-B5-b01-map01-r01layout-seed35 | Map01 | rescue | B0.1 (comparator) | instrumented | 35 | 1978 | 40.09 | 49.3 | 78.84 | disposal 100.0, health 92.0, time 2.2 | 10/10 disposed | all_disposed |
| 10 | p2-B5-b1-map01-r01layout-seed35 | Map01 | rescue | official | instrumented | 35 | 1540 | 21.37 | 72.1 | 88.20 | disposal 100.0, health 95.0, time 46.0 | 10/10 disposed | all_disposed |
| 11 | p2-B6-b01-map02-s1-seed36 | Map02 | rescue | B0.1 (comparator) | instrumented | 36 | 1042 | 19.6 | 53.2 | 99.44 | disposal 100.0, health 97.2, time 100.0 | 10/10 disposed | all_disposed |
| 12 | p2-B6-b1-map02-s1-seed36 | Map02 | rescue | official | instrumented | 36 | 911 | 12.46 | 73.1 | 99.52 | disposal 100.0, health 97.6, time 100.0 | 10/10 disposed | all_disposed |
| 13 | p2-PB-b1-native-map01-s1-seed31 | Map01 | rescue | official | native | 31 | 1551 | 21.49 | 72.2 | 87.82 | disposal 100.0, health 94.2, time 44.9 | 10/10 disposed | all_disposed |
| 14 | p2-R1-r01-map01-seed41 | Map01 | place | R0.1 (comparator) | instrumented | 41 | 969 | 11.06 | 87.7 | 68.85 | map 92.4, time 100.0, wall 63.4 | 10 placed | map_submitted |
| 15 | p2-R1-r1-map01-seed41 | Map01 | place | official | instrumented | 41 | 940 | 9.48 | 99.2 | 72.67 | map 92.1, time 100.0, wall 83.0 | 10 placed | map_submitted |
| 16 | p2-R2-r01-map01-seed42 | Map01 | place | R0.1 (comparator) | instrumented | 42 | 940 | 10.62 | 88.5 | 66.56 | map 88.3, time 100.0, wall 61.9 | 10 placed | map_submitted |
| 17 | p2-R2-r1-map01-seed42 | Map01 | place | official | instrumented | 42 | 940 | 9.53 | 98.6 | 68.34 | map 92.1, time 100.0, wall 61.3 | 10 placed | map_submitted |
| 18 | p2-R3-r01-map01-seed43 | Map01 | place | R0.1 (comparator) | instrumented | 43 | 940 | 10.78 | 87.2 | 58.38 | map 75.8, time 100.0, wall 52.5 | 10 placed | map_submitted |
| 19 | p2-R3-r1-map01-seed43 | Map01 | place | official | instrumented | 43 | 967 | 9.8 | 98.6 | 65.77 | map 86.8, time 100.0, wall 61.9 | 10 placed | map_submitted |
| 20 | p2-R4-r01-map02-seed44 | Map02 | place | R0.1 (comparator) | instrumented | 44 | 2350 | 30.8 | 76.3 | 74.75 | map 97.6, time 100.0, wall 79.7 | 10 placed | map_submitted |
| 21 | p2-R4-r1-map02-seed44 | Map02 | place | official | instrumented | 44 | 2350 | 27.6 | 85.2 | 73.69 | map 97.9, time 100.0, wall 73.8 | 10 placed | map_submitted |
| 22 | p2-PR-r1-native-map01-seed41 | Map01 | place | official | native | 41 | 940 | 9.45 | 99.5 | 72.67 | map 92.1, time 100.0, wall 83.0 | 10 placed | map_submitted |
| 23 | p2-H1-b1-native-map03-s1-seed53 | Map03 | rescue | official | native | 53 | 5000 | 73.12 | 68.4 | 58.48 | disposal 70.0, health 82.4, time 0.0 | 7/10 disposed | step_limit |
| 24 | p2-H2-b1-native-map04-s1-seed55 | Map04 | rescue | official | native | 55 | 1413 | 21.47 | 65.8 | 99.20 | disposal 100.0, health 96.0, time 100.0 | 10/10 disposed | all_disposed |
| 25 | p2-H3-b1-native-map05-s1-seed54 | Map05 | rescue | official | native | 54 | 1185 | 17.37 | 68.2 | 99.40 | disposal 100.0, health 97.0, time 100.0 | 10/10 disposed | all_disposed |
| 26 | p2-H4-r1-native-map04-seed51 | Map04 | place | official | native | 51 | 2350 | 33.74 | 69.6 | 63.97 | map 95.1, time 100.0, wall 32.1 | 10 placed | map_submitted |
| 27 | p2-H5-r1-native-map05-seed52 | Map05 | place | official | native | 52 | 3384 | 45.9 | 73.7 | 74.53 | map 99.4, time 100.0, wall 74.2 | 10 placed | map_submitted |
| 28 | p2-X1-b1-map01-r1layout-seed56 | Map01 | rescue | official | instrumented | 56 | 1486 | 20.97 | 70.9 | 89.48 | disposal 100.0, health 96.0, time 51.4 | 10/10 disposed | all_disposed |
| 29 | p2-RES1-b1-map03-s1-seed53-instrumented | Map03 | rescue | official | instrumented | 53 | 5000 | 73.28 | 68.2 | 58.48 | disposal 70.0, health 82.4, time 0.0 | 7/10 disposed | step_limit |

### 7.2 Blue: B0.1 against B1 (six pairs, instrumented)

Heading error is the agent's own estimate against the truth one physics step
earlier (`devtools/simlab/run_metrics.py`); "carrying" uses the truth grasp
flag. Controller time is the mean per drone-step of `control()`, measured by
the evaluation wrapper.

| Pair | Scenario (seed) | B0.1 disposed, step, score | B1 disposed, step, score | Carrying heading error p50/p95 (deg), B0.1 ; B1 | Controller ms, B0.1 ; B1 | Steps/s, B0.1 ; B1 |
|---|---|---|---|---|---|---|
| B-1 | Map01 S1 (31) | 8/10, 2000, 66.88 | 10/10, 1551, 87.82 | 19.63/50.91 ; 3.01/7.91 | 1.37 ; 0.87 | 51.7 ; 71.1 |
| B-2 | Map01 S1 (32) | 8/10, 2000, 65.96 | 10/10, 1591, 87.22 | 13.77/41.33 ; 2.44/8.43 | 1.40 ; 0.86 | 50.8 ; 71.6 |
| B-3 | Map01 S1 (33) | 7/10, 2000, 60.64 | 10/10, 1463, 89.82 | 16.65/45.62 ; 2.44/9.26 | 1.34 ; 0.89 | 52.3 ; 70.4 |
| B-4 | Map01 far layout (34) | 10/10, 1904, 80.92 | 10/10, 1424, 90.80 | 15.66/44.41 ; 2.65/7.46 | 1.37 ; 0.86 | 51.7 ; 71.9 |
| B-5 | Map01 R0.1 layout (35) | 10/10, 1978, 78.84 | 10/10, 1540, 88.20 | 19.16/46.74 ; 2.60/6.80 | 1.46 ; 0.85 | 49.3 ; 72.1 |
| B-6 | Map02 S1 (36) | 10/10, 1042, 99.44 | 10/10, 911, 99.52 | 13.68/37.04 ; 2.88/8.10 | 1.34 ; 0.85 | 53.2 ; 73.1 |

* Disposed bombs: 53 of 60 (B0.1) against 60 of 60 (B1); B1 disposed more in
  three pairs and as many in the other three. Mean score 75.45 against
  90.56. B1 finished earlier in all six pairs.
* Free-flight heading error p95: 13.7-21.5 degrees (B0.1; part of it is the
  carrying error decaying after a delivery) against 5.4-9.0 (B1). Health
  component 89.8-97.2 against 94.2-97.6. No brain errors and no transport
  failures (releases without disposal) in any of the twelve rounds.
* Exact sign test, two-sided: score higher for B1 in 6 of 6 pairs, p = 0.031;
  disposed bombs higher in 3 of 3 untied pairs, p = 0.25. These are six pairs
  on two development maps with synthetic or Phase 1 layouts: a consistent
  direction, not an effect size that generalizes.
* Decision under the plan's rules: **B1 adopted**.

### 7.3 Red: R0.1 against R1 (four pairs, instrumented)

| Pair | Scenario (seed) | R0.1 map, wall, partial | R1 map, wall, partial | Map credit, R0.1 ; R1 | Controller ms, R0.1 ; R1 | Steps/s, R0.1 ; R1 |
|---|---|---|---|---|---|---|
| R-1 | Map01 (41) | 92.35, 63.4, 68.85 | 92.11, 83.0, 72.67 | 0.925 ; 0.921 | 1.71 ; 1.42 | 87.7 ; 99.2 |
| R-2 | Map01 (42) | 88.34, 61.9, 66.56 | 92.14, 61.3, 68.34 | 0.884 ; 0.922 | 1.69 ; 1.44 | 88.5 ; 98.6 |
| R-3 | Map01 (43) | 75.77, 52.5, 58.38 | 86.78, 61.9, 65.77 | 0.758 ; 0.868 | 1.71 ; 1.43 | 87.2 ; 98.6 |
| R-4 | Map02 (44) | 97.60, 79.7, 74.75 | 97.86, 73.8, 73.69 | 0.977 ; 0.981 | 1.91 ; 1.62 | 76.3 ; 85.2 |

* Mean map score 88.51 (R0.1) against 92.22 (R1); mean partial score 67.13
  against 70.12. R1 mapped more in three pairs and 0.24 points less in R-1;
  the largest difference is on the weakest R0.1 seed (R-3), the failure class
  Phase 1 reported (incomplete coverage on Map01). Every round of both arms
  submitted a map and placed all ten bombs; penalties stayed at or below
  0.002.
* Sign test on the map score: 3 of 4, p = 0.625. Not significant. Wall
  scores vary strongly within each arm (52-83) and are not interpreted.
* Decision under the plan's rules: **R1 adopted**.

### 7.4 Native competition-mode parity

| Check | Instrumented | Native |
|---|---|---|
| B1, Map01 S1, seed 31 | all bombs at step 1551, 87.82, health 94.2, 71.1 steps/s | identical outcome and score, 72.2 steps/s |
| R1, Map01, seed 41 | submitted at step 940, map 92.114, wall 83.046, 99.2 steps/s | identical outcome and scores, 99.5 steps/s |

The round outcomes were identical in both checks. This shows that the
evaluation wrapper did not change these two rounds; it does not show that it
can never matter, because collision damage depends on wall-clock time (D17)
and the wrapper costs about 1 % of wall time here.

### 7.5 Held-out and cross-role rounds (final candidates)

| Round | Map | Agent, mode | Outcome | Score | Steps/s |
|---|---|---|---|---|---|
| H-1 | Map03 (never used for blue) | B1, native | 7/10 disposed at the 5000-step limit, health 82.4 | 58.48 | 68.4 |
| H-2 | Map04 | B1, native | 10/10 at step 1413 | 99.20 | 65.8 |
| H-3 | Map05 | B1, native | 10/10 at step 1185 | 99.40 | 68.2 |
| H-4 | Map04 (never used for red) | R1, native | map 95.11, wall 32.1, all bombs placed | partial 63.97 | 69.6 |
| H-5 | Map05 (never used for red) | R1, native | map 99.40, wall 74.2, submitted at step 3384 | partial 74.53 | 73.7 |
| X-1 | Map01, layout placed by R1 in R-1 | B1, instrumented | 10/10 at step 1486 | 89.48 | 70.9 |
| Reserve 1 | Map03, as H-1 | B1, instrumented | identical to H-1 | 58.48 | 68.2 |

X-1 is an adversarial diagnostic, not a bomb-difficulty score: against an
agent like B1, R1's layout would rate `min(100, 2 x (100 - 89.48)) = 21.0`;
the organizer's reference agents are unknown. H-4's low wall score (blast
radius 135 px on Map04) is not explained yet; placement was complete.

### 7.6 Failure analysis: B1 on Map03 (H-1 and the reserve round)

The instrumented repetition reproduced H-1 exactly. Carry episodes
(`devtools/simlab/carry_episodes.py`, disposal-center box from the evaluation
fixture):

* All ten bombs were grasped; seven were delivered after 176-393 steps of
  carrying. Three drones grasped a bomb at steps 71, 159 and 233 and still
  held it at step 5000, ending 46-69 px from the disposal-center box: two
  south of it, one west of it. Their carrying heading error p95 was 8-9
  degrees, so heading estimation is not involved.
* On Map03 the disposal center (x -125.5 to 85.5, y -102.5 to 8.5) is walled
  on its south side (wall at y -110 to -104 across x 60-140) and its west side
  (x -133 to -126). The three drones were on the far side of those walls.
* Mechanism (from the code): the stand-off goals of the carry field lie
  outward of the observed center points on every side, including the walled
  ones, so the field can lead to the wrong side; the delivery step then aims
  at a center surface point within 50 px, where it skips the line-of-sight
  test and pushes into the wall; delivery timeouts choose other points on the
  same blocked side.
* Offline, after H-1, Map03 also fails for B0.1 (9 of 10 in one seed), so this
  is a Phase 1 weakness that B1 did not introduce. The low health (82.4) comes
  from the stuck drones and from idle drones crowding near the center, which
  lies 50-180 px from the start area.
* Not fixed in Phase 2: a fix would need real-simulator validation that the
  remaining budget could not provide. It is the first recommendation for the
  next phase.

### 7.7 Border-rendering ablation

`devtools/simlab/border_ablation.py` re-rendered the submitting drone's
merged wall evidence, saved in the eight instrumented red rounds, with the
current renderer with border bands on and off, and scored both with the
official scorer against the walls-only truth. Consistency check: re-scoring
each saved submitted map reproduced the round's official map score in all
eight rounds.

| Round | Official map, R0.1 ; R1 | Bands on, R0.1 ; R1 | Bands off, R0.1 ; R1 | Charged pixels, R0.1 ; R1 (on = off) |
|---|---|---|---|---|
| R-1 | 92.35 ; 92.11 | 91.81 ; 92.11 | 73.39 ; 85.31 | 989 ; 289 |
| R-2 | 88.34 ; 92.14 | 87.79 ; 92.14 | 58.75 ; 72.23 | 828 ; 871 |
| R-3 | 75.77 ; 86.78 | 75.06 ; 86.78 | 54.66 ; 63.47 | 694 ; 158 |
| R-4 | 97.60 ; 97.86 | 97.59 ; 97.86 | 95.47 ; 96.24 | 1977 ; 3052 |

* Border bands contribute 6.8-29.6 map points on Map01 and 1.6-2.1 on Map02,
  and no penalized pixel: the charged pixel count is the same with and
  without them in every round, so on these maps the border inference made no
  false positive beyond the 10 px tolerance.
* The R0.1 columns show the cost of the R1 change that clips each band to its
  observed span (12 px margin): 0.01-0.70 points on maps whose borders have no
  gaps. That is the price of never filling a border gap; it is accepted
  because the official maps are not known.
* This is an ablation on the supplied maps; nothing was tuned on it.

### 7.8 Runtime

* **Throughput** on the server (one simulation process, node 0 CPUs, two
  threads per library, one GPU through EGL): B1 70.4-73.1 steps/s on Map01
  and Map02 with 10 drones (instrumented), 72.2 native; B0.1 49.3-53.2. On the
  larger held-out maps B1 ran at 65.8-68.4 steps/s natively. The 70 steps/s
  objective is met on Map01 and Map02, not on Map03-Map05. Nothing here
  measures the organizer's evaluator.
* **Controller against simulator** (B-1): B1 spent 13.5 s of 21.8 s wall time
  in `control()` (8.7 ms per step for ten drones), leaving 8.3 s (5.4 ms per
  step) for the simulator, sensors and the logger; B0.1 spent 27.4 s of 38.7 s
  (13.7 ms per step), leaving 11.3 s (5.6 ms per step).
* **What changed** (profiles of the offline brains, `offline_eval
  --profile`): the geodesic field was not the dominant cost assumed in Phase 1
  (about 9 %); the measured hot spots were visibility tests in path lookahead
  that recomputed the whole occupancy mask (25 %), sorting in the scan update
  (19 %), re-merging of repeated bomb digests (8 %) and the disposal-center
  percentile box (5 %). The scan, visibility, lookahead and masking changes
  reproduce the frozen Phase 1 outputs exactly on randomized inputs
  (`devtools/tests/test_phase2_blue.py`); offline brain time per drone-step
  fell from 1.27-1.34 to 0.76-0.80 ms on the server (attempt A002; the first
  profile, made on the workstation before the remote-first directive, gave
  1.39 and 0.81).
* **Memory and messages**: peak resident memory of the offline process is
  unchanged (94 MB blue with 10 drones, 172 MB red with 5 drones, both
  generations). Messages are passed in-process as Python objects, so there is
  no serialization; the coverage summary (at most 234 bytes, every 10 steps)
  and its merge cost less than 2.3 % of red brain time (no coverage function
  among the 35 most expensive entries; the 35th took 0.18 s of 7.9 s).

### 7.9 Offline development signals (not competition results)

Offline runs used the approximate simulator with the probe's carried-rotation
spread (`--carry-ratio-sigma 0.36`), on the server, through
`devtools/simlab/offline_batch.sh` (one attempt directory per run, per-job
status, nothing overwritten).

| Attempt | Content | Status and result |
|---|---|---|
| A001 | blue Map01, 6 jobs | **failed**: the ad hoc script called `python`, which is not on the server's non-interactive `PATH`; no job ran; an output filter hid the failure. Recorded, not counted. |
| A002 | blue Map01 S1 seeds 1-3, B0.1 and B1 | B1 10/10 at steps 1388-1570, B0.1 7-8/10; carrying heading p95 7-8 against 35-45 degrees |
| A003-A006 | red Map01 seeds 1-3 and Map02 seeds 1-2: R0.1, sharing on, off, coverage only, commitment only, R1 | dropping frontier cells in covered blocks lowered Map02 (92.7, 96.0 against 97.7, 97.8); R1 (frontier kept) not worse |
| A007 | blue Map01 S1 seed 1 with the R1-era code | identical to A002 (blue unchanged) |
| A008 | blue Map01 S1 seeds 1-3 with sharing on | completion at 1564/1803/1457 against 1388/1494/1570; not adopted for blue |
| A009 | blue Map03 S1 seeds 1-2, B1 and B0.1 (after H-1) | B1 9/10 and 7/10, B0.1 9/10; the fourth job was cut off by the 598 s limit and has no result |
| A010 | profiles and peak memory, blue and red, both generations | section 7.8 |

The offline simulator does not reproduce the real Map01 red coverage failure
(offline map scores near 92 for both R0.1 and R1), so offline red results bound
regressions but cannot show the benefit.

## 8. Phase 3 (2026-10-14): delivery to walled disposal centers, red placement

The plan, its arms, scenarios, seeds, metrics and decision rules were frozen
in [PHASE3_PLAN.md](PHASE3_PLAN.md) before the first Phase 3 round; three
dated addenda record the blue decision, the freezing of R2 before any red
round, and the red decision. Phase 3 had its own budget of 30 rounds; **28
were started, none interrupted, all from clean commits; both reserve rounds
were left unused.** Earlier phases' rounds are unchanged and not counted.

Code of the rounds: rounds 1-21 at `bb569c3` (B2 agent code identical to
`9f0cd91`), rounds 22-28 at `5e0803d` (R2 agent code identical to
`e00c54e`; the blue files are byte-identical to `bb569c3`). B1 and R1 rows
ran the frozen comparator package `devtools/comparators/p2_540054b`
(byte-identical to `540054b`). Its first round (M-1, B1) reproduced the
Phase 2 H-1 and reserve rounds exactly (7 of 10 bombs, score 58.48, health
82.4), which checks the comparator end to end.

### 8.1 Run ledger (Phase 3)

Generated with `python -m devtools.simlab.summarize <ledger> --phase phase3`;
comparator rows name their package. "official" is the submission entry point
at the round's commit (B2 for rescue, R2 for place in rounds 22-28).

| # | label | map | mode | agent | run mode | seed | steps | wall s | steps/s | score | components | bombs | termination |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | p3-M1-b1-map03-s1-seed53 | Map03 | rescue | p2_540054b (comparator) | instrumented | 53 | 5000 | 73.07 | 68.4 | 58.48 | disposal 70.0, health 82.4, time 0.0 | 7/10 disposed | step_limit |
| 2 | p3-M1-b2-map03-s1-seed53 | Map03 | rescue | official | instrumented | 53 | 791 | 10.96 | 72.2 | 99.44 | disposal 100.0, health 97.2, time 100.0 | 10/10 disposed | all_disposed |
| 3 | p3-M2-b1-map03-s2-seed61 | Map03 | rescue | p2_540054b (comparator) | instrumented | 61 | 5000 | 73.36 | 68.2 | 53.24 | disposal 60.0, health 86.2, time 0.0 | 6/10 disposed | step_limit |
| 4 | p3-M2-b2-map03-s2-seed61 | Map03 | rescue | official | instrumented | 61 | 1223 | 16.51 | 74.1 | 99.16 | disposal 100.0, health 95.8, time 100.0 | 10/10 disposed | all_disposed |
| 5 | p3-M3-b1-map03-s3-seed62 | Map03 | rescue | p2_540054b (comparator) | instrumented | 62 | 5000 | 74.43 | 67.2 | 63.88 | disposal 80.0, health 79.4, time 0.0 | 8/10 disposed | step_limit |
| 6 | p3-M3-b2-map03-s3-seed62 | Map03 | rescue | official | instrumented | 62 | 716 | 10.08 | 71.1 | 99.28 | disposal 100.0, health 96.4, time 100.0 | 10/10 disposed | all_disposed |
| 7 | p3-M4-b1-map03-s4-seed63 | Map03 | rescue | p2_540054b (comparator) | instrumented | 63 | 5000 | 73.06 | 68.4 | 65.44 | disposal 80.0, health 87.2, time 0.0 | 8/10 disposed | step_limit |
| 8 | p3-M4-b2-map03-s4-seed63 | Map03 | rescue | official | instrumented | 63 | 915 | 12.75 | 71.8 | 99.76 | disposal 100.0, health 98.8, time 100.0 | 10/10 disposed | all_disposed |
| 9 | p3-M5-b1-map03-far-s5-seed64 | Map03 | rescue | p2_540054b (comparator) | instrumented | 64 | 5000 | 76.9 | 65.0 | 69.20 | disposal 90.0, health 76.0, time 0.0 | 9/10 disposed | step_limit |
| 10 | p3-M5-b2-map03-far-s5-seed64 | Map03 | rescue | official | instrumented | 64 | 673 | 9.26 | 72.7 | 99.56 | disposal 100.0, health 97.8, time 100.0 | 10/10 disposed | all_disposed |
| 11 | p3-G1-b1-map01-s1-seed71 | Map01 | rescue | p2_540054b (comparator) | instrumented | 71 | 1667 | 23.14 | 72.0 | 85.26 | disposal 100.0, health 93.0, time 33.3 | 10/10 disposed | all_disposed |
| 12 | p3-G1-b2-map01-s1-seed71 | Map01 | rescue | official | instrumented | 71 | 1430 | 19.69 | 72.6 | 91.00 | disposal 100.0, health 98.0, time 57.0 | 10/10 disposed | all_disposed |
| 13 | p3-G2-b1-map02-s2-seed72 | Map02 | rescue | p2_540054b (comparator) | instrumented | 72 | 758 | 10.68 | 71.0 | 99.68 | disposal 100.0, health 98.4, time 100.0 | 10/10 disposed | all_disposed |
| 14 | p3-G2-b2-map02-s2-seed72 | Map02 | rescue | official | instrumented | 72 | 699 | 9.98 | 70.1 | 99.88 | disposal 100.0, health 99.4, time 100.0 | 10/10 disposed | all_disposed |
| 15 | p3-G3-b1-map04-s2-seed73 | Map04 | rescue | p2_540054b (comparator) | instrumented | 73 | 797 | 13.18 | 60.5 | 99.88 | disposal 100.0, health 99.4, time 100.0 | 10/10 disposed | all_disposed |
| 16 | p3-G3-b2-map04-s2-seed73 | Map04 | rescue | official | instrumented | 73 | 820 | 13.77 | 59.6 | 99.76 | disposal 100.0, health 98.8, time 100.0 | 10/10 disposed | all_disposed |
| 17 | p3-G4-b1-map05-s2-seed74 | Map05 | rescue | p2_540054b (comparator) | instrumented | 74 | 646 | 9.94 | 65.0 | 99.64 | disposal 100.0, health 98.2, time 100.0 | 10/10 disposed | all_disposed |
| 18 | p3-G4-b2-map05-s2-seed74 | Map05 | rescue | official | instrumented | 74 | 731 | 10.81 | 67.6 | 99.56 | disposal 100.0, health 97.8, time 100.0 | 10/10 disposed | all_disposed |
| 19 | p3-N1-b2-native-map03-s1-seed53 | Map03 | rescue | official | native | 53 | 791 | 11.0 | 71.9 | 99.44 | disposal 100.0, health 97.2, time 100.0 | 10/10 disposed | all_disposed |
| 20 | p3-N2-b2-native-map01-far-s3-seed75 | Map01 | rescue | official | native | 75 | 1711 | 23.64 | 72.4 | 84.42 | disposal 100.0, health 93.2, time 28.9 | 10/10 disposed | all_disposed |
| 21 | p3-N3-b2-native-map01-r01layout-seed76 | Map01 | rescue | official | native | 76 | 1336 | 18.6 | 71.8 | 92.24 | disposal 100.0, health 94.8, time 66.4 | 10/10 disposed | all_disposed |
| 22 | p3-R1-r1-map04-seed81 | Map04 | place | p2_540054b (comparator) | instrumented | 81 | 2350 | 34.72 | 67.7 | 64.12 | map 94.9, time 100.0, wall 33.3 | 10 placed | map_submitted |
| 23 | p3-R1-r2-map04-seed81 | Map04 | place | official | instrumented | 81 | 2350 | 34.56 | 68.0 | 71.05 | map 95.9, time 100.0, wall 65.4 | 10 placed | map_submitted |
| 24 | p3-R2-r1-map01-seed82 | Map01 | place | p2_540054b (comparator) | instrumented | 82 | 940 | 9.46 | 99.3 | 67.88 | map 91.5, time 100.0, wall 60.8 | 10 placed | map_submitted |
| 25 | p3-R2-r2-map01-seed82 | Map01 | place | official | instrumented | 82 | 940 | 9.5 | 98.9 | 72.06 | map 91.1, time 100.0, wall 82.6 | 10 placed | map_submitted |
| 26 | p3-R3-r1-map05-seed83 | Map05 | place | p2_540054b (comparator) | instrumented | 83 | 3384 | 46.3 | 73.1 | 73.41 | map 98.9, time 100.0, wall 69.8 | 10 placed | map_submitted |
| 27 | p3-R3-r2-map05-seed83 | Map05 | place | official | instrumented | 83 | 3384 | 46.57 | 72.7 | 77.40 | map 99.4, time 100.0, wall 88.4 | 10 placed | map_submitted |
| 28 | p3-N4-r2-native-map04-seed84 | Map04 | place | official | native | 84 | 2350 | 33.92 | 69.3 | 71.11 | map 96.2, time 100.0, wall 65.0 | 10 placed | map_submitted |

### 8.2 Map03 delivery failure: mechanism

The truth log of the Phase 2 reserve round (instrumented repetition of H-1)
was re-read with the agents' own targets (`carry_episodes.py` and an ad hoc
per-drone trace). On Map03 the disposal center (x -125.5 to 85.5, y -102.5 to
8.5) has a 6 px wall centered 4.5 px below its south face (y -107, x -130 to
198) and another 4.5 px west of its west face (x -130), so the gaps are
about 1.5 px.

* The three stuck carriers spent 4127-4402 carrying steps in the carry
  state, steering towards field goals at (103.5, -133) and (103.5, -125)
  (two drones, south of the wall) and (-144.5, 3) (one drone, west of the
  wall), with occasional terminal pushes at center points within 50 px.
* B1 builds its goals 32 px outward from sampled center surface points,
  along the direction from the center's middle. Near a corner of a flat
  center that direction points diagonally, and the goal lands behind the
  wall. The planner then sets the goal cell traversable even when it is an
  obstacle, so a goal in or next to a wall connects both sides of it; the
  carriers reached their goals and stayed there.
* The terminal push skipped the line-of-sight test within 50 px and timeouts
  re-aimed at other points on the same side.
* The heading estimate was not involved (carrying error p95 8-9 degrees).

Two facts of the supplied simulator make a better goal possible: the semantic
sensor drops wall hits only after ray casting, so walls stop its rays and
every detection of the center is in line of sight; and every drone sees the
center from the start area on the supplied maps.

### 8.3 Development evidence before the rounds (offline approximation)

Offline attempts ran in a scratch worktree of the server clone through
`devtools/simlab/delivery_batch.sh`, `offline_batch.sh` or one-line batch
loops under `remote_cpu.sh` (one attempt directory per batch, per-job exit
status, nothing overwritten). They are development signals, not simulator
rounds.

| Attempt | Content | Status and result |
|---|---|---|
| P3-A001 | B1 on the synthetic `corner` and `open` fixtures, first fixture version (6 px gaps) | corner 1, 2, 4 of 6 bombs at 3000 steps; open 6 of 6 at step 304 |
| P3-A002 | B2 on the same | corner 6 of 6 at steps 793-904; open at step 281 |
| P3-A003 | final fixture (2 px gaps, as on Map03), B1 and B2, corner seeds 1-3, open seeds 1-2 | B1 corner 1, 2, 1 of 6 at 3000; B2 6 of 6 at 769-853; open B1 304, 207 and B2 281, 294 |
| P3-A004 | teleported carrier behind a wall, 4 positions x seeds 3-7 | B1 0 of 20 delivered within 1200 steps; B2 20 of 20 in 173-297 steps |
| P3-A005, A006 | B1, B2 on the real-map fixtures (layouts s1; Map01 and Map03 seeds 1-2) | Map03: B1 9 and 7 of 10 at 5000, B2 10 of 10 at steps 627 and 677; elsewhere 10 of 10 for both (B1 736-1494, B2 674-1402 steps) |
| P3-A007 | profiles of B1 and B2 (Map01, Map03, Map04) | section 8.7 |
| P3-A008 | red wall-coverage diagnosis, R1, Map01-Map05 | section 8.6 |
| P3-A009 | red diagnosis rerun | **deferred** by the occupancy gate (node 0 busy); the scratch worktree had also not been updated; nothing ran |
| P3-A010 | R1 against R2 coordination only, six configurations | section 8.6 |
| P3-A011 | R2 with the first solid-interior version, five maps, two R1 references | section 8.6 |
| P3-A012 | rerun with solid-inference measurement | **failed**: the scratch worktree had been reset without its overlay; all six jobs exited 1 (module missing); nothing used |
| P3-A013 | P3-A012 rerun, including two repetitions of two configurations | repetitions identical; solid precision, section 8.6 |
| P3-A014 | final R2, eight configurations | section 8.6 |

Regression tests: `devtools/tests/test_blue_delivery.py` (20 tests; the
closed-loop cases run the frozen B1 on the same inputs and require it to
fail) and `devtools/tests/test_red_r2.py` (10 tests). Each mechanism was
mutation-checked by planting the old behaviour back: B1's 50 px push without
check, B1's unchecked outward goals, no progress watch, idle drones at the
start area; for red, ignoring relayed claims, no target re-validation, no
solid interiors, interiors next to free space counted as enclosed, and no
disposal-center exclusion. Each planted mutation made at least one test
fail; the first version of the idle test called the helper directly and did
not catch its mutation, and was rewritten to go through the waiting
behaviour.

### 8.4 Blue: B1 against B2 (nine pairs, instrumented)

Delivery failures are carry episodes that ended in a drop or were still held
at the end; in a round that ended because the last bomb was disposed, the
truth log stops before that release, and that episode counts as a disposal
(PHASE3_PLAN.md, addendum 1). Retries: B1 `deliver_retries`, B2
`approach_failures`, summed over the drones.

| Pair | Scenario (seed) | B1 disposed, step, score | B2 disposed, step, score | Delivery failures, B1 ; B2 | Health, B1 ; B2 | Retries, B1 ; B2 |
|---|---|---|---|---|---|---|
| M-1 | Map03 s1 (53) | 7/10, 5000, 58.48 | 10/10, 791, 99.44 | 3 ; 0 | 82.4 ; 97.2 | 86 ; 0 |
| M-2 | Map03 s2 (61) | 6/10, 5000, 53.24 | 10/10, 1223, 99.16 | 4 ; 0 | 86.2 ; 95.8 | 97 ; 0 |
| M-3 | Map03 s3 (62) | 8/10, 5000, 63.88 | 10/10, 716, 99.28 | 2 ; 0 | 79.4 ; 96.4 | 69 ; 0 |
| M-4 | Map03 s4 (63) | 8/10, 5000, 65.44 | 10/10, 915, 99.76 | 2 ; 0 | 87.2 ; 98.8 | 71 ; 0 |
| M-5 | Map03 far s5 (64) | 9/10, 5000, 69.20 | 10/10, 673, 99.56 | 1 ; 0 | 76.0 ; 97.8 | 37 ; 0 |
| G-1 | Map01 s1 (71) | 10/10, 1667, 85.26 | 10/10, 1430, 91.00 | 0 ; 0 | 93.0 ; 98.0 | 0 ; 0 |
| G-2 | Map02 s2 (72) | 10/10, 758, 99.68 | 10/10, 699, 99.88 | 0 ; 0 | 98.4 ; 99.4 | 0 ; 0 |
| G-3 | Map04 s2 (73) | 10/10, 797, 99.88 | 10/10, 820, 99.76 | 0 ; 0 | 99.4 ; 98.8 | 0 ; 0 |
| G-4 | Map05 s2 (74) | 10/10, 646, 99.64 | 10/10, 731, 99.56 | 0 ; 0 | 98.2 ; 97.8 | 0 ; 0 |

* Block M (the Map03 failure family): 38 of 50 bombs (B1) against 50 of 50
  (B2); B2 more in all five pairs (sign test p = 0.0625); mean score 62.05
  against 99.44; full completion 0 of 5 against 5 of 5. Stuck events in
  block M: 211-305 per round for B1, 8-16 for B2.
* Block G (development and alternative layouts): equal disposals in all
  four pairs; mean score 96.12 against 97.55; B2 finished earlier in two
  pairs and later in two (by 23 and 85 steps; sign test on the score p = 1).
* No round crashed, reached the wall-clock limit, reported a brain error or
  lost a drone. The B2 progress watch never fired in these rounds
  (`approach_failures` 0); in G-4 the radial fallback goals were computed 6
  times, when a drone had no supported witnessed stand-off.
* Decision under the plan's rules: **B2 adopted** (addendum 1).

### 8.5 Native competition-mode confirmation (blue)

| Round | Configuration | Outcome |
|---|---|---|
| N-1 | Map03 s1, seed 53 (M-1) | identical to the instrumented B2 arm: all bombs at step 791, 99.44, health 97.2; 71.9 against 72.2 steps/s |
| N-2 | Map01 far s3, seed 75 (held out) | 10/10 at step 1711, 84.42 |
| N-3 | Map01, layout placed by R0.1 in Phase 1, seed 76 | 10/10 at step 1336, 92.24 |

### 8.6 Red: wall-coverage diagnosis and R1 against R2

**Diagnosis** (`devtools/simlab/red_wall_diagnosis.py`, offline). For each
run it scores, with the official `compute_wall_score` against the walls-only
truth, the bombs actually placed and three greedy references: on the team's
merged wall map at the start of the placement window (what the agents knew),
on the map at submission, and on the truth walls. R1 lost wall score in two
places:

| Map (seed) | R1 actual | Greedy, team map at placement | Greedy, truth | Recall of weighted truth walls at placement | Blast redundancy | Pairs closer than one radius |
|---|---|---|---|---|---|---|
| Map01 (1) | 76.89 | 95.87 | 100.00 | 0.934 | 1.327 | 5 |
| Map02 (1) | 75.53 | 95.04 | 99.86 | 0.931 | 1.349 | 3 |
| Map03 (1) | 73.72 | 97.41 | 100.00 | 0.982 | 1.352 | 4 |
| Map04 (1) | 34.77 | 47.41 | 97.65 | 0.602 | 1.292 | 2 |
| Map04 (2) | 36.56 | 46.80 | 97.65 | 0.605 | 1.306 | 2 |
| Map05 (1) | 56.48 | 89.53 | 98.50 | 0.957 | 1.430 | 6 |

(P3-A008.) The placement procedure lost 10-33 points against a greedy
placement on the agents' own knowledge, with overlapping blasts; in the six
R1 runs of P3-A010, 21 of 21 placements within one blast radius of an
earlier bomb were made by a drone that already knew that bomb, through a
target kept after the bomb was gossiped or, in 6 of the 21, an emergency
drop near the rally point. On Map04 the team map itself held only about 60 % of the
weighted truth walls although the submitted map scored about 95: the four
solid boxes of Map04 hold about 74,000 of its 148,805 interior wall pixels,
and the lidar maps only their outlines.

**R2** (PHASE3_PLAN.md, addendum 2) changes the placement only: targets are
given up when a known bomb lies within 0.8 blast radii, claims are relayed,
and enclosed unknown regions of a drone's occupancy grid (not the disposal
center) weigh as solid wall. Offline wall scores, single runs:

| Map (seed) | R1 | R2 coordination only (P3-A010) | R2 final (P3-A014) | Solid inference at placement: area px, precision |
|---|---|---|---|---|
| Map01 (1) | 76.89 | 85.25 | 85.25 | 384, 0.33 |
| Map01 (2) | 62.23 | 83.14 | 83.14 | 256, 0.00 |
| Map02 (1) | 76.26 | | 94.36 | 3136, 0.79 |
| Map03 (1) | 73.72 | 95.58 | 95.58 | 640, 0.41 |
| Map04 (1) | 34.77 | 39.63 | 60.85 | 36480, 0.98 |
| Map04 (2) | 36.56 | | 67.10 | 34176, 0.95 |
| Map05 (1) | 54.74 | 80.03 | 93.81 | 6656, 0.83 |
| Map05 (2) | 80.88 | 79.25 | 77.57 | 6336, 0.81 |

R1 values from P3-A010 and P3-A011 (comparator package). The first solid
version (P3-A011, without the disposal-center exclusion) took the center's
interior as solid on Map03 and Map05 (precision 0.05 and 0.18) and lowered
those two maps by about 5 points against the coordination-only version; a
picture of the inferred cells against the truth showed the disposal center,
and the exclusion was added. Identical code produced identical results in
two repetitions (P3-A013), but R1 on Map05 seed 1 gave 56.48 in P3-A008 and
54.74 in P3-A010 (cause not found), so differences of a few points between
single offline runs are not interpreted.

**Real-simulator pairs** (instrumented; partial score = map 50 %, wall 20 %,
time 10 %, maximum 80):

| Pair | Scenario (seed) | R1 map, wall, partial | R2 map, wall, partial | R2 targets given up |
|---|---|---|---|---|
| R-1 | Map04 (81) | 94.92, 33.28, 64.12 | 95.93, 65.42, 71.05 | 2 |
| R-2 | Map01 (82) | 91.45, 60.76, 67.88 | 91.10, 82.58, 72.06 | 1 |
| R-3 | Map05 (83) | 98.91, 69.78, 73.41 | 99.42, 88.44, 77.40 | 5 |

* Mean wall score 54.61 (R1) against 78.81 (R2), higher in 3 of 3 pairs
  (p = 0.25); mean map score 95.10 against 95.48; mean partial score 68.47
  against 73.50. Every round of both arms submitted at the same step and
  placed all 10 bombs; no brain errors. R2 is adopted (addendum 3).
* N-4 (native, R2, Map04 seed 84): map 96.23, wall 64.96, partial 71.11.
  For orientation only (different seed, not paired): the Phase 2 native R1
  round on Map04 (seed 51) scored wall 32.09.
* These three maps were used offline to develop R2; the pairs are
  development evidence, not a held-out test.

### 8.7 Runtime

* **Throughput** (instrumented, one simulation process, node 0 CPUs, two
  threads per library, one GPU through EGL): B2 59.6-74.1 steps/s with 10
  drones (median over the nine pairs 71.8), B1 60.5-72.0 (median 68.2) in
  the same configurations; the lowest values on Map04. Native blue rounds
  71.8-72.4. Red: R2 68.0-98.9 steps/s with 5 drones, R1 67.7-99.3.
* **Controller** (mean per drone-step, instrumented): B2 0.82-1.14 ms, B1
  0.85-1.12 ms; R2 1.43-2.18 ms, R1 1.43-2.17 ms. **Worst case** (maximum
  `control()` time per round): B2 28.96-33.16 ms, B1 26.61-30.89 ms; R2
  41.13-77.82 ms, R1 39.85-76.91 ms. The extra 2-3 ms of B2's maximum was not
  profiled.
* **Profile** (P3-A007, offline, B2): no function of `delivery.py` is among
  the 35 most expensive entries by cumulative time (the last of them took
  0.20-0.32 s of 7.4-14.8 s of brain time); the geodesic field remains the
  largest single cost, as in B1.
  Nothing was optimized for B2, and no collision check was removed.
* Nothing here measures the organizer's evaluator.
