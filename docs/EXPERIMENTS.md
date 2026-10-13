# Experiments: methodology, benchmarks and Phase 1 results

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
4. **No sharing of searched areas (open).** In round 9 all five red drones
   spent the first 500 steps in the left compartment (each covered about
   2700-3000 px of path) while the right compartment stayed unmapped.
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
