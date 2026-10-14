# Phase 3 report: delivery robustness and adversarial validation

Dates are business dates in UTC+8. Phase 3 ran on 2026-10-14. Details:
[PHASE3_PLAN.md](PHASE3_PLAN.md) (frozen plan and three dated addenda),
[EXPERIMENTS.md](EXPERIMENTS.md) section 8 (ledger, results, analyses),
[AGENT_DESIGN.md](AGENT_DESIGN.md) (current design and open defects).

## 1. Disposition

**PASS** for the phase objective. The Map03 delivery failure is removed in
the real simulator: on five paired Map03 configurations the blue candidate
B2 disposed 50 of 50 bombs against 38 of 50 for the frozen B1, finishing
every round, with no regression on four paired configurations of the other
maps and an exact native-mode parity check. A red candidate R2, built on a
diagnosed mechanism, raised the wall score in three of three real pairs.
Both adoptions follow rules frozen before the rounds. The evidence is small
(five, four and three pairs; no block can reach p < 0.05 with a sign test),
Map03 and the red maps are development scenarios, and the limits in section
7 remain.

## 2. Starting state, model and environment

* Start: workstation HEAD = `origin/main` = `540054b`, clean; server clone at
  `540054b`, clean. Private working notes were kept in this project's own
  ignored directory; no other project's notes were written.

* All Python ran on the remote server's project environment (CPython
  3.11.17, absolute interpreter path), pinned to NUMA node 0 behind the
  occupancy gate; nothing ran on NUMA node 1. When another project of the
  owner occupied node 0, the gate deferred the work (attempt P3-A009) and it
  waited. The wrappers now re-run themselves under `taskset`, so the gate and
  helper processes are pinned too.

## 3. Frozen comparators

`devtools/comparators/p2_540054b` holds B1 and R1: the 15 agent files of
`src/swarm_rescue/solutions/eod/` at `540054b`, byte-identical (SHA-256 in
`p2_540054b.json`, checked against Git history by
`devtools/tests/test_comparators.py`). It imports no module of the current
agent package; its only shared dependencies are the simulator framework,
NumPy, OpenCV and scikit-image, identical for both arms. The Phase 1
comparator `p1_ddc6897` is unchanged. End-to-end check: the first Phase 3
round (B1 on the Phase 2 H-1 configuration) reproduced H-1 exactly.

## 4. Map03 delivery: root cause and B2

* **Root cause** (EXPERIMENTS.md 8.2, from the Phase 2 reserve round's truth
  log). B1 places its carry goals 32 px outward from the disposal center's
  middle; near a corner of a flat center such a goal lands behind a wall that
  runs about 1.5 px from the face, and the planner makes a goal cell
  traversable even inside an obstacle, so the field leads to the wrong side
  and ends there. The terminal push skipped the line-of-sight test within
  50 px. The three stuck carriers sat at such goals for over 4000 steps.
* **B2** (`delivery.py`, `blue.py`). Walls stop semantic rays, so each
  detection of the center witnesses a free segment; stand-offs are taken on
  those segments (36 px out), re-checked against the map, and used as carry
  goals. The terminal push requires a line-of-sight certificate at every
  distance. A carrier that stops progressing near the center excludes that
  approach area for 600-2400 steps and re-plans; it never pushes towards a
  point through whatever lies between. Idle drones wait at least 130 px from
  the center on a reachable known free cell. No map coordinates, no truth
  data; only the drone's estimates, detections and map.
* **Tests** (`devtools/tests/test_blue_delivery.py`, 20). Synthetic
  geometry (`devtools/simlab/delivery_fixtures.py`, a center with two faces
  2 px from long walls; not Map03's coordinates): goal validity against the
  truth walls; B1 builds goals behind walls, B2 none; a carrier within 50 px
  behind a wall (B1 pushes into it, B2 re-routes); a route around a corner
  without crossing walls; unreachable, contradicted and stale observations;
  sparse maps; progress watch, expiry and escalation; idle positions;
  grasper held through re-planning; closed-loop carriers behind the south and
  west walls (the frozen B1 fails in every case, B2 delivers); a parked
  teammate on the approach. Four planted regressions each fail at least one
  test. Offline, B1 failed 20 of 20 teleported-carrier cases that B2
  delivered in 173-297 steps.

## 5. Blue results (real simulator)

| Block | B1 | B2 |
|---|---|---|
| M: Map03 family, 5 pairs | 38/50 disposed, 0/5 completed, mean score 62.05, 12 delivery failures | 50/50, 5/5 completed (steps 673-1223), mean 99.44, 0 failures |
| G: Map01-Map05 layouts, 4 pairs | 40/40, mean 96.12 | 40/40, mean 97.55 |
| Native (N-1 to N-3) | | N-1 identical to its instrumented twin; N-2 (held-out layout) and N-3 (layout placed by R0.1 in Phase 1) 10/10 |

Health in block M rose from 76.0-87.2 to 95.8-98.8; stuck events per round
fell from 211-305 to 8-16. No round crashed, hit the wall-clock limit,
reported a brain error or lost a drone. The congestion change (idle drones
away from the center) is part of B2 and was not evaluated separately.

## 6. Red: diagnosis and R2

* **Diagnosis** (`devtools/simlab/red_wall_diagnosis.py`, offline, official
  wall scorer against the walls-only truth). R1 lost 10-33 wall points
  against a greedy placement on its own knowledge: every overlapping
  placement (21 of 21) was made by a drone that already knew the earlier
  bomb. On Map04, solid boxes hold about half of the interior wall pixels,
  but the lidar sees only their outlines; this explains a large part of the
  low Map04 wall score of Phase 2 (32.09), the rest being walls not yet
  mapped when placement starts.
* **R2**: targets given up when a known bomb lies within 0.8 blast radii;
  claims relayed with their age; enclosed unknown regions of the drone's own
  grid (not the disposal center) weighted as solid wall in placement only.
  10 tests in `devtools/tests/test_red_r2.py`; five planted regressions each
  fail. A first solid-interior version mistook the disposal center for a box
  (found from a picture of the inferred cells) and was corrected before R2
  was frozen.
* **Real pairs** (Map04, Map01, Map05): wall 54.61 to 78.81 on average
  (higher in 3 of 3, p = 0.25), map 95.10 to 95.48, partial score 68.47 to
  73.50, all bombs placed and maps submitted. N-4 (native, Map04): wall
  64.96, map 96.23. R2 adopted. These maps were used to develop R2.

## 7. Budget, compliance and remaining limits

* **Budget:** 30 authorized, **28 started**, 0 interrupted, 0 from a dirty
  tree: 18 blue pair rounds, 3 native blue, 6 red pair rounds, 1 native red;
  both reserve rounds unused. Offline development attempts P3-A001 to
  P3-A014 are recorded separately; P3-A009 (deferred) and P3-A012 (scratch
  worktree reset) ran nothing usable and are recorded as such.
* **Runtime:** B2 median 71.8 steps/s against 68.2 for B1 over the nine
  pairs (range 59.6-74.1, lowest on Map04); mean controller time per
  drone-step 0.82-1.14 ms for B2 and 0.85-1.12 ms for B1 across the rounds;
  B2's worst-case `control()` time is 2-3 ms higher
  (29-33 ms against 27-31 ms) and was not profiled. Nothing measures the
  organizer's evaluator.
* **Compliance:** agent code only in `src/swarm_rescue/solutions/`; the
  static compliance test covers the new module; no framework, map,
  configuration, scoring or official document changed; no dependency added;
  no truth data in agent decisions. The official checker's results at the
  final agent code are recorded in the review package.
* **Limits:** small samples; Map03 and the red maps are development
  scenarios; witnesses are not shared between drones; the solid-interior
  inference cannot distinguish a sealed room from a box; offline red results
  of identical code differed once between processes (cause not found); the
  instrumented harness and the native path agreed in one blue check only in
  this phase.

## 8. Recommended next priority

Measure transfer to configurations that were never used for development:
new official-style maps or legitimately generated wall layouts with sealed
rooms, narrow approaches and centers walled on three sides, paired against
the frozen B2 and R2, before any further feature work. Within the agents,
the most concrete open items are sharing witnessed stand-offs between drones
and scheduling red placement later on large maps, where the team map is
incomplete when placement starts.

## 9. Submission

No file was uploaded to the competition platform and no preliminary, red or
blue submission was made. Packaging and submission remain owner actions.
