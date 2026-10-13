# Phase 1 report: first dual-track development cycle

Dates are business dates in UTC+8. Phase 1 ran on 2026-10-13. Details:
[AGENT_DESIGN.md](AGENT_DESIGN.md) (interfaces, architecture, defects),
[RESEARCH.md](RESEARCH.md) (method selection), [EXPERIMENTS.md](EXPERIMENTS.md)
(methodology, ledger, results).

## 1. Disposition

**PASS for the bounded first cycle.** Every step of the cycle was carried
out: interface reconnaissance with measurements, method selection, a
ledgered evaluation harness, a functional blue candidate, a functional red
candidate, bounded baseline and cross-role tests (16 of 16 authorized
rounds), one narrowly scoped corrective iteration, documentation and tests.
What is **not** established: competitive strength, statistical significance
of any difference (one run per configuration), or behaviour on the official
maps and evaluator. One confirmed defect (heading drift while carrying) is
open by design of the cycle's scope (section 7).

## 2. Starting state

* Repository at `397c620` (Phase 0.1), clean, equal to `origin/main`.
* Organizer repository: `HEAD` and `main` resolve to `53e129bd...`, the commit
  recorded in the archived baseline ZIP comment, so no new organizer release
  had to be incorporated. Nothing was merged.

## 3. What was built

| Item | Where | Status |
|---|---|---|
| Agent package (2,559 lines with the entry point) | `src/swarm_rescue/solutions/eod/`, `my_drone_eval.py` | B0.1 and R0.1 functional in competition mode |
| Shared components | estimator, grid and planner, motion, perception, protocol, navigation core | Unit-tested |
| Blue policy B0.1 | `blue.py` | Exploration, gossiped bomb table, claims, grasp on live detections, delivery |
| Red policy R0.1 | `red.py`, `wallmap.py` | Evidence mapping, snapshot sharing, coverage placement, leader submission, wall-clock safety |
| Harness | `devtools/simlab/runner.py`, `occupancy.py`, `remote_run.sh`, `diagnostics.py`, `summarize.py` | Ledger, budget guard (verified), occupancy gate, truth log |
| Characterization | `probe.py`, `analyze_probe.py`, `fixtures.py`, `egl_device_map.py` | Dynamics, sensors, grasp mechanics measured; EGL device mapping resolved |
| Offline approximation | `offline_sim.py`, `offline_eval.py`, `layouts.py` | Calibrated to the probes, including the carried-rotation effect |
| Tests | `devtools/tests/` | 46 pass (22 Phase 0, 24 new), including a mutation-tested static compliance check |

## 4. Main results (single runs; see EXPERIMENTS.md)

| Configuration | Supplied example | Candidate |
|---|---|---|
| Blue, Map01, synthetic layout S1 | 0/10 disposed, 20.0 | B0 7/10, 61.4; B0.1 8/10, 67.2 |
| Blue, Map01, layout placed by R0.1 | 0/10, 20.0 | B0.1 10/10 at step 1731, 84.2 |
| Blue, Map01, layout placed by the red example | not run | B0.1 10/10 at step 517, 99.8 |
| Blue, Map04 (held out), S1 | not run | B0.1 10/10 at step 1632, 99.6 |
| Blue, Map05 (held out), S1 | not run | B0.1 10/10 at step 1484, 99.4 |
| Red, Map01, seed 21 (partial of 80) | map 0, 16.8 | R0 map 55.4, 48.0; R0.1 map 67.9, 55.0 |
| Red, Map01, seed 28 | not run | R0.1 map 90.6, wall 78.0, 70.9 |
| Red, Map03 (held out) | not run | R0.1 map 99.7, wall 88.6, 77.6 |

## 5. Budget and resources

16 complete rounds were authorized and 16 used (3 probes, 3 baseline rounds,
10 candidate rounds); none was interrupted; all ran from clean commits; a
17th attempt was refused by the harness. One simulation process at a time,
CPUs of NUMA node 0 only (node 1 never used), one GPU identified by UUID
after an all-GPU occupancy check before every round, per-run timeout 600 s
(longest round 44.3 s including start-up; 384 s for all 16 rounds). No
training, sweeps or background queues.

## 6. Compliance

Only the measured sensor API is used; no ground-truth call, framework patching
or print in agent code (static test, mutation-tested). The supplied checker
passes in workspace mode and on a validation-only test package. No behaviour
is designed around D1-D3. Three new discrepancies are recorded in
[COMPETITION_RULES.md](COMPETITION_RULES.md): D15 (the sign of `lateral` is
left, not right), D16 (scope of the ground-truth guard), D17 (wall-clock
collision cooldown).

## 7. Failures, root causes and the next iteration

Fixed in the corrective iteration (B0.1, R0.1): the disposal center treated
as an unknown region, which created a permanent frontier; and planning that
failed from cells inside the inflation margin. Open, in priority order:

1. **B1: carry-aware heading estimation.** While carrying, the drone turns by
   about 0.39 of the command; the estimator integrates the command, and the
   heading error reaches 47-57 degrees at the 95th percentile while carrying.
   Use the odometer rotation while carrying and raise the compass gain then.
   Offline what-if: Map01 10/10 instead of 9/10 and 8/10. Then aim deliveries
   at live disposal-center detections and plan carries through known space
   first.
2. **Shared infrastructure: share searched areas** (gossip compact coverage
   bitmaps or mark teammates' surroundings), and commit to exploration
   targets instead of re-choosing every 40 steps. Benefits both roles; red on
   Map01 most.
3. **R1: map-based localization** (scan matching against the team map) to get
   below the GPS-limited pose accuracy, and an adaptive placement and
   submission schedule for short rounds.
4. **Compute:** keep blue above roughly 70 steps per second on the server so
   that a 7200-step round fits a 240 s limit on a slower evaluator.

## 8. Evidence required before claiming an improvement

* Several rounds per configuration (at least five seeds on Map01 for each
  role) with the score components and failure counters, compared with the
  same scorer against the previous candidate; report spreads, not single
  values, and the worst case.
* Held-out maps that were not used for tuning, and cross-role layouts from
  more than one red policy, including the supplied example.
* For red, a bomb-difficulty estimate requires agreed reference blue agents;
  until then only the partial score (maximum 80) is measurable locally.
* The compute margin measured on hardware closer to the evaluator, or a
  conservative bound.

## 9. Submission

No file was uploaded to the competition platform and no red, blue or
preliminary submission was made. Packaging and submission remain owner
actions.
