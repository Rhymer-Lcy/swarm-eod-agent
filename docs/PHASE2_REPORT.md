# Phase 2 report: robustness and controlled validation

Dates are business dates in UTC+8. Phase 2 ran on 2026-10-13. Details:
[PHASE2_PLAN.md](PHASE2_PLAN.md) (frozen plan and dated addenda),
[EXPERIMENTS.md](EXPERIMENTS.md) section 7 (ledger, results, analyses),
[AGENT_DESIGN.md](AGENT_DESIGN.md) (current design and open defects).

## 1. Disposition

**PARTIAL.** The carried-heading defect is fixed and validated in the real
simulator; the blue candidate B1 and the red candidate R1 were adopted under
decision rules frozen before the rounds; native competition-mode parity held
in both checks; runtime improved by about 40 % for blue. Not achieved: a
held-out blue map (Map03) exposed a delivery failure, inherited from Phase 1,
that the phase could diagnose but not fix within its budget, and the 70
steps/s objective is met on Map01 and Map02 only. All comparisons rest on few
rounds and claim no statistical significance beyond an exact sign test.

## 2. Starting state and execution environment

* Repository at `ddc6897`, equal to `origin/main`; server clone at `8ee9423`
  (agent code identical), brought to `ddc6897` by bundle before any test.
* Phase 2 budget: 30 complete rounds, separate from Phase 1's 16 (closed).
* Mid-phase owner directive: all Python execution on the remote server
  (remote-first policy, recorded in `local project instructions` and ENVIRONMENT.md). From then
  on, the workstation ran no Python; tests, analyses, profiling and the
  checker ran on the server, uncommitted work in a scratch worktree.

## 3. Carry-aware heading estimation (B1)

* **Root cause.** The rotation model (`0.17591 x command` per step) holds only
  without a bomb; a carrying drone turns by a variable fraction of the command
  (probe: median 0.39, 10-90 % range -0.01 to 0.90). The Phase 1 estimator
  integrated the commanded rotation and switched to the odometer only when the
  two differed by more than 3.5 degrees in one step; small persistent
  differences passed, and the heading controller then stopped turning on a
  wrong estimate. Ordering checked in the source: `control()` at step t, then
  the commands are applied, ten physics sub-steps, sensors computed; the
  odometer read in `control()` at t + 1 measures the rotation caused by the
  command of step t, which is the command the estimator recorded.
* **Fix** (`estimator.py`). Rotation of a step that may involve a bomb (held
  at command time or now) or whose command is unknown is taken from the
  odometer (noise 1 degree), unconditionally; unloaded steps keep the exact
  model and the Phase 1 contact handling. Without an odometer a carrying step
  uses the median ratio with a large variance. A propagated heading variance
  sets the compass gain `P / (P + R)` in [0.004, 0.5]: identical to Phase 1 in
  unloaded flight, about 0.2 while carrying, inside the flat optimum computed
  for an AR(1) compass error and a 1-degree odometer random walk. Commands
  are consumed once, so a brain failure (zero command applied by the shell)
  cannot replay a stale command.
* **Tests** (`devtools/tests/test_estimator_carry.py`, 15 cases, closed-loop
  simulation with the simulator's noise models and the probe's carried
  rotation spread, the frozen Phase 1 estimator run on the same noise):
  carried rotation with noisy odometry (the Phase 1 estimator exceeds 15
  degrees p95 in every seed, the new one stays below 9); unloaded flight
  identical to Phase 1 (bit for bit when no contact occurs); grasp and release
  transitions; odometer dropout; compass dropout and recovery (rejoins the
  run without dropout within 50 steps); all sensors missing; wrapping across
  pi; stale commands; abrupt rotation; carrying gain. Mutation check: with the
  carry path disabled, 10 of the 15 fail.
* **Real-simulator effect** (six pairs): carrying heading error p95
  6.8-9.3 degrees for B1 against 37.0-50.9 for B0.1; free-flight p95 5.4-9.0
  against 13.7-21.5.

## 4. Blue B1: changes and measured effect

Changes besides the estimator: deliveries aim at this step's detections of
the disposal center when it is in view; center points are recorded only with a
trusted heading; the carry field charges 10 instead of 3 for unobserved
cells; gossiped bomb digests are validated and de-duplicated; four compute
reductions found by profiling (section 9). Six pairs against the frozen B0.1
(instrumented, two development maps):

| Metric | B0.1 | B1 |
|---|---|---|
| Bombs disposed (6 x 10) | 53 | 60 |
| Mean official score | 75.45 | 90.56 |
| Pairs with the higher score | 0 | 6 (sign test p = 0.031) |
| Pairs with more disposed | 0 | 3 (3 ties; p = 0.25) |
| Steps per second | 49.3-53.2 | 70.4-73.1 |

The plan's adoption rules were all met; **B1 adopted**. Shared exploration
was evaluated for blue offline and not adopted (it delayed completion).

## 5. Shared exploration

`coverage.py`: a grow-only bitmap of 32 px blocks searched by the team, at
most 234 bytes, broadcast every 10 steps, merged by bitwise OR, so delayed,
duplicated, stale or reordered messages and disconnections cannot remove or
corrupt coverage; invalid payloads are rejected. `base.py`: per-role flags for
using team coverage in target choice, for dropping frontier cells in covered
blocks, and for target commitment (until reached or covered, a 60-400 step
travel budget, or a lower identifier targeting the same area). Deterministic
tests cover payload size on every map size, merge order and duplicates,
stale and delayed messages, invalid messages, disconnection, reconnection
and drone failure, target expiry, yielding and coverage-triggered re-choice
(`devtools/tests/test_coverage_sharing.py`). Cost: under 2.3 % of red brain
time. Role decisions: on for red without dropping frontier cells (dropping
them lowered the offline Map02 map score); off for blue.

## 6. Red R1

R1 = R0.1 + shared exploration (red flags) + border bands clipped to their
observed span + the shared compute changes. Four pairs (instrumented):

| Metric | R0.1 | R1 |
|---|---|---|
| Mean map score | 88.51 | 92.22 |
| Lowest map score | 75.77 | 86.78 |
| Mean partial score (of 80) | 67.14 | 70.12 |
| Submissions; placement completion | 4 of 4; 1.0 | 4 of 4; 1.0 |

R1 mapped more in three pairs (most on the weakest seed) and 0.24 points less
in one; sign test p = 0.625. **R1 adopted** under the plan's rules. The bomb
difficulty term cannot be computed without the organizer's reference agents;
the only cross-role figure (B1 clears R1's layout in 1486 steps) is a
diagnostic.

**Border inference.** Reviewed under the owner's conditions: inputs are the
map dimensions, observed returns and the estimated pose, with no map
coordinates and no unconditional fill. Synthetic tests cover a border gap,
interior walls near and far from the edge and unknown space; the gap test was
mutation-checked after a first version was found unable to fail. Ablation on
the real evidence of the eight red rounds: bands add 6.8-29.6 map points on
Map01 and 1.6-2.1 on Map02 with no penalized pixel; the gap clipping costs
0.01-0.70 points on these gap-free maps. Remaining ambiguity: the organizer
does not state that every official map has a complete border frame.

## 7. Instrumented and native evaluation

The runner gained a native mode (`--native`): official entry point exactly as
the launcher imports it, competition mode, no truth log, no wrapper and no
selector replacement; every ledger line records `run_mode`. Both parity
checks (blue and red, the final candidates, same configuration and seed)
reproduced the instrumented outcome exactly; the wrapper cost about 1 % of
wall time. Held-out and generalization rounds ran natively.

## 8. Budget

30 authorized, **29 started**, 0 interrupted, 0 from a dirty tree: 12 blue
pair rounds, 8 red pair rounds, 2 parity rounds, 5 held-out rounds, 1
cross-role round, 1 reserve round (diagnostic repetition of H-1, reason
recorded beforehand). One reserve round unused. In addition, offline
development runs (not simulation rounds) were recorded per attempt; attempt
A001 failed (a batch script called a `python` that does not exist on the
server's non-interactive `PATH`, and an output filter hid it); it is recorded
as failed and not counted as evidence.

## 9. Generalization and runtime

| Held-out round | Result |
|---|---|
| B1 Map03 (unseen for blue) | 7/10 at the step limit, 58.48: delivery failure, section 10 |
| B1 Map04, Map05 | 10/10 at steps 1413 and 1185 (99.20, 99.40) |
| R1 Map04, Map05 (unseen for red) | map 95.11 and 99.40; partial 63.97 and 74.53 |

Runtime: the profile overturned the Phase 1 belief that geodesic fields
dominate (about 9 %); lookahead visibility tests over the whole occupancy mask
(25 %), sorting in the scan update (19 %), repeated digest merges (8 %) and
the center box percentile (5 %) were the measured costs. Brain time per
drone-step fell from 1.27-1.34 to 0.76-0.80 ms in offline runs on the server
(1.39 to 0.81 in the first profile, made on the workstation before the
remote-first directive) and from 1.37 to 0.87 ms in the real rounds; blue throughput rose from about 51 to 71 steps/s on Map01 (66-68 on
the larger maps). Peak memory unchanged. No claim is made about the
organizer's evaluator.

## 10. Known regressions, uncertainties and open defects

1. **Map03 delivery failure (blue, open).** Stand-off goals on walled sides of
   the disposal center and a final approach without line-of-sight test within
   50 px; three carriers pushed into walls for over 4000 steps. Present in
   B0.1 too (offline). Not a B1 regression; not fixed.
2. **Idle drones near the center** add contacts and obstruct deliveries.
3. **Bomb table duplicates** (100-111 records for 10 bombs offline on Map03).
4. **Small samples.** Six blue and four red pairs on two development maps;
   held-out evidence is one round per map. Instrumented and native outcomes
   were identical in two checks only.
5. **Red wall score** varies widely (32-83) and was low on Map04.
6. **Compute** below 70 steps/s on Map03-Map05; evaluator speed unknown.
7. **The border clipping** trades 0.01-0.70 map points on gap-free maps for
   safety on maps with gaps that may not exist.

## 11. Compliance

Agent code uses only the measured sensor API; the static compliance test
(no ground-truth calls, no framework patching, no prints, no `sys`, `gc`,
`ctypes`, `inspect` imports, no `devtools` imports) passes; the frozen
comparator and all diagnostics live in `devtools/`, which agent code never
imports (tested). The official checker passes in workspace mode on the server
at the final commit with the placeholder identity. No framework, map,
configuration, scoring or official document was modified. No behaviour
depends on the open discrepancies D1-D3. No dependency was added; `req.txt`
stays empty.

## 12. Recommended next priority

Fix delivery to partly walled disposal centers (item 1): keep the
line-of-sight test at every distance, build stand-off goals only from center
points observed from reachable free space, and block a failed approach side
in the carry field; then send idle drones away from the center. Validate on
Map03 and on the development maps with paired rounds. This is the only
held-out failure observed, it is shared by both blue generations, and its
mechanism is localized in the real-simulator log.

## 13. Submission

No file was uploaded to the competition platform and no preliminary,
red or blue submission was made. Packaging and submission remain owner
actions.
