# Phase 3 evaluation plan (frozen before the first Phase 3 round)

Dates are business dates in UTC+8. Written on 2026-10-14 before any Phase 3
simulation round was started; the ledger held 0 Phase 3 rounds when this
file was committed. Changes after that point are dated addenda at the end,
never edits of the sections below.

## 1. Questions

1. **Primary.** Does the blue candidate **B2** (delivery through witnessed,
   reachable stand-offs with a line-of-sight certificate at every distance,
   re-planning on lack of progress, idle drones kept clear of the disposal
   center) eliminate or materially reduce the Map03 delivery failure of the
   frozen **B1**, on paired configurations of the Map03 family?
2. Does B2 avoid regressions against B1 on representative scenarios of the
   other maps, including layouts never used to develop B2?
3. Does B2 behave in the native competition-mode path as in the
   instrumented harness, and how does it do on held-out configurations?
4. What does B2 cost in controller time and simulation throughput against
   B1 in the same rounds?
5. **Secondary, conditional.** If a red candidate **R2** is frozen by an
   addendum before any red round, does it raise the wall-coverage and
   partial red score of the frozen **R1** without lowering the map score?

## 2. Frozen code

| Arm | Code | Selected in the runner by |
|---|---|---|
| B1, R1 (comparators) | `devtools/comparators/p2_540054b/`, byte-identical to `src/swarm_rescue/solutions/eod/` at `540054b` (15 files; verified against Git history by `devtools/tests/test_comparators.py`) | `--agent devtools.comparators.p2_540054b.shell:EodRescueDrone` / `:EodPlaceDrone` |
| B0.1, R0.1 | `devtools/comparators/p1_ddc6897/`, unchanged | not used in Phase 3 rounds (offline opponent diversity only) |
| B2 (blue candidate) | `src/swarm_rescue/solutions/eod/` at the commit that adds this file (agent code identical to `9f0cd91`) | `--agent official` |
| R2 (red candidate) | frozen by an addendum before the first red round, or not at all | `--agent official` |

The comparator package imports no module of the current agent package; its
only shared dependencies are the simulator framework, NumPy, OpenCV and
scikit-image, which are the same for both arms of a pair. Every round runs
from a clean server checkout; the runner records the commit and the dirty
flag. Both arms of a pair run from the same commit.

## 3. Harness modes

As in Phase 2 (PHASE2_PLAN.md, section 3): **instrumented** (truth logger,
`control()` timing) for paired comparisons; **native** (`--native`: official
entry point, competition mode, no truth log, no wrapper) for parity and
held-out confirmation. Seeds set `random` and `numpy.random` before the map
is built; pairs are matched in setup, not in noise, and collision damage
depends on wall-clock speed (D17).

## 4. Scenarios, seeds and partitions

Layouts are local files (`_local/layouts/`) whose SHA-256 the runner
records; all are synthetic (`devtools/simlab/layouts.py`, 10 bombs) except
where noted. Layouts with seed 2 or higher were generated for this plan.

| Id | Map | Layout (SHA-256 prefix) | Seed | Partition for blue |
|---|---|---|---|---|
| M-1 | Map03 | uniform s1 (`a532bd7958ed`) | 53 | development: the Phase 2 H-1 configuration; Map03 s1 was used offline to develop B2 |
| M-2 | Map03 | uniform s2 (`4e25b17bd2b3`) | 61 | development map, layout not used before its round |
| M-3 | Map03 | uniform s3 (`2b53451f4bce`) | 62 | as M-2 |
| M-4 | Map03 | uniform s4 (`cb3098e5e45c`) | 63 | as M-2 |
| M-5 | Map03 | far s5 (`f24e8a22937c`) | 64 | as M-2; bombs weighted away from the center |
| G-1 | Map01 | uniform s1 (`ca1b8e6c1f65`) | 71 | development |
| G-2 | Map02 | uniform s2 (`b6f76c6e205d`) | 72 | held out from B2 development |
| G-3 | Map04 | uniform s2 (`0714e2a4dd4a`) | 73 | held out from B2 development (Map04 itself evaluated before) |
| G-4 | Map05 | uniform s2 (`49183fd62dbb`) | 74 | as G-3 |
| N-2 | Map01 | far s3 (`70c7871301e3`) | 75 | held out from B2 development |
| N-3 | Map01 | placed by R0.1 in Phase 1 (`36f74b178af9`) | 76 | evaluated in Phase 2, not tuned on; a historical red opponent |

**Map03 is a development scenario for blue in this phase**: B2 was designed
after the Phase 2 H-1 failure on Map03 and checked offline on Map03 s1.
Map04 and Map05 were evaluated in earlier phases, so their new layouts are
held out from B2 development but the maps are not unseen. Before its
round, no layout other than Map01 s1, Map02 s1, Map03 s1, Map04 s1 and
Map05 s1 is run offline with B2. Synthetic geometric fixtures
(`devtools/simlab/delivery_fixtures.py`) are development tests in the
offline approximation and are never reported as simulator rounds.

## 5. Allocation of the 30 authorized rounds

| Block | Configurations | Arms | Mode | Rounds |
|---|---|---|---|---|
| M (Map03 failure family) | M-1 .. M-5 | B1 and B2 | instrumented | 10 |
| G (representative and alternative layouts) | G-1 .. G-4 | B1 and B2 | instrumented | 8 |
| N (native confirmation) | N-1 = M-1 configuration; N-2; N-3 | final blue candidate | native | 3 |
| R (red, conditional) | Map04 seed 81, Map01 seed 82, Map05 seed 83 | R1 and R2 | instrumented | 6 |
| N-4 (conditional) | Map04 seed 84 | final red candidate if R2 is adopted | native | 1 |
| Reserve | diagnostic repetitions, only with a reason written beforehand | | | 2 |

Total 28 planned (21 unconditional), 2 reserved, 30 maximum; the runner's
default budget is 30 for `phase3` and refuses a 31st start. Every start
counts, including failed or interrupted rounds. Order: block M (pairs in the
order above, comparator arm first), block G, then N-1 to N-3. The red block
runs only after an R2 addendum; without R2 its six rounds and N-4 are not
used and are not reallocated to blue. Unused rounds are not spent merely
because they exist.

## 6. Metrics

Blue, primary: bombs disposed; official blue score; full completion (all
bombs disposed); delivery failures (carry episodes that ended in a drop
without disposal or were still held at the end, from the truth log through
`devtools/simlab/carry_episodes.py`). Secondary: completion step; transport
time (mean carry episode length); failed approach attempts (B2:
`approach_failures`, B1: `deliver_retries`, agent counters); health component
and drones destroyed; steps per second; controller milliseconds per
drone-step (mean) and worst-case controller latency (maximum `control()`
time); termination.

Red (conditional), primary: map score; partial red score (map 50 %, wall
20 %, time 10 %; the bomb-difficulty term is not computed); placement
completion; wall score. Secondary: blast redundancy, submission step,
steps per second.

## 7. Decision rules

* **B2 is adopted** if all of the following hold:
  1. Block M: B2 disposes more bombs in total than B1, and fewer than B1 in
     no pair.
  2. Block G: B2 disposes fewer than B1 in at most one pair, its total is at
     least B1's minus one, and its mean official score is at least B1's
     minus 3 points.
  3. No B2 round crashes, reaches the wall-clock limit or reports brain
     errors, and B2 loses no more drones than B1 over the nine pairs.
  4. B2's median steps per second over the nine pairs is at least 0.9 times
     B1's.
  Otherwise B1 is retained as the blue candidate (subject to section 8).
* **Parity** (N-1 against the B2 arm of M-1): same termination class and a
  disposed-bomb difference of at most one; differences in score and steps
  per second are reported, not tested.
* **R2 is adopted** (if the red block runs) if, over the three pairs: its
  mean wall score is higher than R1's, its mean map score is at least R1's
  minus 1 point, it submits a map in every round, and its placement
  completion is not lower than R1's in any pair. Otherwise R1 is retained.
* **Statistics.** Each pair is reported with both values. The only test is
  an exact sign test on per-pair differences of a primary metric; with five
  pairs the smallest two-sided p-value is 0.0625, with four 0.125, with
  three 0.25, so no block on its own can reach 0.05 and no significance is
  claimed. Development and held-out results are never pooled into a claim of
  general competitive superiority.

## 8. Stop rules

* A B2 crash, brain error or new failure class (for example carriers still
  holding bombs at the step limit on a map where B1 completes) stops the
  blue blocks until a written diagnosis exists.
* If B2 does not dispose more bombs than B1 in either of M-1 and M-2, the
  Map03 block stops for a written diagnosis before more rounds are spent.
* At most one corrective iteration (B2.1) is allowed, only after a written
  diagnosis in an addendum; rounds already started stay counted and
  reported, and the adoption rules then apply to the pairs run with B2.1.
* A server that is busy (NUMA node 0 in use, GPU ownership unclear) defers
  rounds; work never moves to node 1 or to another machine.
* Reserve rounds are used only for a diagnosis written down beforehand.
* No round beyond 30, no sweeps, no parallel rounds.

## Addenda

### Addendum 1 (2026-10-14, after 21 of 30 rounds; before any red round)

* **Blue decision (section 7 rules, rounds 1-18, all at `bb569c3`).**
  Block M: B2 disposed 50 of 50 bombs against 38 of 50 for B1 and fewer in
  no pair (more in all five; sign test p = 0.0625); every B1 round reached
  the step limit with carriers still holding bombs, every B2 round ended
  with all bombs disposed. Block G: equal disposals in all four pairs (10 of
  10 each), mean official score 97.55 for B2 against 96.12 for B1. No round
  of either arm crashed, reached the wall-clock limit, reported a brain
  error or lost a drone. Median steps per second 71.8 (B2) against 68.2
  (B1). **B2 is adopted.** Per-round values are in EXPERIMENTS.md, section 8.
* **Parity (round 19).** The native N-1 round ended exactly like the
  instrumented B2 arm of M-1 (all bombs at step 791, score 99.44, health
  97.2).
* **Held-out native rounds (20, 21).** N-2 and N-3 ended with all bombs
  disposed (steps 1711 and 1336).
* **Measurement note.** In a round that ends because the last bomb was
  disposed, the truth log stops before that release is recorded, so
  `carry_episodes.py` reports the final carry as "held at end"; such an
  episode is counted as a disposal (each of these rounds records 10 of 10
  disposed). Delivery failures are therefore 12 for B1 (all in block M,
  equal to its undisposed bombs) and 0 for B2.
