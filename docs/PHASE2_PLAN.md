# Phase 2 evaluation plan (frozen before the first Phase 2 round)

Dates are business dates in UTC+8. Written on 2026-10-13 before any Phase 2
simulation round was started; the ledger held 0 Phase 2 rounds when this
file was committed. Changes after that point are dated addenda at the end,
never edits of the sections below.

## 1. Questions

1. Does the carry-aware heading estimator, with the transport changes and
   the compute reductions (candidate **B1**), dispose more bombs than the
   Phase 1 blue candidate **B0.1** on paired configurations, without new
   failure classes?
2. Does the red candidate **R1** map more of the arena than **R0.1** on
   paired configurations, with a submitted map in every round?
3. Do the frozen final candidates behave the same in the native
   competition-mode path as in the instrumented harness?
4. How do the final candidates behave on maps not used for their
   development?
5. Does blue reach about 70 simulation steps per second on the project's
   server (engineering objective, not a guarantee)?

## 2. Frozen code

| Arm | Code | Selected in the runner by |
|---|---|---|
| B0.1, R0.1 (comparators) | `devtools/comparators/p1_ddc6897/`, byte-identical to `src/swarm_rescue/solutions/eod/` at `ddc6897` (verified by `devtools/tests/test_comparators.py`) | `--agent devtools.comparators.p1_ddc6897.shell:EodRescueDrone` / `:EodPlaceDrone` |
| B1 (blue candidate) | `src/swarm_rescue/solutions/eod/` at the commit that adds this file (agent code identical to `e701426`) | `--agent official` |
| R1 (red candidate) | frozen by an addendum before the first red round | `--agent official` |

Every round runs from a clean server checkout; the runner records the commit
and the dirty flag. The harness version is the one at the commit of each
round; both arms of a pair run from the same commit.

## 3. Harness modes

* **instrumented**: the evaluation-only truth logger wraps the agent class
  (`devtools/simlab/diagnostics.py`); it also times every `control()` call.
  Used for the paired comparisons, because heading error, carry episodes and
  controller time need the log.
* **native**: `--native`: the official entry point exactly as the launcher
  imports it, competition mode, no wrapper, no selector replacement. Used for
  the parity checks and the held-out rounds.
* **harness**: no truth log but a harness-selected class; not planned.

Seeds set `random` and `numpy.random` before the map is built. They align
initial headings and layouts, not noise streams across different agents, and
collision damage depends on wall-clock speed (D17). Pairs are therefore
matched in setup, not in noise, and instrumented and native scores are not
assumed equal.

## 4. Scenarios and seeds

Layouts are local files whose SHA-256 the runner records.

| Layout file | SHA-256 (prefix) | Kind |
|---|---|---|
| `synthetic_Map01_uniform_n10_s1.json` (S1) | `ca1b8e6c1f65` | synthetic |
| `synthetic_Map01_far_n10_s2.json` | `07bcf5649e69` | synthetic, bombs far from the center |
| `r0.1_Map01_seed21.json` | `36f74b178af9` | placed by R0.1 in Phase 1 round 9 |
| `synthetic_Map02_uniform_n10_s1.json` | `13c3c3877b75` | synthetic |
| `synthetic_Map03_uniform_n10_s1.json` | `a532bd7958ed` | synthetic, generated for this plan |
| `synthetic_Map04_uniform_n10_s1.json` | `c968118acf5a` | synthetic |
| `synthetic_Map05_uniform_n10_s1.json` | `56fc0761c5c6` | synthetic |

Partition status (what the label "held out" means here):

| Map | Blue | Red |
|---|---|---|
| Map01 | development (real and offline) | development (real and offline) |
| Map02 | offline development in Phase 1, never run in the real simulator | same |
| Map03 | never run for blue (real or offline): **held out** | one R0.1 evaluation round in Phase 1: evaluated, not tuned |
| Map04, Map05 | one B0.1 evaluation round each and offline runs in Phase 1, no tuning: evaluated, not strictly unseen | never run for red: **held out** |

No offline run on Map03 (blue) or Map04/Map05 (red) is made before their
rounds below.

## 5. Allocation of the 30 authorized rounds

| # | Block | Configuration (drones as in the supplied plans: blue 10, red 5 with 2 bombs each) | Arms | Mode | Rounds |
|---|---|---|---|---|---|
| B-1..B-6 | Blue pairs | Map01 S1 seed 31; Map01 S1 seed 32; Map01 S1 seed 33; Map01 far seed 34; Map01 R0.1 layout seed 35; Map02 S1 seed 36 | B0.1 and B1 | instrumented | 12 |
| P-B | Blue parity | Map01 S1 seed 31 (same as B-1) | B1 | native | 1 |
| R-1..R-4 | Red pairs | Map01 seeds 41, 42, 43; Map02 seed 44 | R0.1 and R1 | instrumented | 8 |
| P-R | Red parity | Map01 seed 41 (same as R-1) | R1 | native | 1 |
| H-1..H-5 | Held out / generalization | blue Map03 S1 seed 53, Map04 S1 seed 55, Map05 S1 seed 54; red Map04 seed 51, Map05 seed 52 | final B1 / R1 | native | 5 |
| X-1 | Cross-role (adversarial diagnostic, not a difficulty score) | B1 on the layout R1 places in R-1 | B1 | instrumented | 1 |
| reserve | Diagnostic repetitions, only with a written reason | | | | 2 |

Total 28 planned, 2 reserved, 30 maximum; the runner refuses a 31st start.
Every start counts, including failed or interrupted rounds. The blue blocks
run first; red rounds start only after the R1 addendum.

## 6. Metrics

Blue, primary: bombs disposed; official blue score. Secondary: completion
step, health component, transport failures (releases without reward),
heading error of the agent's estimate while carrying (p50, p95; aligned one
physics step as in Phase 1), controller time per drone-step, steps per
second, termination.

Red, primary: map score (official scorer); partial red score (map 50 %,
wall 20 %, time 10 %; the bomb-difficulty term needs the organizer's
reference agents and is not computed). Secondary: wall score, placement
completion, submission step and reason, steps per second.

Runtime: `steps_per_second` from the official elapsed step count and wall
time (simulator plus agents plus, in instrumented mode, the logger);
controller milliseconds per drone-step from the instrumented timing. The
70 steps/s objective is assessed on blue Map01 rounds with 10 drones on the
project server (one simulation process, node 0 CPUs, two threads per
library, one GPU through EGL).

## 7. Decision rules

* **B1 is adopted** if, over the six blue pairs: the total disposed is at
  least B0.1's, B1 disposes fewer than B0.1 in at most one pair, no B1 round
  crashes or reaches the wall-clock limit, the agents report no brain
  errors, and the pooled carrying heading error p95 of B1 is at most 15
  degrees (Phase 1: 47-64). Otherwise the regression is diagnosed and the
  better validated version is retained.
* **R1 is adopted** if, over the four red pairs: the mean map score is at
  least R0.1's, R1 submits a map in every round, and placement completion is
  not lower than R0.1's in any pair. Otherwise R0.1 is retained as the red
  candidate.
* **Parity** is reported as the differences in termination, score and steps
  per second between the native round and its instrumented counterpart. A
  different termination class, or a score difference larger than the spread
  of the paired rounds, is investigated before any held-out round.
* **Statistics.** Each pair is reported with both values. The only test is
  an exact sign test on the per-pair differences of the primary metric; with
  six pairs the smallest two-sided p-value is 0.031 (all six in one
  direction), with four pairs 0.125, so most outcomes cannot be called
  significant and are not. No pooling across maps into a single score.

## 8. Stop rules

* A fundamental correctness failure of B1 (crash, brain errors, heading
  error not reduced) stops the blue blocks until it is understood; no
  further round is spent on it without a written diagnosis.
* A blocked server (node 0 busy, GPU ownership unclear) defers rounds; it
  never moves them to node 1 or to another machine.
* Reserve rounds are used only for a diagnosis written down beforehand.
* No round beyond 30, no sweeps, no parallel rounds.

## Addenda
