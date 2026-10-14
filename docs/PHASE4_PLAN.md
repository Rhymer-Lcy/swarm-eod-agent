# Phase 4 plan: independent generalization validation (frozen before evaluation)

Dates are business dates in UTC+8. Written on 2026-10-14, before any Phase 4
simulation round and before any agent was run on a template scenario
(`P4<Family>V<seed>`); the ledger held 0 Phase 4 rounds when this file was
committed. Before the commit, the frozen B2 had been run offline only on the
two Risk A development fixtures `pocket` and `narrow` (attempts P4-A001 and
P4-A002, section 5). Changes after the commit are dated addenda at the end,
never edits of the sections below.

## 1. Questions

1. Where do the frozen **B2** (blue) and **R2** (red) succeed or fail outside
   the scenarios used to develop them?
2. **Risk A.** Can B2's radial fallback (`delivery.radial_sources`, which
   returns unchecked goal cells when every map-checked candidate is rejected)
   reintroduce blocked-side approach targets, and can it lead to a failure in
   a round? Route goals and terminal movement are assessed separately.
3. **Risk B.** How often does R2's enclosed-unknown-as-solid inference take
   regions that are not walls (sealed rooms, closed narrow entrances, noise,
   the disposal center) as solid, and how does that change placement?
4. **Risk C.** Why did identical R1 code give different offline wall scores
   in two processes in Phase 3 (Map05 seed 1: 56.48 in P3-A008, 54.74 in
   P3-A010), and are the offline and real-simulator runs reproducible?
5. Only if a material defect is demonstrated: does a narrowly scoped
   successor improve on the frozen agent on scenarios not used to build it?

## 2. Frozen agents

| Agent | Code | Selected in the runner by |
|---|---|---|
| B2, R2 (frozen) | `devtools/comparators/p3_067818a/`, byte-identical to `src/swarm_rescue/solutions/eod/` at `067818a` (16 files; SHA-256 in `p3_067818a.json`, checked against Git history by `devtools/tests/test_comparators.py`); the blue files equal those of the Phase 3 blue rounds (`bb569c3`), the package equals the R2 freeze (`5e0803d`) | `--agent official` while `solutions/` is unchanged (asserted by `test_submitted_agent_is_still_the_frozen_b2_r2`), else `devtools.comparators.p3_067818a.shell:EodRescueDrone` / `:EodPlaceDrone` |
| Successors (conditional) | a new commit with a new identifier, **B3** (blue) or **R3** (red); never called B2 or R2 | `--agent official` once committed, with the frozen arm selected from the comparator package |
| B1/R1, B0.1/R0.1 | `p2_540054b`, `p1_ddc6897`, unchanged | not used in Phase 4 rounds |

The comparator package imports no module of the current agent package; its
only shared dependencies are the simulator framework, NumPy, OpenCV and
scikit-image. None of these is changed in Phase 4.

## 3. Harness modes and custom geometry

* **Instrumented** and **native** rounds as in Phases 2 and 3 (truth logger
  and `control()` timing, or the official entry point in competition mode
  without wrapper). Seeds set `random` and `numpy.random` before the map is
  built.
* **Custom geometry** (`devtools/simlab/p4_scenarios.py`). A scenario is a
  specification generated from a template family and a preregistered
  generation seed. Its map class is built only from public framework
  elements (`NormalWall`, `NormalBox`, `DisposalCenter`, `NoGpsZone`,
  `NoComZone`, `ClosedPlayground`) exactly like the supplied map classes; the
  runner makes it resolvable by the launcher for one round (as it already
  substitutes the agent selector). Physics, sensors, communication, scoring
  and the competition-mode guard are the official ones. No file under
  `src/swarm_rescue/` changes. Such rounds carry `map_kind: custom` and the
  specification's SHA-256 in the ledger; they are **synthetic validation
  scenarios, not official maps and not competition results**, also when run
  in native mode.
* **Offline approximation** (`offline_sim.py`) for CPU-only checks; never
  reported as simulator rounds. Disabler zones are modelled by the drone's
  true centre lying inside the zone rectangle (GPS and compass off, or radio
  off).

## 4. Partitions

| Partition | Content | Use |
|---|---|---|
| A, development | Map01-Map05 with every layout and seed used in Phases 1-3; the fixtures `open`, `corner`; Map03 for blue and Map01/Map04/Map05 for red | not evaluated again |
| B, new layouts on official maps | layouts from new generator seeds (101-199) and the new profiles `wall` and `cluster`; red seeds not used before | frozen validation; **the maps are not unseen** |
| C-probe | custom scenarios of generation seed 1 (`P4<Family>V1`) and the Risk A fixtures `pocket`, `narrow` | offline checks and stage-1 rounds of the frozen agents; becomes development data for any successor |
| C-final | generation seed 2 (`P4<Family>V2`) | untouched by every agent (offline and simulator) until stage 2 |
| C-reserve | generation seed 3 | generated, never run in Phase 4 without a prior addendum |
| D, truly independent | organizer-held maps | **not available**: no unseen official map exists locally; nothing is reported as D |

Template families (geometry in the module docstrings; generation applies a
seeded mirror and jitter to positions, sizes and gaps):

| Family | Role | Exercises |
|---|---|---|
| `pocket` | blue | center walled on three sides (2-3 px gaps), open face away from the start area, center not visible at the start, long divider with a 66-95 px and a 104-154 px passage |
| `corridor` | blue | center walled on three sides, the fourth face reachable only through a 70-90 px corridor 200-280 px long; start in the opposite corner; long detour |
| `gpsdark` | blue | supplied-map-like layout, GPS and compass disabled around the center and in one room |
| `comdark` | blue | radio disabled over the eastern 57 % of the arena |
| `clutter` | blue | start in one corner, center in the opposite one with a box and a wall 2-3 px off two faces, field of six boxes with 70-110 px passages |
| `rooms` | red | two sealed empty rooms, a room with a 36-44 px door, a room with a 100 px door, two solid boxes of similar size |
| `mixed` | red | box sharing a wall with a sealed room, U-shaped outline, L wall, box inside a room, box against the frame, center walled on three sides |

Blue layouts on custom scenarios are sampled from the scenario's framework
fixture with `devtools/simlab/layouts.py` (10 bombs, admissible region
connected to the start area). Every layout file's SHA-256 is recorded by the
runner. No layout or seed is replaced after it has been generated; an
unusable one is recorded and the next preregistered seed is used.

## 5. Offline checks before any round (no simulator budget)

Attempts are numbered P4-A001 onwards in EXPERIMENTS.md section 9, each with
its command, commit and status, failures included.

1. Test suite, including the new Risk A, Risk B, scenario and comparator
   tests.
2. Fixture extraction of all C-probe and C-final scenarios (agent-free; uses
   OpenGL for the framework's rasterization, no simulation step) and layout
   generation.
3. Risk A: focused tests (`test_p4_risk_a.py`) and closed-loop offline
   carrier cases on `pocket` and `narrow`.
4. Risk B: `red_solid_audit.py` on P4RoomsV1, P4MixedV1 and Map01-Map05
   (seed 1), with the inference-disabled diagnostic variant for the
   whole-run comparison.
5. Risk C: `repro_check.py repeat` on one blue and one red configuration,
   and `repro_check.py observer` on R1 Map05 seed 1 (the P3-A008/P3-A010
   configuration).
6. B2 offline on the C-probe blue scenarios (seeds 1-2 of the offline
   noise), R2 offline through the audit above.

## 6. Full simulator rounds (at most 24)

The runner's default budget for `phase4` is 24 and refuses a 25th start.
Every start counts, including failed, deferred-after-start or interrupted
ones. Order: stage 1 in the table order; stage 2 only after an addendum.

**Stage 1, frozen validation (14 starts).**

| Id | Agent, mode | Scenario | Layout (profile, generator seed) | Seed | Partition |
|---|---|---|---|---|---|
| V-B1 | B2, instrumented | Map02 | wall, 101 | 401 | B |
| V-B2 | B2, instrumented | Map04 | cluster, 102 | 402 | B |
| V-B3 | B2, instrumented | Map05 | far, 103 | 403 | B |
| V-C1 | B2, instrumented | P4PocketV1 | uniform, 111 | 411 | C-probe |
| V-C2 | B2, instrumented | P4CorridorV1 | uniform, 112 | 412 | C-probe |
| V-C3 | B2, instrumented | P4GpsdarkV1 | uniform, 113 | 413 | C-probe |
| V-C4 | B2, instrumented | P4ComdarkV1 | uniform, 114 | 414 | C-probe |
| V-C5 | B2, instrumented | P4ClutterV1 | wall, 115 | 415 | C-probe |
| V-R1 | R2, instrumented | Map02 | (10 bombs placed by 5 drones) | 421 | B |
| V-R2 | R2, instrumented | P4RoomsV1 | | 422 | C-probe |
| V-R3 | R2, instrumented | P4MixedV1 | | 423 | C-probe |
| V-N1 | B2, native | P4PocketV1 | as V-C1 | 411 | parity with V-C1 |
| V-N2 | R2, native | P4RoomsV1 | | 422 | parity with V-R2 |
| V-D1 | R2, instrumented | P4RoomsV1 | | 422 | repeat of V-R2 (Risk C) |

**Stage 2 (at most 10 starts), only after an addendum written beforehand.**

* If a successor is justified (section 8): paired rounds, frozen arm first,
  on C-final scenarios of the families where the defect was found, plus one
  regression pair on an official map with a new layout (Map03, uniform,
  generator seed 121, seed 431). At most 3 blue pairs and 2 red pairs; the
  addendum names them before the first stage-2 start.
* If no successor is justified: the frozen agents on C-final, one round each
  of P4PocketV2, P4CorridorV2, P4ClutterV2 (B2; layouts uniform, generator
  seeds 211-213; seeds 511-513) and P4RoomsV2, P4MixedV2 (R2; seeds 521-522),
  5 starts. They are run only if stage 1 leaves the transfer question open
  for those families; the addendum says which.

## 7. Metrics

* **Blue, primary:** bombs disposed (official); official blue score; full
  completion; delivery failures reconciled with the official count
  (`carry_episodes.py`, `reconciled`: carries cut off by the round ending on
  its last disposal are disposals; only carries still unresolved, and drops
  not followed by a disposal of that bomb, are failures). Secondary: health
  and collisions, drones destroyed, recovery counters (`approach_failures`,
  `relaxed_sources`, `radial_fallbacks`, `terminal_blocked`, `stuck`), steps
  to completion, controller latency (mean, maximum), steps per second,
  termination.
* **Red, primary:** map score, wall score, placement completion, partial
  score (map 50 %, wall 20 %, time 10 %; the bomb-difficulty term needs
  organizer reference agents and is not computed or proxied). Secondary:
  solid-inference area and precision (offline audit; `solid_cells` counter in
  rounds), blast redundancy, submission step, sensitivity by scenario type.
* **Risk C:** identity of outcomes across repeats (termination, step, score,
  disposed or placed bombs, and positions where logged); wall-clock-dependent
  quantities (health, steps per second) are reported, not tested.

## 8. Decision rules

* **Defect.** A behaviour is a confirmed defect only with (1) a concrete
  reproduction, (2) the component identified, (3) environment and algorithm
  causes separated, (4) its relevance to the official competition assessed.
  A failure in a scenario that no algorithm could solve (for example, a
  center reachable only through an opening narrower than a carrying drone)
  is not a defect.
* **Successor.** Only a confirmed, material defect justifies a successor,
  scoped to that defect, with focused regression tests (including a test
  that fails on the frozen agent), at most one per role.
* **B3 is adopted** only if, over its stage-2 pairs: B3 disposes at least as
  many bombs as B2 in every pair and more in total; in the regression pair
  it disposes at least as many and scores at least B2 minus 3 points; no B3
  round crashes, reaches the wall-clock limit or reports brain errors; and
  its median steps per second is at least 0.9 times B2's. Otherwise B2 is
  retained.
* **R3 is adopted** only if, over its pairs: mean wall score higher than
  R2's, map score at least R2's minus 1 point in every pair, a map submitted
  in every round, placement completion not lower in any pair. Otherwise R2
  is retained.
* **Statistics.** Every pair and every single round is reported. With at
  most three pairs per block no sign test can reach p < 0.05; results are
  exploratory. Development (A, C-probe after use) and validation results are
  never pooled into a claim.
* **Generalization claims** name the partition and the families; C results
  are claims about synthetic scenarios built from the official framework,
  not about the organizer's evaluation maps.

## 9. Stop rules

* A crash, brain error or harness error stops the affected role's rounds
  until a written diagnosis exists.
* The server is used only as `docs/ENVIRONMENT.md` allows: NUMA node 0 after
  inspection, never node 1; one simulation process at a time; every GPU's
  compute processes checked; timeouts on every job. A busy or unclear server
  defers the work; a deferred check is never counted as an experiment.
* Every start is preceded by: source commit and clean tree verified on the
  server, scenario and seed checked against this plan, remaining budget
  checked, occupancy gate, timeout.
* No round beyond 24, no sweeps, no parallel rounds, no background jobs.

## Addenda
