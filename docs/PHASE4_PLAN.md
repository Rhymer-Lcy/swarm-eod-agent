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

### Addendum 1 (2026-10-14, before any agent ran on a template scenario; 0 rounds)

* **Scenario registry.** Generated at `9c1a7bb` (`p4_scenarios.py --list`);
  SHA-256 of the canonical specification JSON, first 12 hex digits:

  | Family | C-probe (V1) | C-final (V2) | C-reserve (V3) |
  |---|---|---|---|
  | pocket | `5d42bd9c60fe` | `9490c57bd15b` | `a7dbe516a367` |
  | corridor | `5febb44d59d5` | `199238591c58` | `6af90d52fdb6` |
  | gpsdark | `6da4c15799e9` | `8f78e758081d` | `dfc87c3b1748` |
  | comdark | `b7c465c55cec` | `83c9e22f5d88` | `b54b5080a4f9` |
  | clutter | `3e95e71d95ea` | `63fe2b26347c` | `37f348e1b87b` |
  | rooms | `e5d0ee04c9df` | `1dbdc3161c5a` | `d3141d3a5334` |
  | mixed | `6c33344971bb` | `bdbc9cf1c145` | `58dc38e62d68` |

* **Fixtures** (P4-A003): the framework's rasterization of the 14 C-probe and
  C-final scenarios agrees with the specifications: every wall pixel of
  either raster lies within 2 px of the other (the framework draws walls
  about 1 px thicker than the nominal 6 px) and every box is solid
  (`test_p4_scenarios_gl.py`, 15 passed; a planted mis-oriented box fails
  by 141 px). The first version of that test used an overlap ratio above
  0.95, which thin walls drawn 1 px thicker cannot meet (0.89-0.97); it was
  replaced before any result depended on it.
* **Layouts** (P4-A004, `layouts.py`, 10 bombs), SHA-256 first 12 hex digits:
  Map02 wall s101 `36b999da78e0`; Map04 cluster s102 `dd8aa93ae47f`; Map05
  far s103 `4599fee97dea`; P4PocketV1 uniform s111 `f09a0bfce0b4`;
  P4CorridorV1 uniform s112 `395f6b52f75e`; P4GpsdarkV1 uniform s113
  `bdebba9f3e51`; P4ComdarkV1 uniform s114 `cc41b9d69dc4`; P4ClutterV1 wall
  s115 `cb7dacb270f8`; stage 2 only: Map03 uniform s121 `e3f3dde02520`;
  P4PocketV2 uniform s211 `932fabf8ecf7`; P4CorridorV2 uniform s212
  `f64f4dcb574c`; P4ClutterV2 uniform s213 `336e44d420f9`. All twelve were
  generated in one batch; none was regenerated.

### Addendum 2 (2026-10-14, after stage 1: 14 of 24 rounds; before any stage-2 round)

* **Stage 1** (rounds 1-14, all at `3021c95`, B2/R2 as `official`; per-round
  values in EXPERIMENTS.md section 9). Blue: 10 of 10 bombs in 7 of 8
  instrumented rounds (Map02, Map04, Map05; P4PocketV1, P4GpsdarkV1,
  P4ComdarkV1, P4ClutterV1); **P4CorridorV1: 0 of 10** at the step limit.
  Red: maps submitted and 10 bombs placed in all rounds; wall 95.66 (Map02),
  51.91 (P4RoomsV1), 70.73 (P4MixedV1). V-N1 (native) equals V-C1 and V-N2
  (native) and V-D1 (repeat) equal V-R2 in every recorded value. No crash,
  brain error, wall-clock limit or lost drone.
* **Confirmed defect (blue).** In V-C2 every drone grasped one bomb, all ten
  ended in the carry state and none had recorded a single surface point of
  the center (agent reports). Mechanism (`blue.py`, `_carry`): a carrier that
  does not know the center drives straight at its start area (no path) and
  waits; blue does not explore while carrying, so when all drones carry
  before anyone has seen a hidden center, nobody searches for it. Reproduced
  offline (P4-A014, seed 1: 0 of 10, no drone ever knew the center).
  Environment ruled out: the corridor (82 px) is wider than the planner's
  closure threshold for ideal walls (56-62 px, P4-A016) and other runs
  passed it. Relevance: on the five supplied maps the center is in view of
  the start area, so the defect cannot occur there; on a map where it is not
  and bombs are at least as many as drones it can lose every bomb.
* **B3** (agent code at `ce09cb7`; only `blue.py` differs from B2): a carrier
  without a known center searches for it with the unloaded drones' target
  choice along planned paths, and with nothing left to search follows a
  planned path home (`_seek_center`). Behaviour with a known center is
  unchanged (command-identical closed-loop test). Tests:
  `test_b3_seek_center.py` (frozen B2 fails the hidden-center case, B3
  delivers; a planted return of B2's behaviour fails the test).
* **Development evidence for B3 (offline, C-probe and development maps,
  P4-A018 against P4-A014, same seeds; not stage-2 evidence).** P4CorridorV1
  seeds 1 and 2: B3 6 and 4 of 10, B2 0 and 10 of 10 (total 10 of 20 each);
  P4PocketV1: both 10 of 10, B3 at steps 1387/1432 against 2111/2357;
  P4ClutterV1: both 10 of 10, 2409/1337 against 2006/1748; P4GpsdarkV1,
  P4ComdarkV1, Map03 s1, Map01 s1: identical outcomes. The development
  evidence is mixed: B3 removes the total deadlock but lost corridor seed 2,
  where its carriers, after finding the center, had no route to it (351
  fallback fields). A second limitation explains such runs and is **not**
  addressed: the planner's 24 px obstacle inflation on 8 px cells closes
  passages below 56-62 px between ideal walls and, in maps drawn under pose
  error, wider ones (a 70 px corridor in the `hidden` fixture, P4-A016).
  B3 is evaluated as frozen; no further iteration is made in Phase 4.
* **Risk A** (radial fallback): unchecked goals can lie on a blocked side
  and, with a complete map and no witness, deadlock a carrier offline
  (`test_p4_risk_a.py`); the terminal push never moved through a wall. In
  rounds the fallback was transient (G-4 of Phase 3; stage 1). No separate
  successor: B3's change does not touch it.
* **Risk B** (R2 solid inference, P4-A013): precision 0.47-0.48 on P4RoomsV1
  with 94-99 % of the false area in sealed rooms, 0.98 on Map04; the
  whole-run effect on the wall score was +21.2 on Map04 and -0.5, +5.2, 0.0
  and -10.0 on the synthetic rooms and mixed maps (the last with one changed
  decision, which scored higher against the truth). No consistent harm is
  shown; **no red successor**; R2 stays frozen.
* **Stage 2** (8 starts, instrumented, frozen arm first in each pair; B2 as
  `devtools.comparators.p3_067818a.shell:EodRescueDrone`, B3 as `official`,
  all at the commit that adds this addendum):

  | Id | Arms | Scenario | Layout | Seed |
  |---|---|---|---|---|
  | S2-1 | B2, B3 | P4CorridorV2 (C-final) | uniform s212 | 611 |
  | S2-2 | B2, B3 | P4PocketV2 (C-final) | uniform s211 | 612 |
  | S2-3 | B2, B3 | Map03 (regression, partition B) | uniform s121 | 431 |
  | S2-4 | R2 | P4RoomsV2 (C-final) | | 521 |
  | S2-5 | R2 | P4MixedV2 (C-final) | | 522 |

  B3 is adopted only under section 8 (no fewer bombs than B2 in any pair,
  more in total, regression pair at least B2's bombs and score minus 3, no
  crash or brain error, median steps per second at least 0.9 of B2's);
  otherwise B2 is retained and `blue.py` returns to B2's bytes. P4ClutterV2
  and the C-reserve scenarios are not run. Two starts stay unused.
