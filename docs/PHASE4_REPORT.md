# Phase 4 report: independent generalization validation

Dates are business dates in UTC+8. Phase 4 ran on 2026-10-14. Details:
[PHASE4_PLAN.md](PHASE4_PLAN.md) (protocol frozen before evaluation, three
dated addenda), [EXPERIMENTS.md](EXPERIMENTS.md) section 9 (ledger,
offline attempts, analyses), [AGENT_DESIGN.md](AGENT_DESIGN.md) section 7
(open defects).

## 1. Disposition

**PASS for the bounded validation cycle; B2 and R2 retained unchanged.**
Outside their development scenarios the frozen agents completed every task
except one class: on synthetic maps where the disposal center cannot be seen
from the start area, B2 can lose every bomb (P4CorridorV1 and P4CorridorV2:
0 of 10). The mechanism was confirmed in the real simulator. A narrowly
scoped candidate, B3, recovered 6 of 10 there but disposed one bomb fewer
than B2 in another pair, so under the preregistered rule it was not adopted.
The cause of the unexplained Phase 3 reproducibility difference was found
(OpenCV threading in evaluation tooling) and fixed. Broad generalization is
not claimed: the evidence is single rounds per scenario on synthetic
geometry built from the official framework, not the organizer's maps.

## 2. Git states

* Start: workstation `main` = `origin/main` = `067818a`, clean; server clone
  at `067818a`, clean. Private working notes were read from this project's own
  ignored directory; no other project's notes were written.
* Agent code: at the end of Phase 4 `src/swarm_rescue/solutions/` is
  byte-identical to `067818a` (B2 and R2). The final commit and the pushed
  `origin/main` are recorded in the review package manifest.

## 3. Execution environment

Environment details were verified using the documented project setup.

## 4. Scenarios and partitions

| Partition | Content |
|---|---|
| A, development | every map, layout and seed of Phases 1-3 (not re-evaluated) |
| B, new layouts on official maps | Map02 `wall` layout, Map04 `cluster`, Map05 `far`, Map03 `uniform` (new generator seeds); **the maps are not unseen** |
| C-probe | seven synthetic families, generation seed 1: `pocket`, `corridor` (center walled on three sides, hidden from the start, long detours, narrow approach), `gpsdark` (GPS and compass off around the center), `comdark` (radio off over 57 % of the arena), `clutter` (box field, two-sided center), `rooms` and `mixed` (sealed rooms, narrow doors, boxes, incomplete outlines) |
| C-final | the same families, generation seed 2; untouched by any agent until stage 2 |
| C-reserve | generation seed 3; generated, never run |
| D, truly independent | not available: no unseen official map exists locally |

Synthetic scenarios are built only from public framework elements and run
through the official launcher, physics, sensors, communication and scorers;
their specifications and layouts are hash-registered (addendum 1).

## 5. B2 findings

* Partition B: 10/10 in all three rounds (scores 98.48-99.48).
* C-probe: 10/10 on pocket, gpsdark, comdark and clutter (99.20-99.76);
  **0/10 on corridor**. C-final: pocket 10/10, **corridor 0/10**. Map03 new
  layout 10/10.
* Communication blackout over most of the arena and GPS/compass loss around
  the center did not cost a bomb in these rounds.
* Confirmed defect: a carrier that does not know the center drives straight
  to its start area and waits; blue does not explore while carrying, so a
  hidden center is never found once every drone carries a bomb.
* Risk A (radial fallback): unchecked goals can lie on a blocked side; with
  a complete map and no witness a carrier can wait behind a wall
  indefinitely (offline); the terminal push never moved through a wall; in
  rounds the fallback was transient.
* Not addressed, shared with every version: the planner closes passages
  narrower than about 56-62 px between ideal walls (drones are 30 px wide),
  and wider ones in maps drawn under pose error.

## 6. R2 findings

* All rounds submitted at step 2350 with 10 bombs placed. Map scores
  91.50-98.22; wall scores 95.66 (Map02), 51.91 and 67.54 (rooms V1, V2),
  70.73 and 71.03 (mixed V1, V2).
* Risk B: the solid-interior inference takes sealed rooms as solid
  (precision 0.47-0.48 on the rooms map, 94-99 % of the false area inside
  them) and is accurate on boxes (0.98 on Map04, where it adds 21.2 wall
  points offline). Its net effect on synthetic maps was mixed (-0.5, +5.2,
  0.0, -10.0, the last from a single changed decision that was better
  against the truth). No consistent harm; no red successor.

## 7. Reproducibility findings

* Phase 3's R1 difference (56.48 against 54.74) is explained: the offline
  simulator used `cv2.distanceTransform` (precise mask), which gives one
  result with two or more OpenCV threads and a different, not even
  repeatable one with a single thread; P3-A010 had pinned each process to
  one CPU, and OpenCV sizes its pool from the CPU affinity. Reproduced 8 of
  8 times; fixed in the tooling; verified.
* The wrappers' OpenCV thread cap never worked (OpenCV reads
  `OPENCV_FOR_THREADS_NUM`, not `OPENCV_NUM_THREADS`); fixed.
* Offline runs are identical across Python hash seeds and thread settings;
  the agents' own OpenCV operations are thread-invariant.
* Real simulator: identical configurations reproduced every recorded value
  (native against instrumented blue, native and repeat against instrumented
  red); only wall-clock quantities differ.
* The official red scorers gave bit-identical results under 1 and 8 OpenCV
  threads on the seven Phase 3 red rounds.

## 8. Confirmed defects

1. Hidden disposal center never searched for (blue; real rounds V-C2 and
   S2-1).
2. Radial fallback without an exit (blue; offline, constructed state).
3. Planner closes narrow passages (blue and red; offline).
4. Sealed rooms taken as solid (red; offline, measured, mixed effect).
5. Tooling: offline results depended on OpenCV threading; the wrappers'
   OpenCV cap was ineffective; the red diagnosis read live brains; the
   carry-episode logger double-counted a bomb held by two drones. All fixed.

## 9. Corrective changes

* Agents: none adopted. B3 (`ce09cb7`) was built, evaluated and rejected;
  `blue.py` was restored to B2 (`4fe480b`), and B3 is kept as
  `devtools/comparators/p4_ce09cb7`.
* Tooling (`devtools/` only): custom-geometry scenarios and runner support;
  disabler zones in the offline simulator; layout profiles `wall` and
  `cluster`; reconciled carry accounting (termination and shared
  disposals); `stable_cv` distance transform; wrapper OpenCV cap and a NUMA
  node-0 check before pinning; Risk B audit and reproducibility tools; the
  frozen comparator packages `p3_067818a` (B2/R2) and `p4_ce09cb7` (B3).

## 10. Tests

247 tests pass on the remote server at the final agent code (new: Risk A,
Risk B, scenario specifications, framework rasters, B3, carry
reconciliation, `stable_cv`, comparator freeze). Planted regressions fail
their tests: a mis-oriented box (raster test), B2's behaviour planted into
B3 (B3 test). Exact command outputs are in the review package.

## 11. Simulator budget

24 authorized, **22 started**: 14 in stage 1 (8 blue instrumented, 3 red
instrumented, 2 native, 1 red repeat) and 8 in stage 2 (3 blue pairs, 2 red
rounds); 0 failed, interrupted or from a dirty tree; 2 unused. Offline work
did not use the budget (EXPERIMENTS.md 9.2).

## 12. Native competition-mode results

V-N1 (B2, P4PocketV1) equals its instrumented twin V-C1 in every recorded
value (10/10 at step 1903, 99.52, health 97.6). V-N2 (R2, P4RoomsV1) equals
V-R2 (map 93.85, wall 51.91, partial 67.31). Both ran on synthetic geometry,
so they show parity, not competition performance.

## 13. Quantitative robustness

Blue B2 over 11 instrumented rounds outside its development scenarios
(stage 1 plus the B2 arms of stage 2): 9 rounds 10/10, 2 rounds 0/10 (both
the corridor family). Red R2 over 7 rounds (6 instrumented, 1 native): all
placed and submitted; wall 51.91-95.66. Runtime: B2 59.3-80.8 steps per second, worst-case controller
time 22.90-33.37 ms; R2 84.6-94.0 steps per second, 39.24-48.71 ms. Single
rounds per configuration; no confidence intervals are claimed.

## 14. Regressions and limitations

* No regression of B2 or R2: their code did not change.
* B3 regressed one pair (P4PocketV2: 9/10 against 10/10, with poor health),
  which is why it was rejected.
* The synthetic families were designed with the known mechanisms in mind;
  they are a stress test, not a sample of the organizer's maps. The
  corridor family's failure depends on the center being hidden and on
  bombs being at least as many as drones.
* Disabler zones in the offline simulator act on the drone centre, not its
  body.

## 15. Adoption decisions

| Candidate | Decision | Basis |
|---|---|---|
| B2 (frozen) | retained | B3 failed rule 1 of section 8 (fewer bombs in pair S2-2) |
| B3 | not adopted | 25 against 20 bombs in total, but 9 against 10 in one pair |
| R2 (frozen) | retained | no successor justified by Risk B evidence |

## 16. Official submission checker

At the final agent code on the remote server: workspace mode `Submission
check passed.`; package mode on a validation-only `team000_preliminary.zip`
(24 entries, 60,472 bytes, placeholder identity, built in the
`create_zip_submission.sh` layout) `Submission check passed.` No
`__pycache__` in `solutions/`. The package was not submitted and is not kept
in the repository.

## 17-18. Publication and review package

The commits were checked against the owner's private patterns before each
commit and the hygiene scan over the history before the push; GitHub
`origin/main` and the review package path and SHA-256 are given in the
session's final report and in the package manifest.

## 19. Recommended next action

Retain B2 and R2 for now. Next, narrowly: (1) a B3-like search for an
unknown center that does not cost deliveries elsewhere, evaluated on fresh
C-reserve scenarios; (2) a planner change for passages a drone can fly
through but the inflation closes; (3) for red, separating sealed rooms from
boxes before weighting them as walls. Each needs its own preregistered
evaluation; the C-final scenarios used in stage 2 are now development data.

## 20. Submission

No file was uploaded to the competition platform and no preliminary, red or
blue submission was made.
