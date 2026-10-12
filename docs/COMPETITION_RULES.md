# Competition rules and scoring: technical summary

Dates are business dates in UTC+8. This file is an English technical summary
prepared by this project. It is not an official translation.

## Sources and precedence

| Source | Status here |
|---|---|
| Organizer scoring rules `评分规则.pdf` (Chinese, 5 pages, SHA-256 `924595248194328b7475415c3f918326e1c8f4b0d4d029fe639ebc262db9ecc1`, PDF creation date 2026-09-09) | **Scoring authority supplied to this project.** Kept locally and not redistributed (see [PROVENANCE.md](PROVENANCE.md)). |
| Participant guides `doc/participants/01-03-*.md` in the baseline | Authoritative for the development and submission interface. Kept verbatim in Chinese. |
| Evaluation code in the baseline (`launcher.py`, `simulation/reporting/*`, `tools/map_exploration_scoring.py`, `submission_check.py`) | What a local run actually computes. Where it differs from the PDF, the difference is listed under [Discrepancies](#discrepancies-requiring-organizer-clarification) and neither side is changed. |

Formulas below are taken from the PDF and checked line by line against the
code. File references are relative to `src/swarm_rescue/`.

## Two independently scored roles

Red (`place`) and blue (`rescue`) are ranked separately. Each round score is
out of 100.

| Map | Size (px) | Step limit | Wall-clock limit | Default drones | Blast radius (0.12 x shorter side) |
|---|---|---|---|---|---|
| Map01 | 1250 x 800 | 2000 | 90 s | 10 | 96 px |
| Map02 | 1200 x 900 | 5000 | 180 s | 10 | 108 px |
| Map03 | 1113 x 750 | 5000 | 180 s | 10 | 90 px |
| Map04 | 1660 x 1122 | 5000 | 180 s | 10 | 135 px |
| Map05 | 1200 x 1200 | 7200 | 240 s | 10 | 144 px |

Values from `maps/map_0*.py` and `PlaceScoreConfig.blast_radius`. Every
supplied map uses a 6 px border frame and none defines a return area. The
evaluation plan may override the drone count (`number_drones`); the supplied
local plans use 5 drones with 2 bombs each for red and 10 drones for blue.
The maps used in official rounds have not been published to this project.

## Blue / rescue (`simulation/reporting/score_manager.py`)

`score = 0.6 * disposal + 0.2 * health + 0.2 * time`, each term in [0, 100].

* **Disposal (60 %)**: `100 * disposed / total_bombs`. With zero bombs the
  code returns 100 (relevant only to local runs without a bomb layout).
* **Return-area health (20 %)**: if the map has a return area, the summed
  health of the drones inside it at the end; otherwise the summed health of
  all drones; divided by `number_drones * 50` (initial health 50; each
  collision with a wall or drone costs 1; a drone at 0 is removed and still
  counts in the denominator). `map_abstract.compute_score_health_returned`.
* **Time (20 %)**: `clamp((L - t) / (L - 0.5 L), 0, 1) * 100`, where `L` is the
  map step limit and `t` the elapsed step count at round end: full marks at or
  below half the limit, zero at the limit, linear in between, no completion
  gate. A crashed round scores 0 on time.
* **Early termination**: the round ends at the step on which the last bomb is
  disposed (`gui_sr._update_disposal_progress`), so for a completed run `t` is
  the completion step. An unfinished run continues until a limit is reached.
  Per the PDF it then reaches the step limit and scores 0 on time; see
  discrepancy D1 for the wall-clock case.
* **Crash**: the time term is 0; disposed bombs and health are still read from
  the simulation state at the crash.

## Red / place (`simulation/reporting/place_score_manager.py`)

`score = 0.5 * map + 0.2 * bomb + 0.2 * wall + 0.1 * time`, each in [0, 100].

### Map accuracy (50 %) (`tools/map_exploration_scoring.py`)

The submission is an `(H, W)` array passed to `submit_exploration_map(grid)`;
non-zero means obstacle, one cell per world pixel, `col = round(x + W/2)`,
`row = round(H/2 - y)`. Calling it ends the round immediately (the physics
step of that frame is not applied). A shape mismatch, a missing submission or
a crash scores 0.

* **Truth** is a walls-only grid rasterized from the real wall entities
  (collision type `WALL`: line walls, boxes, the border frame). Disposal
  centers, return areas and disabler zones are not walls: drawing them earns
  nothing and is charged as a false positive.
* **Regions**: obstacle depth `d_free` is the Euclidean distance to the
  nearest free pixel, with the space outside the grid counted as free.
  Surface `W = {d_free <= 4}`; deep interior `N = {d_free > 4}`. A standard 6 px
  wall has no deep interior.
* **Credit**: `credit = 1.0 * h_W + 0.0 * h_N`, where `h_W` is the fraction of
  surface pixels that are themselves predicted (exact pixel overlap, distance 0).
  The deep interior carries zero weight.
* **Penalty**: for each predicted pixel `q` at distance `d` from the nearest
  truth wall, `charge(d) = 0` if `d <= 10`, else
  `min(0.1 * ceil((d - 10) / 2), 1.0)`;
  `penalty = 0.2 * sum(charge) / |W|`. The denominator is the surface pixel
  count, so over-drawing cannot dilute it.
* `S_map = clamp(credit - penalty, 0, 1)`; map score `= 100 * S_map`.

PDF reference cases (the PDF does not name the map): perfect 100; 1 px dilation 100;
1 px erosion 73.03; outline only 26.97; blank 0; all black 0 (penalty
5.4677); 50 % random 0 (credit 0.5007, penalty 2.7371). The supplied
`my_drone_place_example.py` submits such a random grid; the first Phase 0 smoke
run reproduced the pattern on Map01 (credit 0.4986, penalty 2.5444, map 0).

### Bomb-layout difficulty (20 %)

Computed offline from reference blue agents run against the red team's
layout; a local run reports it as pending.
`difficulty_i = min(100, max(0, 100 - blue_score_i) * 2)` (saturates at 100 for
a blue score of 50 or less). `aggregate` is the mean (or median, if configured)
over valid reference runs. `bomb = completion * aggregate`, with
`completion = min(1, placed / expected)` and
`expected = number_drones * initial_bombs_per_drone`. See D2 for how failed
reference runs are treated.

### Wall coverage by blasts (20 %)

Each placed bomb destroys every wall pixel within `blast_radius_px =
round(0.12 * min(H, W))` of it; blasts are unioned. Boundary-frame wall pixels
weigh `k = 0.5`, interior wall pixels 1.
`weighted = interior_destroyed + k * boundary_destroyed`.
`wall = 100 * clamp(weighted / reference, 0, 1)`, where `reference` comes from
`wall_theoretical_optimum()`: a deterministic greedy cover using the same
radius, weights and the round's expected bomb count, with candidate centers
restricted to free cells. It is a reproducible **lower bound** on the true
optimum, not a proven optimum (D5), so a layout can reach the clamp at 100.

### Time (10 %)

Same formula and clock as the blue time term (`time_metric = "timestep"`,
`time_best_ratio = 0.5`). The red round ends when any drone submits a map, so
early submission raises the time term at the expense of the others. A crash
scores 0.

### Placement mechanics relevant to scoring

`place_bomb` triggers on a 0 to 1 rising edge. Placement fails silently when
the inventory is empty, the drone holds a bomb, the grasper is active, or
another bomb lies within 24 px. The launcher prints that the competition takes
the **best** red round (D7).

## Competition-mode constraints on agents

* `true_position()`, `true_angle()`, `true_velocity()` and
  `true_angular_velocity()` raise inside `control()` in `--competition` mode and
  in batch evaluation (`simulation/drone/ground_truth_guard.py`); the attributes
  `self.position`, `self.angle`, `self.velocity`, `self.angular_velocity` are
  disabled everywhere. The static checker rejects any call of the `true_*`
  methods anywhere in submitted code.
* Sensors are noisy and can be disabled in special zones: GPS (AR(1) noise),
  compass, odometer, lidar (181 rays, 360 degrees, 300 px), semantic sensor
  (35 rays, 200 px, walls filtered out); communication radius 250 px.
* Runtime modification of framework behaviour from inside `solutions/` is
  prohibited (setattr on framework classes, `sys.modules` replacement,
  `gc.get_objects()`, `ctypes`/`inspect` tampering, bypassing ground-truth
  interception, noise or disabler zones). Submissions are reviewed by an AI
  code review; a violation scores 0 for that submission.

## Discrepancies requiring organizer clarification

Recorded, not acted on: the simulator, the scoring code and the official
documents are left unchanged. Severity reflects the possible effect on a
score or on submission validity.

| ID | Topic | PDF or guide says | Code does | Severity |
|---|---|---|---|---|
| D1 | Wall-clock termination and the time term | An unfinished round runs to the step limit and scores 0 on time. | A round also ends when the wall-clock limit (90-240 s) is exceeded (`gui_sr.on_update`). `elapsed_timestep` then stays below the step limit, so an unfinished blue run, or a red run without a submission, receives a positive time score. Wall-clock limits make results depend on evaluator hardware speed. | High |
| D2 | Failed reference blue runs | Only successful, valid reference runs are used. | With the default `min_successful_references = None`, every reference run must be valid; if any fails or crashes, `aggregate` is 0 and the red bomb score is 0. `crash_difficulty` is defined but unused. | High |
| D3 | Boundary-wall classification | A wall whose bounding box touches the arena boundary is a boundary wall. | A wall is boundary only if its contact with a border spans at least 50 % of that side (`_polygon_is_boundary`); a partition abutting the frame (two on Map01) counts as interior at weight 1, not 0.5. | Medium |
| D4 | Map names and sizes | Refers to MapSimple / MapMedium / MapComplex, 9 px borders on MapComplex, and a ~60 px blast radius on MapSimple. | The baseline ships Map01-Map05 only, all with 6 px borders; radii are 90-144 px. The official map set is unknown. | Medium |
| D5 | "Theoretical optimum" | Full marks equal the map's optimum layout. | A greedy lower bound with free-cell centers; the kernel uses `r^2 + 0.5` while scoring uses `dist <= r`. Scores are clamped at 1. | Low |
| D6 | Penalty saturation | Guide: charges start "from 11 px" and saturate "at >= 30 px"; PDF: the cap is reached "20 px past the dead zone". | Charges start for any `d > 10` (Euclidean) and reach the cap for `d > 28`. | Low |
| D7 | Aggregation across rounds | Not stated in the PDF for either role. | The launcher states that red takes the best round; blue aggregation is not defined in the supplied material (`config_weight` exists). | Medium |
| D8 | Location of `req.txt` | Guide 02, section 0 table: `solutions/req.txt`. | Guide 03, the checker and the packaging script use the repository root. | Low |
| D9 | `team_number` type | Guide 03: must be an integer; `"006"` is wrong. | The checker converts with `int()` and accepts `"006"`. | Low |
| D10 | Packaging of `__pycache__` | Guide 03: the packaging script does not include `__pycache__`. | `create_zip_submission.sh` zips `solutions/` with `zip -r .`, so a `__pycache__` left by a local run is packed and the zip check then rejects the package; the workspace check does not detect it. | Medium |
| D11 | Example place agent | Guide 02, section 9: the example submits an all-zero grid. | `my_drone_place_example.py` submits a 50 % random grid (map score 0). | Low |
| D12 | Rule text "任何行数过多的做法" | Listed among prohibited techniques ("any approach with too many lines"). | No corresponding check exists; the intended meaning is unclear. | Medium |
| D13 | `place_scoring` YAML defaults | Not documented. | A `place_scoring` section that sets `w_wall` or `w_time` without `blast_radius` defaults the radius to 100.0, which fails validation (must be in (0, 1]). The supplied plans have no such section. | Low (latent) |
| D14 | `exploration_threshold_for_time_bonus` | Set to 90 in the supplied rescue plan. | Stored but not used by any score. | Informational |

D1 and D2 can change rankings, and D7 and D12 affect how work should be
planned. The questions that need the organizer's answer are drafted in
[ORGANIZER_QUESTIONS.md](ORGANIZER_QUESTIONS.md); until answered they are
unknown. No agent behaviour may be designed to exploit D1.
