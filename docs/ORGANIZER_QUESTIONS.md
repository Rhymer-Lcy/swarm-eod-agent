# Clarification requests to the competition organizer (draft)

**Status: draft, not sent.** The project owner decides whether and when to
submit it. Until answers arrive, every question below is treated as
**unknown**: the supplied scoring PDF remains the scoring authority, and no
agent behaviour depends on the code behaviour described here.

References are to the supplied baseline `swarm-rescue` 5.1.3 (files under
`src/swarm_rescue/`) and to the scoring rules PDF (`评分规则.pdf`, created
2026-09-09). Further detail is in [COMPETITION_RULES.md](COMPETITION_RULES.md)
(discrepancies D1-D14).

## Q1. Wall-clock termination and the time score (D1)

The PDF states that a round which does not finish runs to the step limit and
therefore scores 0 on time. In `simulation/gui_map/gui_sr.py` (`on_update`) a
round also ends when the wall-clock limit is exceeded (90-240 s on the supplied
maps). `elapsed_timestep` then remains below the step limit, so
`ScoreManager._compute_time_score` awards a positive time score to an
unfinished blue round, and `PlaceScoreManager` does the same for a red round
that ends without a map submission.

* Is the PDF or the code authoritative for rounds ended by the wall-clock limit?
* Do official evaluations enforce a wall-clock limit, and on what hardware?

## Q2. Failed reference blue runs in the red bomb-difficulty score (D2)

The PDF says only successful, valid reference runs are used. In
`simulation/reporting/place_score_manager.py` (`compute_bomb_score`), with the
default `min_successful_references = None`, every reference run must be valid;
if any one fails or crashes, the aggregate difficulty and the red bomb score
become 0. The `crash_difficulty` setting is defined but not used.

* Which behaviour applies in the official evaluation, and with which
  `place_scoring` settings?

## Q3. Boundary-wall classification (D3)

The PDF classifies a wall as a boundary wall when its bounding box touches the
arena boundary. `_polygon_is_boundary` in `place_score_manager.py` requires the
wall's contact with a border to span at least 50 % of that side, so partitions
that abut the frame (two on Map01) count as interior walls with weight 1
instead of 0.5.

* Which rule is authoritative for the wall-coverage score?

## Q4. Official maps (D4)

The PDF refers to MapSimple, MapMedium and MapComplex (with 9 px borders on
MapComplex and a blast radius of about 60 px on MapSimple). The baseline ships
Map01-Map05, all with 6 px borders and blast radii of 90-144 px.

* Which maps, drone counts, bomb counts and zone configurations will the
  official rounds use, and will they be published before evaluation?

## Q5. Aggregation across rounds and maps (D7)

The PDF defines per-round scores only. The launcher states that the red score
is the best round; the blue aggregation is not described, and evaluation plans
carry a `config_weight` field.

* How are round scores combined across rounds and maps for each role, and is
  `config_weight` used?

## Q6. Meaning of "任何行数过多的做法" (D12)

Section 0 of `doc/participants/02-队伍开发指南.md` lists "任何行数过多的做法"
("any approach with too many lines") among prohibited techniques. No checker
rule corresponds to it.

* What does this prohibition cover? Is there a code-size limit?

## Q7. Evaluation image

The participant guide describes a Python 3.11 image with the dependencies
preinstalled and Mesa for OpenGL 4.4, distributed as an offline archive.
`pyproject.toml` sets lower bounds only (for example `numpy>=1.24.4`).

* Which package versions does the image contain (numpy, pandas,
  opencv-contrib-python, scikit-image, pymunk, pyglet), and when will the base
  image archive be distributed to participants?

## Q8. Redistribution of the baseline

The baseline archive contains one license file: the MIT license of the upstream
Swarm-Rescue project. The competition-specific additions (EOD mechanics,
Map01-Map05, participant guides, checker) carry no separate notice.

* Are participants permitted to keep the baseline, including these additions,
  in a public source repository under that license, or should the organizer's
  additions be removed from public copies?
