# Phase 0 completion report

Business date 2026-10-12 (UTC+8). Scope: workspace initialization, source
preservation, Git preparation, environment audit, documentation, compliance
review and limited baseline validation. No agent strategy was developed,
nothing was submitted, and nothing was published.

## Disposition: PASS WITH BLOCKERS

The engineering foundation is in place and verified. The blockers do not
prevent Phase 1 development, but they prevent publication and limit how far
local results can be trusted to transfer to the official evaluation.

## Completed and verified

| Area | Result | Evidence |
|---|---|---|
| Source preservation | Both originals re-hashed (match), moved by copy, verify, delete into the ignored archive with a manifest; neither modified | [PROVENANCE.md](PROVENANCE.md) |
| Baseline import | 202 files extracted after archive safety checks; import commit verified byte-for-byte and mode-for-mode against the ZIP; tagged `baseline-supplied-5.1.3` | `baseline/integrity.json` in the review package |
| Repository hygiene | LF policy; baseline `.gitignore` kept and extended; team identity untracked with a placeholder template; hygiene scan of HEAD and history: 0 errors | [WORKFLOW.md](WORKFLOW.md) |
| Rules and interface | Scoring for both roles checked against the code; 14 discrepancies recorded; submission interface and checker behaviour documented and characterized by tests | [COMPETITION_RULES.md](COMPETITION_RULES.md), [SUBMISSION.md](SUBMISSION.md) |
| Environments | Python 3.11.17 environments on the workstation and the server with identical package sets (except `tzdata`); locks recorded | [ENVIRONMENT.md](ENVIRONMENT.md) |
| Server audit | Topology, memory, disk, GPUs, GL stack, occupancy inspected read-only; no system changes; no other user's process touched | [ENVIRONMENT.md](ENVIRONMENT.md) |
| Validation | Windows: checker PASS, devtools 20/20, tier A 84/84. Server: checker PASS, devtools 20/20, tier A 84/84, tier B 134/134 under EGL headless, smoke run PASS (2 of 2) | [ENVIRONMENT.md](ENVIRONMENT.md#phase-0-results) |
| Review packages | Generator with hygiene gate and post-write verification; Phase 0 package built from the final commit | [WORKFLOW.md](WORKFLOW.md#review-packages) |

## Not run or not verified

* GL-dependent tests on Windows: NOT RUN (by design).
* Video capture, replay viewer, interactive tools: NOT RUN.
* Installing from the lock files on a clean environment: NOT VERIFIED.
* Behaviour under the organizer's evaluation image (Mesa OpenGL, unknown
  package versions): NOT VERIFIED; the image is not available to this project.
* The mapping of EGL device indices to physical GPUs on the server: NOT VERIFIED.
* The baseline ZIP's commit ID (from its ZIP comment) against any remote:
  NOT VERIFIED.

## Blockers and open questions

| ID | Item | Owner of the answer | Blocks |
|---|---|---|---|
| B1 | Redistribution rights for the supplied baseline and rules on publishing participant solutions | Organizer, via the owner | Public GitHub repository |
| B2 | Discrepancies D1 (wall-clock end gives time credit), D2 (one failed reference run zeroes the red bomb score), D7 (round aggregation), D12 (the "too many lines" rule) | Organizer | Trustworthy local scoring and planning |
| B3 | Evaluator package versions and OpenGL implementation | Organizer | Confidence that local results transfer |
| B4 | Official map set (the PDF names maps absent from the baseline) | Organizer | Map-specific tuning |
| B5 | Identity of the short GPU job seen on the server's node 0 during Phase 0, and current node ownership | Owner | Sustained server runs |
| B6 | The registered team identity has not been entered locally (placeholder in use) | Owner | Packaging a real submission |

## Recommended next phase: Phase 1, agent foundations

Goal: a minimal, compliant, sensor-only agent framework under
`solutions/` for both roles (state estimation from noisy GPS, compass and
odometry; lidar occupancy mapping at 1 px resolution; safe motion), with a
local evaluation harness that reproduces the official score formulas outside
`solutions/` and logs every score term.

Entry conditions:

1. The owner accepts this report and the review package.
2. Server use for repeated simulation runs is authorized, with the CPU set
   (one NUMA node) and GPU confirmed after B5 is answered.
3. Decision on publication (B1): publish now, publish only project layers, or
   keep private until the competition ends.
4. Questions B2-B4 sent to the organizers; until answered, Phase 1 treats the
   PDF as the scoring authority and does not rely on D1.
5. The real team identity is written locally (B6) before the first packaging
   rehearsal.
