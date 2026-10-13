# Agent entry points, modification boundaries and submission

Summary of `doc/participants/02-*.md` and `03-*.md` (authoritative, Chinese)
as checked against `src/swarm_rescue/submission_check.py`,
`scripts/check_submission.py`, `check_submission.sh` and
`create_zip_submission.sh`. Phase 0 prepared and verified this workflow
locally; nothing has been submitted.

## Where competition code may live

| Allowed | Prohibited |
|---|---|
| `src/swarm_rescue/solutions/my_drone_eval.py` and additional modules under `src/swarm_rescue/solutions/` | Any change to `simulation/`, `maps/`, `config/`, `tools/`, `resources/` or `launcher.py` |
| `src/swarm_rescue/solutions/team_info.yml` (local only, see below) | Runtime patching of framework behaviour from inside `solutions/` |
| Root `req.txt` (one package per line; empty is valid) | Ground-truth APIs in agent code |

In image mode the evaluator mounts the framework read-only from the
organizer's repository and runs only `solutions/` from the submitted image;
framework layers changed in an image make the submission `invalid_input`.

Project tooling (`devtools/`, `docs/`, `environments/`) lives outside
`solutions/` so it is never packaged.

## Entry point

`my_drone_eval.py` must define `drone_class_for_mode(mode)` (preferred;
`mode` is `"rescue"` or `"place"`) or a class `MyDroneEval`; the function wins
if both exist. The checker only verifies by AST that one of them is defined
and that the file parses. Drone classes subclass
`simulation.drone.drone_abstract.DroneAbstract` and implement
`define_message_for_all()` and `control()`; red commands add `place_bomb`.
Since Phase 1 the entry point returns this project's agents:
`EodRescueDrone` for `rescue` and `EodPlaceDrone` for `place`
(`solutions/eod/shell.py`). Their modules live in the subpackage
`solutions/eod/` and are imported as `swarm_rescue.solutions.eod`; the
packaging script zips them as `eod/...` at the package root, which the checker
accepts (no entry contains a forbidden marker or prefix). The agents need no
packages beyond the framework's own dependencies (NumPy, OpenCV; scikit-image
is used when importable, with an OpenCV fallback), so the root `req.txt`
stays empty. They print nothing.

Validation on 2026-10-13: `scripts/check_submission.py` passed in workspace
mode, and on a test package built like `create_zip_submission.sh` (21 entries,
47,354 bytes) with the placeholder identity and the validation-only name
`team000_preliminary.zip`. That package was not submitted and is not kept in
the repository. A passing static check does not show that online evaluation
will succeed.

## Package layout

```
teamNNN_<preliminary|semifinal|final>.zip
├── my_drone_eval.py      required, at the zip root
├── team_info.yml         required, team_number must equal NNN
├── req.txt               required, may be empty
├── image_ref.txt         optional; empty means "run the code in this zip"
└── other .py modules     optional; model weights allowed (size warnings apply)
```

The checker rejects: a name not matching `^team\d{3}_(preliminary|semifinal|final)\.zip$`;
required files missing or nested in a folder; entries containing `.venv/`,
`swarm_rescue/`, `__pycache__/`, `.git/`, `node_modules/`, or starting with
`maps/`, `simulation/`, `config/`; `launcher.py`; deprecated API names
(`WoundedPerson`, `RescueCenter`, ...); any `true_position()`-style call; direct
`self.position`-style access inside a method named `control`; `req.txt` lines
with paths, URLs or several packages; an invalid `image_ref.txt`. It warns on
more than 15 `print()` calls, `imshow`/`waitKey`, `import torch` and large
packages (500 KiB code-only, 30 MiB with weights, 100 MiB hard warning).

The checker is static. Passing it does not show that the agent runs; the
guide requires a full `--competition` simulation before submitting.

## Team identity workflow

The registered identity must never enter this public repository, yet the
launcher (`TeamInfo()` at start-up) and both scripts read
`src/swarm_rescue/solutions/team_info.yml`. Therefore:

1. The path is ignored by Git and was removed from tracking; the baseline's
   demonstration file remains in the archived original and in the import commit.
2. A fresh clone creates it from the placeholder template:
   `cp devtools/templates/team_info.example.yml src/swarm_rescue/solutions/team_info.yml`.
   The placeholder (team 0) is valid for local simulation only.
3. Before packaging, the real values are written into that local file, and the
   real team and member names are added to the owner's private pattern list so
   that `devtools/check_repo_hygiene.py` blocks them everywhere.

## Packaging procedure (for a later phase)

1. Remove `src/swarm_rescue/solutions/__pycache__/` (D10) and run Python with
   `PYTHONDONTWRITEBYTECODE=1` during local runs.
2. `./check_submission.sh` (workspace mode).
3. `./create_zip_submission.sh` (interactive; requires the `zip` program; packs
   everything under `solutions/` plus root `req.txt`). Generated
   `team*_*.zip` files are ignored by Git.
4. `./check_submission.sh teamNNN_<step>.zip` must print
   `Submission check passed.`
5. Inspect `unzip -l` for unexpected files before sending the package by the
   organizer's channel. Submission is a manual owner action.

## Image mode (recorded, not used in Phase 0)

A non-empty `image_ref.txt` (`[registry/]repo[:tag][@sha256:<64 hex>]`, one
line, at the zip root) makes the evaluator pull that image and run its
`/app/src/swarm_rescue/solutions/`; the zip code and `req.txt` are then
ignored, but the three required files must still be present and team identity
is always taken from the zip. The image must be `linux/amd64`, built `FROM` the
organizer's base image (distributed offline as a tar.gz, Python 3.11,
dependencies preinstalled, no PyTorch, Mesa for OpenGL 4.4) without altering its
layers, and pullable by the evaluator without interactive login. Pinning by
digest is recommended.

## Characterization tests

`devtools/tests/test_submission_checker_characterization.py` pins the supplied
checker's behaviour on synthetic fixtures (team 006, `FIXTURE-TEAM`): a flat
minimal package passes; nested files, `__pycache__`, name/number mismatch and
ground-truth calls fail; a quoted `"006"` passes (D9); a `__pycache__` that the
workspace check accepts makes the zip check fail (D10); the placeholder
template is valid.
