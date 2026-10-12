# Environments, remote execution and testing

Dates are business dates in UTC+8. Results quoted here were measured on
2026-10-12; evidence is in the Phase 0 review package. Host addresses,
accounts and paths of the remote server are kept out of this repository.

## Target interpreter

Python **3.11**: the organizer's evaluation image runs Python 3.11
(`doc/participants/03-*.md`, section 10.3). `pyproject.toml` allows `>=3.8` and
pins only `arcade==2.6.17`; every other dependency has a lower bound only.

## Environment specification

* `environments/swarm-eod-agent.conda.yml`: conda-forge `python=3.11` and `pip`.
* `environments/lock/windows-py311.txt`, `environments/lock/linux-py311.txt`:
  the package sets the resolver chose on 2026-10-12, recorded with
  `pip freeze`. They are identical except `tzdata` (a Windows-only dependency
  of pandas).

```bash
conda env create -f environments/swarm-eod-agent.conda.yml
conda activate swarm-eod-agent
python -m pip install -e ".[dev]"
cp devtools/templates/team_info.example.yml src/swarm_rescue/solutions/team_info.yml
```

What was verified: the Windows environment was created exactly this way. The
Linux environment was created from the same spec and installed with the same
`pip install -e ".[dev]"`, offline, from wheels pre-downloaded for the
Windows-resolved versions (the server has no direct internet access).
Installing from the lock files has not yet been exercised separately.

Resolved versions of note: arcade 2.6.17, pyglet 2.0.dev23 (pre-release,
required by arcade), pymunk 6.4.0, numpy 2.4.6, pandas 3.0.6,
opencv-contrib-python 5.0.0.93, scikit-image 0.24.0, Pillow 9.3.0 (held back
by arcade), PySide6 6.12.0, pytest 9.1.1.

**Unresolved risk R1:** the organizer image's package versions are not
published. The open lower bounds let pip choose new major releases (numpy 2,
pandas 3, OpenCV 5) that the evaluator may not have. Agent code should use
only stable, long-standing APIs of these packages until the image's versions
are known.

## Windows workstation (development and Git)

| Item | Value |
|---|---|
| OS | Windows 11 Pro for Workstations, build 26300 |
| GPU / OpenGL | NVIDIA GeForce RTX 5080, driver 617.42; arcade context OpenGL 4.6, GLSL 4.60; baseline compute shader compiles |
| Environment | conda `swarm-eod-agent`, CPython 3.11.17 (conda-forge) |
| Git / GitHub CLI | Git 2.56.0; gh 2.92.0, authenticated |

Role: editing, review, Git, documentation, static checks and the GL-free test
tier. Simulation-scale work goes to the remote server. The upstream install
guide reports Windows-specific OpenGL problems; the GL-dependent test tier was
deliberately not run on Windows in Phase 0.

## Remote Linux server (isolated execution)

| Item | Value |
|---|---|
| OS | Ubuntu 22.04.5 LTS, kernel 5.15, glibc 2.35, x86-64 |
| GPUs | NVIDIA data-center GPUs, driver 535.216.03 |
| Display | none: no X server, no Xvfb/`xvfb-run`, no `glxinfo` |
| GL libraries | glvnd with the NVIDIA EGL vendor library; Mesa 23.2.1 GLX/DRI; no Mesa EGL |
| Internet | none by default; package installs go through the workstation |
| Environment | conda `swarm-eod-agent`, CPython 3.11.17, in the shared conda installation |

### OpenGL capability (measured with `devtools/probe_opengl.py`)

| Mode | Result |
|---|---|
| Default (no `DISPLAY`) | **FAIL**: `NoSuchDisplayException`; no context. |
| `PYGLET_HEADLESS=True` (EGL, NVIDIA) | Context created on the data-center GPU; it reports **OpenGL 3.3 core** (the version arcade requests) and GLSL "3.30 NVIDIA via Cg compiler", yet the `#version 440` compute shader compiles. |
| Functional check | `tests/test_ray_compute_gpu_cpu.py`: **10/10 PASS** under EGL headless; GPU lidar and semantic rays match the CPU reference. |

Conclusion: simulation works on the server through pyglet's EGL headless
mode, selected by environment variables only (`PYGLET_HEADLESS=True`,
`PYGLET_HEADLESS_DEVICE=<n>`), with no change to framework code. The baseline's
`xvfb-run` recipe cannot be used there (no Xvfb). Even where Xvfb exists it is
normally served by Mesa's software rasterizer rather than the GPU; that was not
measured here. Caveats: the context
does not *report* 4.4, so the behaviour depends on the NVIDIA driver accepting
4.40 shaders in a 3.3 context (true for driver 535); the evaluator's Mesa
stack is a different implementation; and the mapping from EGL device index to
physical GPU was not verified.

## Remote execution safety rules

The server is shared. These rules apply to every job:

1. **Inspect first**: `lscpu`, `uptime`, `/proc/pressure/cpu`, `free -g`,
   `df -h`, `nvidia-smi` (including `--query-compute-apps`), and for busy
   processes their CPU affinity (`taskset -cp`). Identify other jobs by working
   directory, never by user name.
2. **One NUMA node at most** for this project, chosen from the current
   arrangement. Use node 0 only while the inspection shows it is free of other
   work; if another workload is pinned there, use the other node or wait. Pin
   with `taskset -c <node CPU list>`.
3. **Bound every run**: `timeout`, thread caps (`OMP_NUM_THREADS`,
   `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`, `NUMEXPR_NUM_THREADS`,
   `OPENCV_NUM_THREADS`), one simulation process unless a parallel mode has been
   tested, a GPU without other compute processes.
4. **Never** terminate, renice, re-pin or otherwise touch another user's
   processes; never use `pkill`, `killall` or `pgrep -f` patterns; terminate
   only this project's own PIDs. No `sudo`, no system packages, no driver or
   display changes.
5. **Keep output inside the project**: logs and results under the clone's
   ignored `_local/` (`SWARM_RESCUE_RESULTS_DIR`), pytest `--basetemp` there too.
   Run Python with `PYTHONNOUSERSITE=1` (a user-site install would shadow every
   environment on the shared account) and `PYTHONDONTWRITEBYTECODE=1`.
6. **Sustained work** (multi-round evaluations, sweeps, training) needs a
   separate authorization from the owner, recorded with the CPU set, GPU and
   duration chosen.
7. `/usr/bin/time` and `numactl` are not installed; time runs with bash's
   `SECONDS` or `time` keyword.

### Synchronisation and single source of truth

The workstation repository is the only place commits are made. Until the
public GitHub repository is approved, the server clone is updated from
`git bundle` files copied over the LAN (`git fetch <bundle> main`, then
`git merge --ff-only`); afterwards it will fetch from GitHub. The server clone
never contains `_local/` material from the workstation, and its
`team_info.yml` is the placeholder template.

## Test tiers

| Tier | Content | Where | Command |
|---|---|---|---|
| Devtools | Hygiene scanner, review-package generator, checker characterization | Both | `python -m pytest -q -p no:cacheprovider devtools/tests` |
| A (no GL) | `test_bomb_deployer`, `test_drone_distance_sensors`, `test_generate_mock_map_prediction`, `test_json_to_map`, `test_map_exploration_properties`, `test_map_wall_editor`, `test_score_manager`, `test_timer`, `test_trajectory`, `test_utils`, `test_utils_noise`, `test_wall` | Both | `python -m pytest -q -p no:cacheprovider <files>` |
| B (needs a GL context) | The other 15 files in `tests/` | Server, EGL headless | `PYGLET_HEADLESS=True PYGLET_HEADLESS_DEVICE=<n> python -m pytest -q -p no:cacheprovider <files>` |
| Static checker | Workspace and package checks | Both | `./check_submission.sh [teamNNN_<step>.zip]` |
| Smoke | One red round and one blue round on Map01 with the example agents, `--headless --competition` | Server | `python -m swarm_rescue.launcher --headless --competition -c <local plan>` |

The tier A/B split is by inspection of the imports (playgrounds, maps, GUI or
arcade), confirmed by the fact that tier B needed a GL context to run.

### Phase 0 results

| Check | Windows | Server |
|---|---|---|
| `pip check` | PASS | PASS |
| Workspace checker (demo or placeholder identity) | PASS | PASS (shell wrapper) |
| Devtools tests | PASS 20/20 | PASS 20/20 |
| Tier A | PASS 84/84 | PASS 84/84 (5 s) |
| Tier B | NOT RUN (by design) | PASS 134/134 (13 s), including ray parity 10/10 |
| OpenGL probe | PASS (4.6) | Default FAIL (no display); EGL PASS (3.3 context, shader compiled) |
| Smoke run | NOT RUN | PASS: both rounds completed, no crash, 28 s wall-clock |

Smoke numbers come from the supplied example agents and are not performance
claims: red placed 10/10 bombs, submitted a random grid (map 0), wall 25.8,
time 40.3 (partial 9.2 of a reachable 80); blue disposed 0/10 bombs, health
99.0, time 0 (19.8 of 100), at 189 steps per second.

## Known blockers and open issues

* R1 (above): evaluator package versions unknown.
* R2: no display server on the remote server; only EGL headless works there.
  Video capture and the interactive tools (`bomb_editor`, keyboard examples)
  were not tested.
* R3: whether OpenGL behaviour under the evaluator's Mesa stack matches the
  NVIDIA path is unknown.
* R4: GPU ownership on the server is shared and dynamic: during Phase 0 an
  unrelated short job briefly used four GPUs with node-0 CPU affinity. Every
  run must re-inspect occupancy.
