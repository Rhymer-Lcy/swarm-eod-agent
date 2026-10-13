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

## Setup procedures

Each environment is a dedicated conda environment; never install into `base`,
a system Python or another project's environment.

**Windows workstation** (PowerShell or Git Bash, from the repository root):

```bash
conda env create -f environments/swarm-eod-agent.conda.yml
conda activate swarm-eod-agent
python -m pip install -e ".[dev]"
cp devtools/templates/team_info.example.yml src/swarm_rescue/solutions/team_info.yml
python -m pytest -q -p no:cacheprovider devtools/tests
python devtools/probe_opengl.py
```

**Ubuntu host with internet access**: the same five commands, then run
GL-dependent work with `PYGLET_HEADLESS=True PYGLET_HEADLESS_DEVICE=<n>` when no
display server exists.

**Ubuntu host without internet access** (the procedure used on the project's
server):

1. On a connected machine, download Linux wheels for the locked set:
   `python -m pip download --no-deps --only-binary=:all: --python-version 3.11
   --implementation cp --abi cp311 --abi abi3 --abi none --platform
   manylinux_2_35_x86_64 --platform manylinux_2_28_x86_64 --platform
   manylinux2014_x86_64 -r <requirements without fpdf> -d wheels/`, add the
   `fpdf==1.7.2` sdist (`--no-binary=:all:`) and wheels for `setuptools` and
   `wheel`, and copy `wheels/` to the host.
2. Create the conda environment (the Python interpreter itself must come from a
   conda channel or a local package cache).
3. `python -m pip install --no-index --find-links wheels setuptools wheel`, then
   `python -m pip install --no-index --find-links wheels --no-build-isolation -e ".[dev]"`.
4. Copy the team template as above and run with `PYTHONNOUSERSITE=1`.

What was verified: the Windows environment was created exactly as above. The
server environment was created with the offline procedure from wheels
downloaded for the Windows-resolved versions. **Installing from the lock files
into a clean environment has not been exercised**; the locks record what was
resolved, not a proven reproduction recipe.

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
measured here.

Scope of this conclusion: the participant guides require OpenGL 4.4 / GLSL
4.40, and the organizer's image provides it through Mesa. The server context
*reports* 3.3; it works only because the NVIDIA 535 driver accepts and runs a
4.40 compute shader in that context. Shader compilation, the ray parity tests
and two smoke runs show that the local pipeline functions; they do **not**
establish behavioural equivalence with the evaluator's Mesa OpenGL stack,
which has not been tested.

EGL device index versus physical GPU: **resolved for the project's server
(2026-10-13), still host-dependent in general.** `PYGLET_HEADLESS_DEVICE`
indexes the EGL device list, whose order is not guaranteed to match
`nvidia-smi` or CUDA numbering. `devtools/egl_device_map.py` resolves every
EGL device read-only (no context is created): EGL device, DRM node, PCI
address, then the `nvidia-smi` index and UUID, cross-checked with
`EGL_CUDA_DEVICE_NV`. On the project's server EGL index k is `nvidia-smi`
index k and CUDA device k for all eight GPUs, and every GPU is attached to
NUMA node 0. The GPU used in Phase 1 is recorded by UUID in the private
operations notes. Re-run the mapping on any other host.

## Remote execution safety rules

The server is shared. These rules apply to every job:

1. **Inspect first**: `lscpu`, `uptime`, `/proc/pressure/cpu`, `free -g`,
   `df -h`, `nvidia-smi` (including `--query-compute-apps`), and for busy
   processes their CPU affinity (`taskset -cp`). Identify other jobs by working
   directory, never by user name.
2. **NUMA node 0 only.** This project may use only NUMA node 0, pinned with
   `taskset -c <node 0 CPU list from lscpu>`, and only while the inspection
   shows node 0 free of other work. NUMA node 1 is used by other personnel and
   is **never** a fallback. If node 0 is busy, or whether it is free cannot be
   established, **defer the run**.
3. **GPUs are checked, not assumed.** Before any GPU use, confirm with
   `nvidia-smi --query-compute-apps` that the intended GPU has no compute
   processes. A GPU that is idle at that moment is not thereby allocated to
   this project; if allocation is in doubt, defer. Because the EGL device
   index is not mapped to a physical GPU (see above), GPU-dependent runs need
   every GPU's state checked, not only the one intended.
4. **Bound every run**: `timeout`, thread caps (`OMP_NUM_THREADS`,
   `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`, `NUMEXPR_NUM_THREADS`,
   `OPENCV_NUM_THREADS`), one simulation process unless a parallel mode has been
   tested.
5. **Never** terminate, renice, re-pin or otherwise touch another user's
   processes; never use `pkill`, `killall` or `pgrep -f` patterns; terminate
   only this project's own PIDs. No `sudo`, no system packages, no driver or
   display changes.
6. **Keep output inside the project**: logs and results under the clone's
   ignored `_local/` (`SWARM_RESCUE_RESULTS_DIR`), pytest `--basetemp` there too.
   Run Python with `PYTHONNOUSERSITE=1` (a user-site install would shadow every
   environment on the shared account) and `PYTHONDONTWRITEBYTECODE=1`.
7. **Sustained work** (multi-round evaluations, sweeps, training) needs a
   separate authorization from the owner, recorded with the CPU set, GPU and
   duration chosen.
8. `/usr/bin/time` and `numactl` are not installed; time runs with bash's
   `SECONDS` or `time` keyword.
9. Since Phase 1 the rules above are applied by
   `devtools/simlab/remote_run.sh`: it runs the occupancy gate
   (`devtools/simlab/occupancy.py`: every GPU without compute processes and
   below 50 MiB and 5 % use, the pinned CPUs below 25 % busy) and defers with
   exit status 75 otherwise, then runs the command with `taskset`, thread caps,
   EGL headless on the chosen device, `PYTHONNOUSERSITE=1`,
   `PYTHONDONTWRITEBYTECODE=1` and a hard `timeout`, and prints a report-only
   occupancy snapshot afterwards. Simulation rounds go through
   `devtools/simlab/runner.py`, which enforces a per-phase round budget
   through its ledger (see [EXPERIMENTS.md](EXPERIMENTS.md)).

10. **Remote-first Python (owner directive, 2026-10-13).** All Python
    execution, including unit tests, static checks written in Python, the
    submission checker, offline simulation, profiling and analysis, runs in the
    project environment on the server; the workstation is for editing, Git,
    GitHub and SSH and does not run Python. The interpreter is called by its
    absolute path: a non-interactive SSH shell does not initialize conda and its
    `PATH` contains no `python` (found when an ad hoc batch failed; see
    EXPERIMENTS.md, 7.9). CPU-only work goes through
    `devtools/simlab/remote_cpu.sh`: the occupancy gate in CPU-only mode
    requires the pinned CPUs to be idle and on NUMA node 0 (it reads
    `/sys/devices/system/node/node0/cpulist`), then the command runs with
    `taskset`, one thread per library and a timeout; no GPU is required.
    Batches of offline runs go through `devtools/simlab/offline_batch.sh`,
    which records every job's exit status and never overwrites an earlier
    attempt. The official `check_submission.sh` picks `python3` from `PATH`;
    on the server it is run with the environment's `bin` first in `PATH` and
    `VIRTUAL_ENV` set, so that it uses the project interpreter (unmodified
    script). Uncommitted work is tested in a separate scratch worktree of the
    server clone, so the clone used for ledgered rounds stays clean and at a
    committed state.

### Synchronisation and single source of truth

The workstation repository is the only place commits are made, and GitHub is
its published mirror. The server has no direct internet access, so its clone
is updated from `git bundle` files copied over the LAN (`git fetch <bundle>
main`, then `git merge --ff-only`). The server clone never contains `_local/`
material from the workstation, and its `team_info.yml` is the placeholder
template.

## Test tiers

| Tier | Content | Where | Command |
|---|---|---|---|
| Devtools | Hygiene scanner, review-package generator, checker characterization, agent unit and offline-integration tests (`test_eod_agent.py`), truth-logger test | Both | `python -m pytest -q -p no:cacheprovider devtools/tests` |
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
| Devtools tests | PASS 22/22 | PASS 22/22 |
| Tier A | PASS 84/84 | PASS 84/84 (5 s) |
| Tier B | NOT RUN (by design) | PASS 134/134 (13 s), including ray parity 10/10 |
| OpenGL probe | PASS (4.6) | Default FAIL (no display); EGL PASS (3.3 context, shader compiled) |
| Smoke run | NOT RUN | PASS in 2 of 2 runs: both rounds completed, no crash, 24-28 s wall-clock |

Smoke numbers come from the supplied example agents and are not performance
claims; runs are stochastic (random initial headings, sensor noise, the example
agent's random map). First run: red placed 10/10 bombs, submitted a random grid
(map 0), wall 25.8, time 40.3 (partial 9.2 of a reachable 80); blue disposed
0/10 bombs, health 99.0, time 0 (19.8 of 100), at 189 steps per second. Second
run, at commit `0fd1b76`: red submitted at step 804 (wall 25.0, time 100.0,
partial 15.0); blue 0/10, health 98.4 (19.7). Later commits changed only
`docs/` and `devtools/`, not the baseline.

The server test run at `0fd1b76` first reported errors in all three pytest
tiers; the cause was the harness (a `--basetemp` directory whose parent did
not exist, `FileNotFoundError`), and the re-run with the directory created
passed 20/20 (the devtools suite then had 20 tests), 84/84 and 134/134. Both
outputs are kept in the review evidence. Two devtools tests were added
afterwards, and the 22-test devtools suite was re-run on both hosts at the
final commit.

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
* R5: agent compute. On the server the Phase 1 blue candidate ran at about
  50 simulation steps per second with 10 drones (the supplied examples at
  200-260); the Phase 2 candidate B1 runs at 70-73 on Map01 and Map02 and
  66-68 on Map03-Map05. A full 7200-step round at 68 steps/s would take about
  106 s of its 240 s limit, so an evaluator more than about 2.3 times slower
  than this server would hit the wall-clock limit on such a round (see D1).
  The evaluator's speed is unknown.
