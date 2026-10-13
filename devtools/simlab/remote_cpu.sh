#!/usr/bin/env bash
# Run one bounded CPU-only Python command (tests, offline simulator,
# profiling, analysis) on a shared Linux host.
#
# Usage (from anywhere; host-specific values come from the environment):
#   CPUSET=<CPUs on NUMA node 0> PYTHON=<env interpreter> \
#       devtools/simlab/remote_cpu.sh <timeout_seconds> -m pytest ...
#
# The occupancy gate (CPU-only mode) runs first: the command is deferred
# (exit 75) unless the pinned CPUs are on NUMA node 0 and idle. No GPU is
# needed or used. The command then runs pinned with taskset, with thread
# caps, no user site packages, no bytecode files and a hard timeout. Set
# REPO_DIR to run from another checkout of the repository (for example a
# scratch worktree); its src/ then takes precedence on the import path.
# Set INTERPRETER=bash to run a batch script (for example
# devtools/simlab/delivery_batch.sh) under the same gate and limits; the
# gate itself always runs with PYTHON.
set -euo pipefail

: "${CPUSET:?set CPUSET to a CPU list on NUMA node 0}"
: "${PYTHON:?set PYTHON to the project environment interpreter}"

# Re-run this wrapper itself pinned, so that the gate and every helper
# process stay on the permitted CPUs as well, not only the command.
if [ -z "${_REMOTE_CPU_PINNED:-}" ]; then
    _REMOTE_CPU_PINNED=1 exec taskset -c "$CPUSET" bash "$0" "$@"
fi

timeout_s="$1"
shift

cd "${REPO_DIR:-$(dirname "$0")/../..}"
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY

export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1 OPENCV_NUM_THREADS=1
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"

if ! "$PYTHON" -m devtools.simlab.occupancy --cpus "$CPUSET" --cpu-only; then
    echo "DEFERRED: pinned CPUs not free for this run" >&2
    exit 75
fi

set +e
timeout --kill-after=10 "$timeout_s" taskset -c "$CPUSET" "${INTERPRETER:-$PYTHON}" "$@"
rc=$?
set -e
echo "EXIT $rc"
exit "$rc"
