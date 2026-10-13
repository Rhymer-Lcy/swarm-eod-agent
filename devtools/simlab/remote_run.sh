#!/usr/bin/env bash
# Run one bounded simlab command on a shared Linux host.
#
# Usage (from anywhere; host-specific values come from the environment):
#   CPUSET=<node-0 CPU list> EGL_DEVICE=<n> PYTHON=<env interpreter> \
#       devtools/simlab/remote_run.sh <timeout_seconds> -m devtools.simlab.runner ...
#
# The occupancy gate runs first and the command is deferred (exit 75) unless
# every GPU is idle and the pinned CPUs are free. The command then runs pinned
# with taskset, with thread caps, headless EGL on EGL_DEVICE, no user site
# packages, no bytecode files and a hard timeout. A report-only occupancy
# snapshot is printed afterwards for the record.
set -euo pipefail

: "${CPUSET:?set CPUSET to the CPU list of the permitted NUMA node}"
: "${EGL_DEVICE:?set EGL_DEVICE to the EGL device index of the chosen GPU}"
: "${PYTHON:?set PYTHON to the project environment interpreter}"

timeout_s="$1"
shift

cd "$(dirname "$0")/../.."
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY

export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
export NUMEXPR_NUM_THREADS=2 OPENCV_NUM_THREADS=2

if ! "$PYTHON" -m devtools.simlab.occupancy --cpus "$CPUSET" --egl-device "$EGL_DEVICE"; then
    echo "DEFERRED: host not free for this run" >&2
    exit 75
fi

export PYGLET_HEADLESS=True PYGLET_HEADLESS_DEVICE="$EGL_DEVICE"
set +e
timeout --kill-after=10 "$timeout_s" taskset -c "$CPUSET" "$PYTHON" "$@"
rc=$?
set -e
"$PYTHON" -m devtools.simlab.occupancy --cpus "$CPUSET" --egl-device "$EGL_DEVICE" --report-only || true
echo "EXIT $rc"
exit "$rc"
