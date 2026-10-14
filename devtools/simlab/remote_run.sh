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

# Refuse, before pinning anything (the wrapper re-runs itself under taskset
# below), a CPU list that is not entirely on NUMA node 0. Until Phase 4 the
# check ran only inside the gate, after the re-run, so a mistaken list put
# the wrapper and the gate on the wrong node for about a second.
_expand_cpus() {
    local part out=""
    for part in ${1//,/ }; do
        if [[ "$part" == *-* ]]; then out="$out $(seq "${part%-*}" "${part#*-}")"; else out="$out $part"; fi
    done
    echo $out
}
if [ -z "${_NODE0_CHECKED:-}" ]; then
    node0=$(cat /sys/devices/system/node/node0/cpulist 2>/dev/null || true)
    [ -n "$node0" ] || { echo "REFUSED: NUMA node 0 CPU list unknown" >&2; exit 75; }
    allowed=" $(_expand_cpus "$node0") "
    for c in $(_expand_cpus "$CPUSET"); do
        case "$allowed" in *" $c "*) ;; *) echo "REFUSED: CPU $c is not on NUMA node 0" >&2; exit 75 ;; esac
    done
    export _NODE0_CHECKED=1
fi
# Re-run this wrapper itself pinned, so that the gate and every helper
# process stay on the permitted CPUs as well, not only the command.
if [ -z "${_REMOTE_RUN_PINNED:-}" ]; then
    _REMOTE_RUN_PINNED=1 exec taskset -c "$CPUSET" bash "$0" "$@"
fi

timeout_s="$1"
shift

cd "$(dirname "$0")/../.."
unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY

export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
export NUMEXPR_NUM_THREADS=2 OPENCV_NUM_THREADS=2
# OpenCV reads OPENCV_FOR_THREADS_NUM; it ignores OPENCV_NUM_THREADS and
# otherwise sizes its pool from the CPU affinity; two threads, because its
# single-threaded distance transform is not repeatable (Phase 4, stable_cv.py).
export OPENCV_FOR_THREADS_NUM=2

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
