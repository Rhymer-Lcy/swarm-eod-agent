#!/usr/bin/env bash
# Run a list of synthetic delivery-scenario jobs (development signals, not
# simulator rounds) with every outcome recorded and nothing overwritten.
#
# Usage, from the repository root, normally under remote_cpu.sh:
#   PYTHON=<absolute env interpreter> devtools/simlab/delivery_batch.sh \
#       <attempt-id> "<scenario> <seed> <agent> [drones] [steps]" ...
#
# Each job writes <job>.json, <job>.err and <job>.rc under
# _local/offline/<attempt-id>/, which must not exist yet. The batch exits
# with the number of failed jobs; STATUS.txt lists every job.
set -uo pipefail

: "${PYTHON:?set PYTHON to the absolute project interpreter}"
case "$PYTHON" in
    /*) ;;
    *) echo "PYTHON must be an absolute path, got $PYTHON" >&2; exit 2 ;;
esac
[ -x "$PYTHON" ] || { echo "PYTHON is not executable: $PYTHON" >&2; exit 2; }

attempt="$1"; shift
out="_local/offline/$attempt"
if [ -e "$out" ]; then
    echo "attempt directory exists, choose a new attempt id: $out" >&2
    exit 2
fi
mkdir -p "$out"
{ echo "attempt $attempt delivery scenarios"; echo "python $PYTHON"; "$PYTHON" --version 2>&1; \
  git rev-parse HEAD; git status --porcelain | wc -l; } > "$out/CONTEXT.txt"

fails=0
for spec in "$@"; do
    read -r scenario seed agent drones steps <<< "$spec"
    drones="${drones:-4}"; steps="${steps:-3000}"
    job="delivery_${scenario}_s${seed}_${agent}_d${drones}_t${steps}"
    "$PYTHON" -m devtools.simlab.delivery_fixtures --scenario "$scenario" --seed "$seed" \
        --agent "$agent" --drones "$drones" --steps "$steps" \
        > "$out/$job.json" 2> "$out/$job.err"
    rc=$?
    echo "$rc" > "$out/$job.rc"
    if [ "$rc" -ne 0 ] || [ ! -s "$out/$job.json" ]; then
        fails=$((fails + 1))
        echo "FAIL rc=$rc $job" | tee -a "$out/STATUS.txt"
    else
        echo "OK $job" >> "$out/STATUS.txt"
    fi
done
echo "batch $attempt: $# jobs, $fails failed" | tee -a "$out/STATUS.txt"
exit "$fails"
