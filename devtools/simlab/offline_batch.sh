#!/usr/bin/env bash
# Run a list of offline_eval jobs (development signals) with every outcome
# recorded and nothing overwritten.
#
# Usage, from the repository root, normally under remote_cpu.sh:
#   PYTHON=<absolute env interpreter> devtools/simlab/offline_batch.sh \
#       <attempt-id> <role> "<map> <layout|-> <seed> <agent>" ...
#
# Each job writes <out>/<job>.json, <job>.err and <job>.rc under
# _local/offline/<attempt-id>/, where the attempt directory must not exist
# yet (a rerun gets a new attempt id, so earlier evidence is preserved).
# The batch exits with the number of failed jobs (0 only if every job
# exited 0 and wrote a non-empty JSON file); STATUS.txt lists every job.
set -uo pipefail

: "${PYTHON:?set PYTHON to the absolute project interpreter}"
case "$PYTHON" in
    /*) ;;
    *) echo "PYTHON must be an absolute path, got $PYTHON" >&2; exit 2 ;;
esac
[ -x "$PYTHON" ] || { echo "PYTHON is not executable: $PYTHON" >&2; exit 2; }

attempt="$1"; role="$2"; shift 2
out="_local/offline/$attempt"
if [ -e "$out" ]; then
    echo "attempt directory exists, choose a new attempt id: $out" >&2
    exit 2
fi
mkdir -p "$out"
{ echo "attempt $attempt role $role"; echo "python $PYTHON"; "$PYTHON" --version 2>&1; \
  git rev-parse HEAD; git status --porcelain | wc -l; } > "$out/CONTEXT.txt"

fails=0
for spec in "$@"; do
    read -r map layout seed agent <<< "$spec"
    job="${role}_${map}_${layout}_s${seed}_${agent}"
    lay=()
    [ "$layout" != "-" ] && lay=(--layout "_local/layouts/$layout.json")
    "$PYTHON" -m devtools.simlab.offline_eval --role "$role" --map "$map" "${lay[@]}" \
        --seed "$seed" --agent "$agent" --carry-ratio-sigma 0.36 \
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
