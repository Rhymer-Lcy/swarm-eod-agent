#!/usr/bin/env python3
"""Tabulate offline_eval JSON outputs (development signals only).

    python -m devtools.simlab.offline_summary _local/offline/*.json

One Markdown row per file: agent, outcome counters, proxy score or partial
red score, heading error and brain compute time. Exits 1 if a file is empty
or unreadable, so a failed run cannot pass as a missing row.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def row(path: Path) -> str:
    data = json.loads(path.read_text(encoding="utf-8"))
    head = data.get("heading_error_deg", {})
    timing = data.get("brain_ms_per_drone_step", {})
    if "proxy_score" in data:
        outcome = f"{data['disposed']}/{data['bombs']} at {data['steps']}"
        score = data["proxy_score"]
    else:
        scores = data.get("scores", {})
        outcome = f"map {scores.get('map')} wall {scores.get('wall')} submit {data.get('submit_step')}"
        score = scores.get("partial")
    return (f"| {path.stem} | {data.get('agent')} | {outcome} | {score} | "
            f"{data.get('health_lost')} | {head.get('all_p95', '')} | {head.get('carry_p95', '')} | "
            f"{timing.get('mean', '')} |")


def main(argv=None) -> int:
    paths = [Path(p) for p in (argv if argv is not None else sys.argv[1:])]
    if not paths:
        print("no input files", file=sys.stderr)
        return 1
    print("| run | agent | outcome | score | health lost | heading p95 | carry p95 | brain ms |")
    print("|---|---|---|---|---|---|---|---|")
    bad = 0
    for path in sorted(paths):
        try:
            print(row(path))
        except (OSError, ValueError, KeyError) as exc:
            print(f"| {path.stem} | UNREADABLE: {exc!r} |")
            bad += 1
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
