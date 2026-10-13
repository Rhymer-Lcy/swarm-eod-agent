#!/usr/bin/env python3
"""Summarize a run ledger into a Markdown table and a JSON digest.

    python -m devtools.simlab.summarize _local/runs/server/ledger.jsonl --phase phase1

Every started round is listed; a started round without a finished record is
reported as ``interrupted``. Scores are copied from the finished records,
which hold the official score managers' outputs (red scores are partial: the
bomb-difficulty term is pending locally).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List


def load(ledger: Path, phase: str) -> List[Dict[str, Any]]:
    started: Dict[str, Dict[str, Any]] = {}
    finished: Dict[str, Dict[str, Any]] = {}
    for line in ledger.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        entry = json.loads(line)
        if entry.get("phase") != phase:
            continue
        if entry.get("event") == "started":
            started[entry["run_id"]] = entry
        elif entry.get("event") == "finished":
            finished[entry["run_id"]] = entry
    rows = []
    for run_id, start in sorted(started.items()):
        rec = finished.get(run_id)
        row = {"run_id": run_id, "index": start.get("round_index_in_phase"),
               "label": start["label"], "map": start["map"], "mode": start["mode"],
               "agent": start["agent"], "seed": start["seed"],
               "competition": start["competition_mode"], "commit": start["git_head"][:7],
               "dirty": start["git_dirty"], "bombs_file": Path(start["bombs_file"]).name
               if start.get("bombs_file") else None}
        if rec is None:
            row["status"] = "interrupted"
        else:
            row.update({"status": rec.get("status"), "termination": rec.get("termination"),
                        "steps": rec.get("elapsed_timestep"),
                        "walltime_s": rec.get("elapsed_walltime_s"),
                        "walltime_limit_s": rec.get("max_walltime_limit_s"),
                        "steps_per_s": rec.get("steps_per_second"),
                        "score": rec.get("round_score"),
                        "components": rec.get("score_components"),
                        "bombs": (rec.get("bombs_disposed"), rec.get("bombs_total"))
                        if start["mode"] == "rescue" else
                        (rec.get("bombs_placed"), rec.get("inventory_remaining"))})
        rows.append(row)
    return rows


def markdown(rows: List[Dict[str, Any]]) -> str:
    head = ("| # | label | map | mode | agent | seed | steps | wall s | score | components | "
            "bombs | termination |\n|---|---|---|---|---|---|---|---|---|---|---|---|\n")
    out = []
    for r in rows:
        comp = r.get("components") or {}
        comp_txt = ", ".join(f"{k} {v:.1f}" for k, v in comp.items() if v is not None)
        agent = "official" if r["agent"] == "official" else r["agent"].split(":")[-1]
        bombs = r.get("bombs")
        if r["mode"] == "rescue" and bombs:
            bombs_txt = f"{bombs[0]}/{bombs[1]} disposed"
        elif bombs:
            bombs_txt = f"{bombs[0]} placed"
        else:
            bombs_txt = ""
        score = r.get("score")
        out.append(f"| {r['index']} | {r['label']} | {r['map']} | {r['mode']} | {agent} | "
                   f"{r['seed']} | {r.get('steps', '')} | {r.get('walltime_s', '')} | "
                   f"{'' if score is None else f'{score:.2f}'} | {comp_txt} | {bombs_txt} | "
                   f"{r.get('termination', r['status'])} |")
    return head + "\n".join(out) + "\n"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("ledger")
    parser.add_argument("--phase", default="phase1")
    parser.add_argument("--json-out", default=None)
    args = parser.parse_args(argv)
    rows = load(Path(args.ledger), args.phase)
    print(f"rounds started: {len(rows)}; interrupted: "
          f"{sum(r['status'] == 'interrupted' for r in rows)}; "
          f"dirty commits: {sum(bool(r['dirty']) for r in rows)}\n")
    print(markdown(rows))
    if args.json_out:
        Path(args.json_out).write_text(json.dumps(rows, indent=1) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
