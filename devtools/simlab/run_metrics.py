#!/usr/bin/env python3
"""Per-round metrics from runner outputs (record.json and, if instrumented,
truth.npz).

    python -m devtools.simlab.run_metrics _local/runs/<run_id> [...] [--json out.json]

For every run: official score and components, termination, steps per second,
controller time per drone-step (instrumented rounds), transport counters
from the truth log, and the heading error of the agent's own estimate.

Alignment: a row logged at step s holds the truth at that step and the
estimate written by the previous ``control()``, which describes the state
one physics step earlier; the estimate of row s is therefore compared with
the truth of row s - 1 of the same drone. "Carrying" is the truth grasp flag
of the compared truth row, independent of the agent's own state label.
Exits 1 if any run directory lacks a finished record.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

import numpy as np


def heading_errors(truth_path: Path) -> Dict[str, Any]:
    data = np.load(truth_path, allow_pickle=False)
    drones, brain = data["drones"], data["brain"]
    fields = [str(f) for f in data["drone_fields"]]
    di = {name: i for i, name in enumerate(fields)}
    codes = json.loads(str(data["state_codes"]))
    names = {v: k for k, v in codes.items()}
    errors, carry, labels = [], [], []
    for ident in np.unique(drones[:, di["drone"]]):
        tr = drones[drones[:, di["drone"]] == ident]
        br = brain[brain[:, 1] == ident]
        truth_by_step = {int(r[di["step"]]): r for r in tr}
        for row in br:
            prev = truth_by_step.get(int(row[0]) - 1)
            if prev is None:
                continue
            err = np.arctan2(np.sin(row[4] - prev[di["angle"]]), np.cos(row[4] - prev[di["angle"]]))
            errors.append(abs(np.degrees(err)))
            carry.append(prev[di["grasped"]] > 0)
            labels.append(names.get(int(row[7]), "?"))
    errors = np.asarray(errors)
    carry = np.asarray(carry, dtype=bool)
    labels = np.asarray(labels)
    out: Dict[str, Any] = {"n": int(errors.size)}
    if errors.size == 0:
        return out

    def stats(values: np.ndarray) -> Dict[str, Any]:
        if values.size == 0:
            return {"n": 0}
        return {"n": int(values.size), "p50": round(float(np.median(values)), 2),
                "p95": round(float(np.percentile(values, 95)), 2),
                "share_above_20": round(float(np.mean(values > 20.0)), 3)}

    out["all"] = stats(errors)
    out["carrying"] = stats(errors[carry])
    out["free"] = stats(errors[~carry])
    out["by_state"] = {lab: stats(errors[labels == lab]) for lab in sorted(set(labels.tolist()))}
    return out


def metrics(run_dir: Path) -> Dict[str, Any]:
    record = json.loads((run_dir / "record.json").read_text(encoding="utf-8"))
    if record.get("event") != "finished":
        raise ValueError(f"{run_dir.name}: no finished record")
    out: Dict[str, Any] = {k: record.get(k) for k in (
        "run_id", "label", "agent", "run_mode", "map", "mode", "seed", "git_head", "git_dirty",
        "status", "termination", "elapsed_timestep", "round_score", "score_components",
        "bombs_disposed", "bombs_total", "steps_per_second", "elapsed_walltime_s",
        "walltime_limit_reached", "crashed", "bombs_placed", "placement_completion")}
    diag = record.get("diagnostics") or {}
    out["controller"] = diag.get("controller")
    out["team"] = diag.get("team")
    reports = run_dir / "agent_reports.json"
    if reports.exists():
        rep = json.loads(reports.read_text(encoding="utf-8"))
        out["brain_errors"] = int(sum(int(v.get("errors", 0)) for v in rep.values()))
    truth = run_dir / "truth.npz"
    if truth.exists():
        out["heading_error_deg"] = heading_errors(truth)
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--json", type=Path, default=None)
    args = parser.parse_args(argv)
    results, bad = [], 0
    for run in args.runs:
        try:
            results.append(metrics(run))
        except (OSError, ValueError, KeyError) as exc:
            print(f"ERROR {run}: {exc!r}", file=sys.stderr)
            bad += 1
    text = json.dumps(results, indent=1, default=str)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
