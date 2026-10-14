#!/usr/bin/env python3
"""Phase 4, Risk C: reproducibility of offline runs (development only).

    python -m devtools.simlab.repro_check repeat --out <dir> -- \
        -m devtools.simlab.offline_eval --role red --map Map05 --seed 1 --agent p2_540054b
    python -m devtools.simlab.repro_check observer --out <dir> --map Map05 --seed 1 \
        --agent p2_540054b

``repeat`` runs one command in fresh interpreter processes under a fixed set
of environment variations (Python hash seed 0, 1 and 12345; library thread
caps 1 and 2) and compares the JSON outputs after removing wall-clock fields;
it reports, per variation, whether the canonical output is identical to the
first run and which top-level keys differ.

``observer`` tests whether a measuring tool changes the run it measures:
``red_wall_diagnosis`` reads the live brains during the run (for example the
disposal-center box, whose ``bbox()`` refreshes a cache and resets a counter),
while ``offline_eval`` does not. The same red configuration is run through
both, and the placed bombs and submission step are compared.

Every run's stdout, stderr and exit status are kept under ``--out``; the
command exits non-zero when any run fails, so a failure cannot read as
"identical".
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

TIMING_KEYS = {"wall_s", "brain_ms_per_drone_step", "profile_cumulative", "harness_wall_s"}

VARIATIONS: List[Tuple[str, Dict[str, str]]] = [
    ("hash0_t1", {"PYTHONHASHSEED": "0", "OMP_NUM_THREADS": "1", "OPENCV_NUM_THREADS": "1"}),
    ("hash1_t1", {"PYTHONHASHSEED": "1", "OMP_NUM_THREADS": "1", "OPENCV_NUM_THREADS": "1"}),
    ("hash12345_t1", {"PYTHONHASHSEED": "12345", "OMP_NUM_THREADS": "1",
                      "OPENCV_NUM_THREADS": "1"}),
    ("hash0_t2", {"PYTHONHASHSEED": "0", "OMP_NUM_THREADS": "2", "OPENCV_NUM_THREADS": "2",
                  "OPENBLAS_NUM_THREADS": "2", "MKL_NUM_THREADS": "2"}),
]


def canonical(doc: Any) -> Any:
    if isinstance(doc, dict):
        return {k: canonical(v) for k, v in sorted(doc.items()) if k not in TIMING_KEYS}
    if isinstance(doc, list):
        return [canonical(v) for v in doc]
    return doc


def digest(doc: Any) -> str:
    return hashlib.sha256(json.dumps(canonical(doc), sort_keys=True).encode()).hexdigest()


def run_once(argv: List[str], env_over: Dict[str, str], out: Path, tag: str) -> Dict[str, Any]:
    env = {**os.environ, **env_over}
    proc = subprocess.run([sys.executable, *argv], capture_output=True, text=True, env=env)
    (out / f"{tag}.stdout").write_text(proc.stdout, encoding="utf-8")
    (out / f"{tag}.stderr").write_text(proc.stderr, encoding="utf-8")
    (out / f"{tag}.rc").write_text(f"{proc.returncode}\n", encoding="utf-8")
    doc = None
    if proc.returncode == 0:
        try:
            doc = json.loads(proc.stdout)
        except json.JSONDecodeError:
            doc = None
    return {"tag": tag, "rc": proc.returncode, "doc": doc}


def repeat(argv: List[str], out: Path) -> int:
    results = [run_once(argv, env, out, tag) for tag, env in VARIATIONS]
    failed = [r["tag"] for r in results if r["doc"] is None]
    ref = results[0]["doc"]
    report = {"command": argv, "failed": failed, "runs": []}
    for r in results:
        entry = {"tag": r["tag"], "rc": r["rc"]}
        if r["doc"] is not None:
            entry["digest"] = digest(r["doc"])
            if ref is not None:
                a, b = canonical(ref), canonical(r["doc"])
                entry["identical_to_first"] = a == b
                entry["differing_keys"] = sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k)) \
                    if isinstance(a, dict) and isinstance(b, dict) else None
        report["runs"].append(entry)
    (out / "REPORT.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    print(json.dumps(report))
    return 1 if failed else 0


def observer(args: argparse.Namespace, out: Path) -> int:
    common = ["--map", args.map, "--seed", str(args.seed), "--agent", args.agent]
    diag = run_once(["-m", "devtools.simlab.red_wall_diagnosis", *common], VARIATIONS[0][1],
                    out, "diagnosis")
    plain = run_once(["-m", "devtools.simlab.offline_eval", "--role", "red", "--drones", "5",
                      "--carry-ratio-sigma", "0.36", *common], VARIATIONS[0][1], out, "offline_eval")
    report: Dict[str, Any] = {"map": args.map, "seed": args.seed, "agent": args.agent,
                              "rc": {"diagnosis": diag["rc"], "offline_eval": plain["rc"]}}
    if diag["doc"] is None or plain["doc"] is None:
        report["result"] = "FAILED"
        (out / "REPORT.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
        print(json.dumps(report))
        return 1
    d, p = diag["doc"], plain["doc"]
    bombs_d = [tuple(b) for b in d.get("bombs", [])]
    bombs_p = [tuple(round(v, 1) for v in b) for b in p.get("bombs", [])]
    report.update({
        "diagnosis_wall": d.get("actual"), "offline_eval_wall": (p.get("scores") or {}).get("wall"),
        "diagnosis_submit_step": d.get("submit_step"), "offline_eval_submit_step": p.get("submit_step"),
        "bombs_identical": bombs_d == bombs_p,
        "diagnosis_bombs": bombs_d, "offline_eval_bombs": bombs_p,
    })
    report["result"] = "IDENTICAL" if report["bombs_identical"] and \
        report["diagnosis_submit_step"] == report["offline_eval_submit_step"] else "DIFFERENT"
    (out / "REPORT.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    print(json.dumps(report))
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="mode", required=True)
    rep = sub.add_parser("repeat")
    rep.add_argument("--out", required=True)
    rep.add_argument("command", nargs=argparse.REMAINDER)
    obs = sub.add_parser("observer")
    obs.add_argument("--out", required=True)
    obs.add_argument("--map", required=True)
    obs.add_argument("--seed", type=int, default=1)
    obs.add_argument("--agent", default="current")
    args = parser.parse_args(argv)
    out = Path(args.out)
    if out.exists():
        print(f"output directory exists, choose a new one: {out}", file=sys.stderr)
        return 2
    out.mkdir(parents=True)
    if args.mode == "repeat":
        command = [c for c in args.command if c != "--"]
        if not command:
            print("no command given", file=sys.stderr)
            return 2
        return repeat(command, out)
    return observer(args, out)


if __name__ == "__main__":
    sys.exit(main())
