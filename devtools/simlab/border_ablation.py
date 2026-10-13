#!/usr/bin/env python3
"""Border-rendering ablation on the evidence of real red rounds (diagnostic).

    python -m devtools.simlab.border_ablation _local/runs/<run_id> [...]

For each instrumented red round that saved ``submission_evidence.npz``, the
submitting drone's merged wall evidence is rendered with the current agent
renderer twice, with border bands on and off, and both maps are scored with
the official scorer against the walls-only truth of the round's map. The
round's own submitted map is scored as a consistency check (it must equal
the official map score of the round when the renderer did not change). The
result is a mapping ablation; it must not be used to tune rendering against
the supplied maps.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from .fixtures import load_fixture


def ablate(run_dir: Path) -> dict:
    from swarm_rescue.solutions.eod.wallmap import render
    from swarm_rescue.tools.map_exploration_scoring import scorer_for_walltime

    record = json.loads((run_dir / "record.json").read_text(encoding="utf-8"))
    data = np.load(run_dir / "submission_evidence.npz")
    fx = load_fixture(record["map"])
    truth = fx["walls"]
    scorer = scorer_for_walltime(float(fx["max_walltime_limit"]))
    box = tuple(float(v) for v in data["dc_box"]) if data["dc_box"].size == 4 else None
    out = {"label": record["label"], "map": record["map"],
           "official_map_score": round(float(record["score_components"]["map"]), 3)}
    submitted = data["submitted_map"]
    if submitted.size:
        out["submitted_rescored"] = round(100 * scorer.score(truth, submitted, 0.0).s_map, 3)
    for name, border in (("border_on", True), ("border_off", False)):
        grid = render(data["evidence"], dc_box=box, border=border)
        b = scorer.score(truth, grid, 0.0)
        out[name] = {"map": round(100 * b.s_map, 3), "credit": round(b.credit, 4),
                     "penalty": round(b.penalty, 4), "charged_pixels": int(b.charged_pixels),
                     "predicted_pixels": int(b.predicted_pixels)}
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", type=Path)
    args = parser.parse_args(argv)
    results, bad = [], 0
    for run in args.runs:
        if not (run / "submission_evidence.npz").exists():
            print(f"no saved evidence in {run}", file=sys.stderr)
            bad += 1
            continue
        results.append(ablate(run))
    print(json.dumps(results, indent=1))
    return 1 if bad or not results else 0


if __name__ == "__main__":
    sys.exit(main())
