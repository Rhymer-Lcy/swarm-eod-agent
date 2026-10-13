"""The frozen comparator agents must stay byte-identical to their source commit."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

COMPARATORS = REPO / "devtools" / "comparators"
MANIFESTS = sorted(COMPARATORS.glob("*.json"))


def test_at_least_one_comparator_is_registered():
    assert [m.stem for m in MANIFESTS] == ["p1_ddc6897"]


@pytest.mark.parametrize("manifest", MANIFESTS, ids=lambda p: p.stem)
def test_comparator_files_match_manifest(manifest: Path):
    data = json.loads(manifest.read_text(encoding="utf-8"))
    package = COMPARATORS / manifest.stem
    present = sorted(p.name for p in package.glob("*.py"))
    assert present == sorted(data["files_sha256"])
    assert len(present) == 14
    for name, digest in data["files_sha256"].items():
        assert hashlib.sha256((package / name).read_bytes()).hexdigest() == digest, name


@pytest.mark.parametrize("manifest", MANIFESTS, ids=lambda p: p.stem)
def test_comparator_files_match_git_history(manifest: Path):
    data = json.loads(manifest.read_text(encoding="utf-8"))
    commit = data["source_commit"]
    probe = subprocess.run(["git", "-C", str(REPO), "cat-file", "-e", f"{commit}^{{commit}}"],
                           capture_output=True)
    if probe.returncode != 0:
        pytest.skip(f"source commit {commit[:7]} not in this clone")
    package = COMPARATORS / manifest.stem
    for name in data["files_sha256"]:
        blob = subprocess.run(["git", "-C", str(REPO), "show",
                               f"{commit}:{data['source_dir']}/{name}"],
                              check=True, capture_output=True).stdout
        assert blob == (package / name).read_bytes(), name


def test_comparator_is_importable_and_isolated():
    from devtools.comparators.p1_ddc6897 import estimator as frozen_estimator
    from devtools.comparators.p1_ddc6897.shell import EodPlaceDrone, EodRescueDrone
    from swarm_rescue.solutions.eod import estimator as current_estimator
    assert EodRescueDrone.__module__ == "devtools.comparators.p1_ddc6897.shell"
    assert EodPlaceDrone.PLACE_MODE is True
    assert frozen_estimator.PoseEstimator is not current_estimator.PoseEstimator
    # Agent code must never import the comparators.
    solutions = REPO / "src" / "swarm_rescue" / "solutions"
    for path in solutions.rglob("*.py"):
        assert "comparators" not in path.read_text(encoding="utf-8"), path
