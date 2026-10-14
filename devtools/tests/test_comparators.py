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


# Number of agent modules in each frozen package (the Phase 2 package added
# coverage.py). Typed here, not read from the manifest, so that a manifest
# and package that lose a file together still fail.
EXPECTED_FILES = {"p1_ddc6897": 14, "p2_540054b": 15, "p3_067818a": 16}


def test_registered_comparators():
    assert [m.stem for m in MANIFESTS] == sorted(EXPECTED_FILES)


@pytest.mark.parametrize("manifest", MANIFESTS, ids=lambda p: p.stem)
def test_comparator_files_match_manifest(manifest: Path):
    data = json.loads(manifest.read_text(encoding="utf-8"))
    package = COMPARATORS / manifest.stem
    present = sorted(p.name for p in package.glob("*.py"))
    assert present == sorted(data["files_sha256"])
    assert len(present) == EXPECTED_FILES[manifest.stem]
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


def test_submitted_agent_is_still_the_frozen_b2_r2():
    """Phase 4: the agent package under solutions/ is B2/R2 unchanged. A
    successor must be adopted explicitly (new identifier, this test updated
    in the same change), never introduced by an unnoticed edit."""
    data = json.loads((COMPARATORS / "p3_067818a.json").read_text(encoding="utf-8"))
    current = REPO / "src" / "swarm_rescue" / "solutions" / "eod"
    assert sorted(p.name for p in current.glob("*.py")) == sorted(data["files_sha256"])
    for name, digest in data["files_sha256"].items():
        assert hashlib.sha256((current / name).read_bytes()).hexdigest() == digest, name


def test_comparator_is_importable_and_isolated():
    from devtools.comparators.p1_ddc6897 import estimator as frozen_estimator
    from devtools.comparators.p1_ddc6897.shell import EodPlaceDrone, EodRescueDrone
    from swarm_rescue.solutions.eod import estimator as current_estimator
    assert EodRescueDrone.__module__ == "devtools.comparators.p1_ddc6897.shell"
    assert EodPlaceDrone.PLACE_MODE is True
    assert frozen_estimator.PoseEstimator is not current_estimator.PoseEstimator
    from devtools.comparators.p2_540054b import blue as frozen_blue
    from devtools.comparators.p2_540054b import grid as frozen_grid
    from devtools.comparators.p2_540054b.shell import EodRescueDrone as B1Drone
    from swarm_rescue.solutions.eod import blue as current_blue
    from swarm_rescue.solutions.eod import grid as current_grid
    assert B1Drone.__module__ == "devtools.comparators.p2_540054b.shell"
    # The frozen B1/R1 package resolves its own modules, never the current ones.
    assert frozen_blue.RescueBrain is not current_blue.RescueBrain
    assert frozen_blue.BrainBase.__module__ == "devtools.comparators.p2_540054b.base"
    assert frozen_grid.Planner is not current_grid.Planner
    # The frozen B2/R2 package (Phase 4) resolves its own modules too.
    from devtools.comparators.p3_067818a import blue as b2_blue
    from devtools.comparators.p3_067818a import delivery as b2_delivery
    from devtools.comparators.p3_067818a import red as r2_red
    from devtools.comparators.p3_067818a.shell import EodPlaceDrone as R2Drone
    from swarm_rescue.solutions.eod import delivery as current_delivery
    from swarm_rescue.solutions.eod import red as current_red
    assert R2Drone.__module__ == "devtools.comparators.p3_067818a.shell"
    assert b2_blue.RescueBrain is not current_blue.RescueBrain
    assert b2_blue.ApproachSet is b2_delivery.ApproachSet
    assert b2_delivery.ApproachSet is not current_delivery.ApproachSet
    assert r2_red.PlaceBrain is not current_red.PlaceBrain
    assert r2_red.PlaceBrain.__mro__[1].__module__ == "devtools.comparators.p3_067818a.base"
    # Agent code must never import the comparators.
    solutions = REPO / "src" / "swarm_rescue" / "solutions"
    for path in solutions.rglob("*.py"):
        assert "comparators" not in path.read_text(encoding="utf-8"), path
