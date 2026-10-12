"""Tests for devtools/make_review_package.py."""

from __future__ import annotations

import hashlib
import json
import zipfile
from pathlib import Path

import pytest

import make_review_package as mrp
from conftest import write_files

PRIVATE_IP = ".".join(["10", "20", "30", "40"])


def build(repo: Path, tmp_path: Path, *extra: str) -> Path:
    scope = tmp_path / "scope.md"
    scope.write_text("# Scope\n\nReview the test project.\n", encoding="utf-8")
    out = tmp_path / "packages"
    mrp.main(["--repo", str(repo), "--phase", "test", "--scope-file", str(scope),
              "--out-dir", str(out), *extra])
    zips = sorted(out.glob("*.zip"))
    assert len(zips) == 1
    return zips[0]


def test_package_layout_checksums_and_sidecar(make_repo, tmp_path):
    repo = make_repo({"devtools/tool.py": "print('tool')\n"})
    evidence = tmp_path / "evidence"
    write_files(evidence, {"tests/summary.md": "2 passed\n"})
    package = build(repo, tmp_path, "--evidence-dir", str(evidence))
    assert package.name.startswith("swarm-eod-agent_test_") and package.name.endswith(".zip")
    with zipfile.ZipFile(package) as zf:
        names = set(zf.namelist())
        assert {"README.md", "MANIFEST.json", "scope.md", "SHA256SUMS",
                "git/project-changes.patch", "git/log.txt", "git/status.txt",
                "baseline/integrity.json", "hygiene/head.json", "hygiene/history.json",
                "source/devtools/tool.py", "source/docs/README.md", "source/.gitignore",
                "evidence/tests/summary.md"} <= names
        # The supplied baseline is not duplicated.
        assert "source/README.md" not in names and "source/tools/legacy.py" not in names
        manifest = json.loads(zf.read("MANIFEST.json"))
        assert manifest["working_tree_dirty"] is False
        assert manifest["head_commit"] in package.name or manifest["head_commit"][:7] in package.name
        integrity = json.loads(zf.read("baseline/integrity.json"))
        assert integrity["counts"] == {"unchanged": 2, "modified": 0, "removed_from_tracking": 1}
        sums = dict(reversed(line.split("  ", 1))
                    for line in zf.read("SHA256SUMS").decode().splitlines())
        assert set(sums) == names - {"SHA256SUMS"}
        for rel, digest in sums.items():
            assert hashlib.sha256(zf.read(rel)).hexdigest() == digest
    sidecar = package.parent / (package.name + ".sha256")
    assert sidecar.read_text().split()[0] == hashlib.sha256(package.read_bytes()).hexdigest()


def test_dirty_tree_is_refused_unless_allowed(make_repo, tmp_path):
    repo = make_repo()
    (repo / "docs" / "README.md").write_text("edited\n", encoding="utf-8")
    with pytest.raises(SystemExit, match="dirty"):
        build(repo, tmp_path)
    package = build(repo, tmp_path, "--allow-dirty")
    assert package.stem.endswith("-dirty")


def test_evidence_with_private_content_blocks_the_build(make_repo, tmp_path):
    repo = make_repo()
    evidence = tmp_path / "evidence"
    write_files(evidence, {"server.log": f"ssh to {PRIVATE_IP} ok\n"})
    with pytest.raises(SystemExit, match="hygiene gate"):
        build(repo, tmp_path, "--evidence-dir", str(evidence))
    assert not list((tmp_path / "packages").glob("*.zip"))


def test_non_text_evidence_is_refused(make_repo, tmp_path):
    repo = make_repo()
    evidence = tmp_path / "evidence"
    evidence.mkdir()
    (evidence / "capture.png").write_bytes(b"\x89PNG\r\n")
    with pytest.raises(SystemExit, match="non-text"):
        build(repo, tmp_path, "--evidence-dir", str(evidence))


def test_repository_errors_block_the_build(make_repo, tmp_path):
    repo = make_repo({"docs/hosts.md": f"server {PRIVATE_IP}\n"})
    with pytest.raises(SystemExit, match="hygiene scan"):
        build(repo, tmp_path)
