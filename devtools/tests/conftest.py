"""Shared fixtures for the devtools tests: throwaway Git repositories."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Callable, Dict

import pytest

DEVTOOLS = Path(__file__).resolve().parents[1]
if str(DEVTOOLS) not in sys.path:
    sys.path.insert(0, str(DEVTOOLS))

GIT_ENV = {
    "GIT_AUTHOR_NAME": "Test Author",
    "GIT_AUTHOR_EMAIL": "test@example.com",
    "GIT_COMMITTER_NAME": "Test Author",
    "GIT_COMMITTER_EMAIL": "test@example.com",
    "GIT_CONFIG_NOSYSTEM": "1",
}


def run_git(repo: Path, *args: str) -> str:
    env = {**os.environ, **GIT_ENV}
    return subprocess.run(
        ["git", "-C", str(repo), "-c", "core.autocrlf=false", *args],
        check=True, capture_output=True, env=env,
    ).stdout.decode("utf-8")


def write_files(repo: Path, files: Dict[str, str]) -> None:
    for rel, text in files.items():
        path = repo / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(text.encode("utf-8"))


@pytest.fixture
def make_repo(tmp_path: Path) -> Callable[..., Path]:
    """Create a repository with a tagged baseline commit and a project commit."""

    def _make(project_files: Dict[str, str] | None = None) -> Path:
        repo = tmp_path / "repo"
        repo.mkdir()
        run_git(repo, "init", "-q", "-b", "main")
        write_files(repo, {
            "README.md": "Supplied baseline.\n",
            # Built at run time so that this test source does not itself
            # contain a home-directory path for the scanner to report.
            "tools/legacy.py": 'IMG = "/' + 'home/upstream/project/map.png"\n',
            "src/swarm_rescue/solutions/team_info.yml":
                "team_number: 6\nteam_name: SkySaviors\nteam_members: tom, alice\n",
        })
        run_git(repo, "add", "-A")
        run_git(repo, "commit", "-q", "-m", "baseline")
        run_git(repo, "tag", "baseline-supplied-5.1.3")
        run_git(repo, "rm", "-q", "--cached", "src/swarm_rescue/solutions/team_info.yml")
        (repo / ".gitignore").write_text(
            "/_local/\n/_review_packages/\n/src/swarm_rescue/solutions/team_info.yml\n",
            encoding="utf-8")
        write_files(repo, {"docs/README.md": "Project documentation.\n",
                           **(project_files or {})})
        run_git(repo, "add", "-A")
        run_git(repo, "commit", "-q", "-m", "project")
        return repo

    return _make
