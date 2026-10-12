"""Tests for devtools/check_repo_hygiene.py, including planted-error mutations.

Secret-shaped and address-shaped strings are assembled at run time so that this
file does not itself contain anything the scanner would report.
"""

from __future__ import annotations

import json
from pathlib import Path

import check_repo_hygiene as hygiene
from conftest import run_git, write_files

FAKE_TOKEN = "gh" + "p_" + "A1b2C3d4" * 5
PRIVATE_IP = ".".join(["192", "168", "7", "42"])
OWNER_SECRET = "Zeta" + "Falcon" + "Squad"


def scan(repo: Path, *extra: str, tmp_json: Path | None = None):
    args = ["--repo", str(repo), "--quiet", *extra]
    if tmp_json is not None:
        args += ["--json", str(tmp_json)]
    return hygiene.run(args)


def findings(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))["findings"]


def test_clean_repository_passes_head_and_history(make_repo, tmp_path):
    repo = make_repo()
    out = tmp_path / "head.json"
    assert scan(repo, tmp_json=out) == 0
    report = json.loads(out.read_text(encoding="utf-8"))
    assert report["files_scanned"] == 4  # README, tools/legacy.py, .gitignore, docs/README.md
    assert report["baseline_tag_found"] is True
    # The supplied baseline's home path is informational, not an error.
    assert [f["rule"] for f in report["findings"]] == ["infra:unix-home-path"]
    assert report["findings"][0]["origin"] == "baseline"
    hist = tmp_path / "hist.json"
    assert scan(repo, "--history", tmp_json=hist) == 0
    rules = sorted(f["rule"] for f in findings(hist))
    # The demo team_info.yml in the baseline commit is reported as INFO only.
    assert rules == ["forbidden-path", "infra:unix-home-path"]
    assert all(f["severity"] == "INFO" for f in findings(hist))


def test_planted_token_is_an_error_and_is_masked(make_repo, tmp_path):
    repo = make_repo({"src/agent.py": f'TOKEN = "{FAKE_TOKEN}"\n'})
    out = tmp_path / "r.json"
    assert scan(repo, tmp_json=out) == 1
    hits = [f for f in findings(out) if f["rule"] == "secret:github-token"]
    assert len(hits) == 1 and hits[0]["path"] == "src/agent.py" and hits[0]["line"] == 1
    assert FAKE_TOKEN not in out.read_text(encoding="utf-8")


def test_planted_private_address_in_project_file_is_an_error(make_repo, tmp_path):
    repo = make_repo({"docs/server.md": f"Run on {PRIVATE_IP} tonight.\n"})
    out = tmp_path / "r.json"
    assert scan(repo, tmp_json=out) == 1
    hits = [f for f in findings(out) if f["rule"] == "infra:private-ipv4"]
    assert [(h["severity"], h["path"]) for h in hits] == [("ERROR", "docs/server.md")]


def test_owner_private_pattern_is_loaded_from_local_file(make_repo, tmp_path):
    repo = make_repo({"docs/team.md": f"Team {OWNER_SECRET} registered.\n"})
    assert scan(repo) == 0  # no pattern file yet: the generic rules do not know the name
    write_files(repo, {"_local/config/private-patterns.txt": "# comment\n" + OWNER_SECRET + "\n"})
    out = tmp_path / "r.json"
    assert scan(repo, tmp_json=out) == 1
    text = out.read_text(encoding="utf-8")
    assert '"private-pattern#1"' in text and OWNER_SECRET not in text


def test_forbidden_path_and_changed_team_identity(make_repo, tmp_path):
    repo = make_repo()
    write_files(repo, {"notes/rules.pdf": "%PDF-1.4 fake\n"})
    run_git(repo, "add", "-f", "notes/rules.pdf")
    out = tmp_path / "idx.json"
    assert scan(repo, "--index", tmp_json=out) == 1
    assert [f["path"] for f in findings(out) if f["rule"] == "forbidden-path"] == ["notes/rules.pdf"]
    run_git(repo, "rm", "-q", "--cached", "notes/rules.pdf")
    write_files(repo, {"src/swarm_rescue/solutions/team_info.yml":
                       "team_number: 41\nteam_name: Real\nteam_members: x\n"})
    run_git(repo, "add", "-f", "src/swarm_rescue/solutions/team_info.yml")
    assert scan(repo, "--index", tmp_json=out) == 1
    hits = [f for f in findings(out) if f["rule"] == "forbidden-path"]
    assert [(h["severity"], h["origin"]) for h in hits] == [("ERROR", "project")]


def test_directory_mode_and_empty_input(tmp_path):
    empty = tmp_path / "empty"
    empty.mkdir()
    assert hygiene.run(["--directory", str(empty), "--quiet"]) == 1  # zero files is a failure
    staged = tmp_path / "stage"
    write_files(staged, {"evidence/log.txt": f"connected to {PRIVATE_IP}\n"})
    out = tmp_path / "d.json"
    assert hygiene.run(["--directory", str(staged), "--quiet", "--json", str(out)]) == 1
    assert findings(out)[0]["origin"] == "directory"


def test_allowed_and_disallowed_email_addresses(make_repo, tmp_path):
    allowed = "123+someone@users.noreply.github.com"
    other = "someone" + "@" + "corp-mail.org"
    repo = make_repo({"docs/contact.md": f"{allowed}\n{other}\n"})
    out = tmp_path / "r.json"
    assert scan(repo, tmp_json=out) == 0  # e-mail addresses are warnings, not errors
    hits = [f for f in findings(out) if f["rule"] == "email-address"]
    assert [(h["severity"], h["line"]) for h in hits] == [("WARN", 2)]
