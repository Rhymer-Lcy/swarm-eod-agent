"""Characterization tests for the supplied submission checker.

These tests pin down how ``swarm_rescue.submission_check`` (supplied with the
competition baseline, not modified by this project) actually behaves on small,
clearly synthetic fixture packages. They exist so that the project's packaging
workflow is built on the checker's real behaviour, and so that a future
baseline release that changes the checker is noticed. Every fixture uses the
placeholder identity team 006 / FIXTURE-TEAM; none is a real submission.
"""

from __future__ import annotations

import sys
import zipfile
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

submission_check = pytest.importorskip("swarm_rescue.submission_check")
validate_submission = submission_check.validate_submission
validate_team_info_yaml = submission_check.validate_team_info_yaml

EVAL = (
    "from swarm_rescue.solutions.my_drone_rescue_example import MyDroneRescueExample\n\n"
    "def drone_class_for_mode(mode):\n"
    "    return MyDroneRescueExample\n"
)
TEAM = "team_number: 6\nteam_name: FIXTURE-TEAM\nteam_members: fixture-a, fixture-b\n"


def make_zip(directory: Path, name: str, members: dict) -> Path:
    path = directory / name
    with zipfile.ZipFile(path, "w") as zf:
        for member, text in members.items():
            zf.writestr(member, text)
    return path


def base_members() -> dict:
    return {"my_drone_eval.py": EVAL, "team_info.yml": TEAM, "req.txt": ""}


def errors_of(path: Path):
    return validate_submission(path).errors


def test_minimal_flat_package_passes(tmp_path):
    result = validate_submission(make_zip(tmp_path, "team006_preliminary.zip", base_members()))
    assert result.ok, result.errors


def test_nested_solutions_folder_is_rejected(tmp_path):
    members = {f"solutions/{k}": v for k, v in base_members().items()}
    errs = errors_of(make_zip(tmp_path, "team006_preliminary.zip", members))
    assert any("must be at the zip root" in e for e in errs)


def test_pycache_entry_is_rejected(tmp_path):
    members = {**base_members(), "__pycache__/my_drone_eval.cpython-311.pyc": "x"}
    errs = errors_of(make_zip(tmp_path, "team006_preliminary.zip", members))
    assert any("__pycache__" in e for e in errs)


def test_zip_name_and_team_number_must_agree(tmp_path):
    errs = errors_of(make_zip(tmp_path, "team6_preliminary.zip", base_members()))
    assert any("teamNNN" in e for e in errs)
    errs = errors_of(make_zip(tmp_path, "team007_final.zip", base_members()))
    assert any("does not match" in e for e in errs)


def test_ground_truth_call_anywhere_is_rejected(tmp_path):
    helper = "def f(self):\n    return self.true_position()\n"
    members = {**base_members(), "helper.py": helper}
    errs = errors_of(make_zip(tmp_path, "team006_preliminary.zip", members))
    assert any("true_position" in e for e in errs)


def test_string_team_number_is_accepted_despite_the_documentation(tmp_path):
    # The participant guide says team_number must be an integer, not "006";
    # the checker converts with int() and accepts the quoted form.
    team = TEAM.replace("team_number: 6", 'team_number: "006"')
    members = {**base_members(), "team_info.yml": team}
    result = validate_submission(make_zip(tmp_path, "team006_preliminary.zip", members))
    assert result.ok, result.errors


def test_workspace_check_does_not_see_pycache_that_the_zip_check_rejects(tmp_path):
    # create_zip_submission.sh zips solutions/ wholesale ("zip -r ."), so a
    # __pycache__ left there by a local run passes the workspace check but
    # makes the resulting package fail the zip check.
    root = tmp_path / "workspace"
    solutions = root / "src" / "swarm_rescue" / "solutions"
    (solutions / "__pycache__").mkdir(parents=True)
    for name, text in base_members().items():
        target = root / name if name == "req.txt" else solutions / name
        target.write_text(text, encoding="utf-8")
    (solutions / "__pycache__" / "my_drone_eval.cpython-311.pyc").write_bytes(b"\0")
    assert validate_submission(root).ok
    members = {**base_members(), "__pycache__/my_drone_eval.cpython-311.pyc": "\0"}
    assert not validate_submission(make_zip(tmp_path, "team006_preliminary.zip", members)).ok


def test_placeholder_template_is_valid_for_local_use_only(tmp_path):
    template = REPO_ROOT / "devtools" / "templates" / "team_info.example.yml"
    info, err = validate_team_info_yaml(template)
    assert err is None and info["team_number"] == 0
    assert info["team_name"] == "PLACEHOLDER-TEAM"
