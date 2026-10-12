#!/usr/bin/env python3
"""Build a focused, verifiable review package for independent assessment.

The package is a ZIP written to ``_review_packages/`` (ignored by Git) for
manual upload to an external reviewer. It contains:

* ``README.md``        how to read and verify the package;
* ``MANIFEST.json``    phase, scope, timestamps, commit, tree and status data;
* ``scope.md``         the review scope (copied from ``--scope-file``);
* ``git/``             status, refs, log and the project diff against the
                       supplied baseline (``baseline-supplied-5.1.3`` by default);
* ``source/``          every file added or modified since the baseline, as
                       committed at ``HEAD`` (the supplied baseline itself is not
                       duplicated; ``baseline/integrity.json`` accounts for it);
* ``baseline/``        per-file integrity of the supplied baseline at ``HEAD``,
                       plus any baseline files named with ``--baseline-file``
                       (for example the scoring code a finding cites);
* ``hygiene/``         repository hygiene reports for ``HEAD`` and the history;
* ``evidence/``        sanitized validation reports copied from ``--evidence-dir``;
* ``SHA256SUMS``       SHA-256 of every other file in the package.

Safety gates: the generator refuses to run on a dirty working tree unless
``--allow-dirty`` is given (the state is then recorded), refuses to overwrite an
existing package, copies only text evidence of known types, and scans the staged
package with ``check_repo_hygiene`` (including the owner's private patterns)
before writing; any error aborts the build. After writing, the ZIP is reopened
and every member is checked against ``SHA256SUMS``.

Standard library only; Python 3.8 or newer.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import tempfile
import zipfile
from dataclasses import asdict
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_repo_hygiene as hygiene  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
PROJECT = "swarm-eod-agent"
GENERATOR_VERSION = "1"
DEFAULT_BASE = hygiene.BASELINE_TAG
EVIDENCE_SUFFIXES = {".md", ".txt", ".json", ".log", ".csv", ".yml", ".yaml", ".patch"}
EVIDENCE_MAX_BYTES = 2 * 1024 * 1024
TZ8 = timezone(timedelta(hours=8))


def git(args: Sequence[str], repo: Path) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args], check=True, capture_output=True
    ).stdout.decode("utf-8")


def git_bytes(args: Sequence[str], repo: Path) -> bytes:
    return subprocess.run(
        ["git", "-C", str(repo), *args], check=True, capture_output=True
    ).stdout


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def repo_state(repo: Path) -> Dict[str, object]:
    head = git(["rev-parse", "HEAD"], repo).strip()
    status = git(["status", "--porcelain=v1", "--branch", "--untracked-files=all"], repo)
    dirty_lines = [ln for ln in status.splitlines() if not ln.startswith("##")]
    commit_epoch = int(git(["show", "-s", "--format=%ct", "HEAD"], repo).strip())
    return {
        "head": head,
        "short": head[:7],
        "branch": git(["rev-parse", "--abbrev-ref", "HEAD"], repo).strip(),
        "tree": git(["rev-parse", "HEAD^{tree}"], repo).strip(),
        "commit_time_utc": datetime.fromtimestamp(commit_epoch, timezone.utc).isoformat(),
        "commit_epoch": commit_epoch,
        "status_porcelain": status,
        "dirty": bool(dirty_lines),
        "dirty_entries": dirty_lines,
    }


def baseline_integrity(repo: Path, base: str) -> Dict[str, object]:
    base_entries = dict(hygiene._tree_entries(base, repo))
    head_entries = dict(hygiene._tree_entries("HEAD", repo))
    files = []
    counts = {"unchanged": 0, "modified": 0, "removed_from_tracking": 0}
    for path in sorted(base_entries):
        if path not in head_entries:
            state = "removed_from_tracking"
        elif head_entries[path] == base_entries[path]:
            state = "unchanged"
        else:
            state = "modified"
        counts[state] += 1
        files.append({"path": path, "state": state, "baseline_blob": base_entries[path],
                      "head_blob": head_entries.get(path)})
    return {"base": base, "base_commit": git(["rev-parse", base + "^{commit}"], repo).strip(),
            "baseline_file_count": len(base_entries), "counts": counts, "files": files}


def project_files(repo: Path, base: str) -> List[Tuple[str, str]]:
    out = git(["diff", "--name-status", "--no-renames", base, "HEAD"], repo)
    changed = []
    for line in out.splitlines():
        status, path = line.split("\t", 1)
        changed.append((status, path))
    return changed


def stage_package(
    stage: Path,
    repo: Path,
    *,
    phase: str,
    base: str,
    scope_file: Path,
    evidence_dir: Optional[Path],
    baseline_files: Sequence[str],
    state: Dict[str, object],
    generated_at: datetime,
) -> None:
    aliases: Dict[str, str] = {}

    def write(rel: str, data: bytes) -> None:
        # Some unzip tools ignore the ZIP UTF-8 name flag and mangle non-ASCII
        # names, which breaks `sha256sum -c`. Such paths are stored under an
        # ASCII alias and the mapping is recorded in MANIFEST.json.
        if not rel.isascii():
            parent, _, name = rel.rpartition("/")
            parent = "/".join(p if p.isascii() else "x" + hashlib.sha256(p.encode()).hexdigest()[:8]
                              for p in parent.split("/")) if parent else ""
            stem, dot, suffix = name.rpartition(".")
            safe = "".join(c for c in (stem if dot else name) if c.isascii()).strip("-_. ")
            alias = (f"{safe}-" if safe else "") + hashlib.sha256(rel.encode()).hexdigest()[:8]
            alias += f".{suffix}" if dot and suffix.isascii() else ""
            aliases[rel] = rel = (parent + "/" if parent else "") + alias
        target = stage / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)

    write("scope.md", scope_file.read_bytes())

    write("git/status.txt", str(state["status_porcelain"]).encode("utf-8"))
    write("git/refs.txt", git(["show-ref", "--heads", "--tags"], repo).encode("utf-8"))
    write("git/log.txt", git(
        ["log", "--format=commit %H%nAuthor: %an <%ae>%nDate:   %aI%n%n    %s%n",
         "--stat", "HEAD"], repo).encode("utf-8"))
    write("git/project-changes.patch", git_bytes(
        ["diff", "--no-renames", "--no-color", base, "HEAD"], repo))

    changes = project_files(repo, base)
    for status, path in changes:
        if status.startswith("D"):
            continue
        write("source/" + path, git_bytes(["show", f"HEAD:{path}"], repo))
    write("source/CHANGES.txt", "".join(f"{s}\t{p}\n" for s, p in changes).encode("utf-8"))

    integrity = baseline_integrity(repo, base)
    write("baseline/integrity.json", (json.dumps(integrity, indent=1) + "\n").encode("utf-8"))
    tracked_baseline = {f["path"] for f in integrity["files"] if f["head_blob"] is not None}
    for path in baseline_files:
        if path not in tracked_baseline:
            raise SystemExit(f"--baseline-file is not a tracked baseline file: {path}")
        write("baseline/files/" + path, git_bytes(["show", f"HEAD:{path}"], repo))

    patterns = hygiene.load_private_patterns(repo)
    baseline_map = hygiene._baseline_map(repo)
    for label, entries in (("head", hygiene._tree_entries("HEAD", repo)),
                           ("history", hygiene._history_entries(repo))):
        findings, scanned = hygiene.scan_blobs(
            hygiene._read_blobs(entries, repo), baseline=baseline_map,
            private_patterns=patterns)
        counts = {sev: sum(1 for f in findings if f.severity == sev)
                  for sev in ("ERROR", "WARN", "INFO")}
        report = {"mode": label, "files_scanned": scanned, "counts": counts,
                  "private_patterns_loaded": len(patterns),
                  "findings": [asdict(f) for f in findings]}
        write(f"hygiene/{label}.json", (json.dumps(report, indent=1) + "\n").encode("utf-8"))

    copied = []
    if evidence_dir is not None:
        for src in sorted(p for p in evidence_dir.rglob("*") if p.is_file()):
            rel = src.relative_to(evidence_dir).as_posix()
            if src.suffix.lower() not in EVIDENCE_SUFFIXES:
                raise SystemExit(f"refusing non-text evidence file: {rel}")
            if src.stat().st_size > EVIDENCE_MAX_BYTES:
                raise SystemExit(f"refusing oversized evidence file: {rel}")
            write("evidence/" + rel, src.read_bytes())
            copied.append(rel)

    manifest = {
        "project": PROJECT,
        "generator": f"devtools/make_review_package.py v{GENERATOR_VERSION}",
        "phase": phase,
        "generated_at_utc": generated_at.isoformat(timespec="seconds"),
        "generated_at_utc8": generated_at.astimezone(TZ8).isoformat(timespec="seconds"),
        "head_commit": state["head"],
        "head_tree": state["tree"],
        "branch": state["branch"],
        "commit_time_utc": state["commit_time_utc"],
        "working_tree_dirty": state["dirty"],
        "dirty_entries": state["dirty_entries"],
        "diff_base": base,
        "project_changes": [{"status": s, "path": p} for s, p in changes],
        "evidence_files": copied,
        "baseline_files_included": sorted(baseline_files),
        "path_aliases": dict(sorted(aliases.items())),
    }
    write("MANIFEST.json", (json.dumps(manifest, indent=1) + "\n").encode("utf-8"))

    readme = f"""# Review package: {PROJECT}, {phase}

Generated {manifest['generated_at_utc']} (UTC) at commit `{state['head']}`
on branch `{state['branch']}`; working tree dirty: {state['dirty']}.

Read `scope.md` first: it states what is to be reviewed and which questions
are open. Then:

| Path | Content |
|---|---|
| `MANIFEST.json` | Phase, timestamps, commit, tree, dirty state, file lists; `path_aliases` maps original non-ASCII paths to the ASCII names stored here |
| `git/project-changes.patch` | Every change made on top of the supplied baseline (`{base}`) |
| `git/log.txt`, `git/status.txt`, `git/refs.txt` | History, working-tree status and refs |
| `source/` | Each added or modified file as committed at HEAD; `CHANGES.txt` lists them |
| `baseline/integrity.json` | Per-file state of the supplied baseline at HEAD (unchanged, modified, removed from tracking) |
| `baseline/files/` | Selected baseline files as committed at HEAD, included because the scope cites them |
| `hygiene/head.json`, `hygiene/history.json` | Publication-hygiene scans of HEAD and of every blob in the history |
| `evidence/` | Sanitized validation reports (tests, environment audits) |
| `SHA256SUMS` | SHA-256 of every other file; verify with `sha256sum -c SHA256SUMS` |

The supplied baseline is not duplicated here beyond the files the scope
cites; its integrity is accounted for file by file in `baseline/integrity.json`, and the
archive it came from is identified by SHA-256 in the project documentation.
Infrastructure details (host names, addresses, accounts) are intentionally
omitted from this package.
"""
    write("README.md", readme.encode("utf-8"))

    lines = []
    for path in sorted(p for p in stage.rglob("*") if p.is_file()):
        rel = path.relative_to(stage).as_posix()
        lines.append(f"{sha256_bytes(path.read_bytes())}  {rel}\n")
    write("SHA256SUMS", "".join(lines).encode("utf-8"))


def write_zip(stage: Path, out_path: Path, date_time: Tuple[int, ...]) -> None:
    files = sorted(p for p in stage.rglob("*") if p.is_file())
    with zipfile.ZipFile(out_path, "x", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in files:
            info = zipfile.ZipInfo(path.relative_to(stage).as_posix(), date_time=date_time)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            zf.writestr(info, path.read_bytes())


def verify_zip(out_path: Path) -> int:
    with zipfile.ZipFile(out_path) as zf:
        if zf.testzip() is not None:
            raise SystemExit("ZIP CRC verification failed")
        names = zf.namelist()
        sums = {}
        for line in zf.read("SHA256SUMS").decode("utf-8").splitlines():
            digest, rel = line.split("  ", 1)
            sums[rel] = digest
        expected = set(names) - {"SHA256SUMS"}
        if set(sums) != expected:
            raise SystemExit("SHA256SUMS does not list exactly the package members")
        for rel, digest in sums.items():
            if sha256_bytes(zf.read(rel)) != digest:
                raise SystemExit(f"checksum mismatch for {rel}")
        if any(n.startswith("/") or ".." in n.split("/") for n in names):
            raise SystemExit("unsafe member name in package")
    return len(names)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Build a review package (see module docstring).")
    parser.add_argument("--phase", required=True, help="phase label, e.g. phase0")
    parser.add_argument("--scope-file", type=Path, required=True,
                        help="Markdown file stating the review scope")
    parser.add_argument("--evidence-dir", type=Path, default=None,
                        help="directory of sanitized text evidence to include")
    parser.add_argument("--baseline-file", action="append", default=[],
                        help="baseline path to include under baseline/files/ (repeatable)")
    parser.add_argument("--base", default=DEFAULT_BASE, help="diff base (default: baseline tag)")
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "_review_packages")
    parser.add_argument("--repo", type=Path, default=REPO_ROOT)
    parser.add_argument("--allow-dirty", action="store_true",
                        help="build from a dirty working tree (the state is recorded)")
    args = parser.parse_args(argv)

    repo = args.repo.resolve()
    phase = args.phase.strip()
    if not phase or any(c in phase for c in "/\\ "):
        raise SystemExit("invalid phase label")
    state = repo_state(repo)
    if state["dirty"] and not args.allow_dirty:
        print("\n".join(state["dirty_entries"]), file=sys.stderr)
        raise SystemExit("working tree is dirty; commit first or pass --allow-dirty")

    generated_at = datetime.now(timezone.utc).replace(microsecond=0)
    stamp = generated_at.strftime("%Y%m%dT%H%M%SZ")
    suffix = "-dirty" if state["dirty"] else ""
    name = f"{PROJECT}_{phase}_{stamp}_{state['short']}{suffix}.zip"
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.out_dir / name
    if out_path.exists():
        raise SystemExit(f"refusing to overwrite existing package: {out_path.name}")

    commit_dt = datetime.fromtimestamp(int(state["commit_epoch"]), timezone.utc)
    date_time = (max(1980, commit_dt.year), commit_dt.month, commit_dt.day,
                 commit_dt.hour, commit_dt.minute, commit_dt.second)

    with tempfile.TemporaryDirectory(prefix="review-package-") as tmp:
        stage = Path(tmp) / "package"
        stage.mkdir()
        stage_package(stage, repo, phase=phase, base=args.base,
                      scope_file=args.scope_file.resolve(),
                      evidence_dir=args.evidence_dir.resolve() if args.evidence_dir else None,
                      baseline_files=args.baseline_file,
                      state=state, generated_at=generated_at)
        findings, scanned = hygiene.scan_blobs(
            hygiene._directory_blobs(stage), baseline={},
            private_patterns=hygiene.load_private_patterns(repo), directory_mode=True)
        # Files under baseline/files/ are exact HEAD blobs of the supplied
        # baseline; the HEAD scan below already classifies them and blocks on
        # any error, so the directory pass must not re-judge them as project files.
        blocking = [f for f in findings if f.severity == "ERROR"
                    and not f.path.startswith("baseline/files/")]
        for label in ("head", "history"):
            report = json.loads((stage / "hygiene" / f"{label}.json").read_text(encoding="utf-8"))
            if report["counts"]["ERROR"]:
                raise SystemExit(f"repository hygiene scan ({label}) reports errors; "
                                 "run devtools/check_repo_hygiene.py and fix them first")
        if scanned == 0 or blocking:
            for f in blocking:
                print(f"BLOCKED {f.rule} {f.path}:{f.line}: {f.excerpt}", file=sys.stderr)
            raise SystemExit("package content failed the hygiene gate; nothing written")
        write_zip(stage, out_path, date_time)

    members = verify_zip(out_path)
    digest = sha256_bytes(out_path.read_bytes())
    (out_path.parent / (out_path.name + ".sha256")).write_text(
        f"{digest}  {out_path.name}\n", encoding="utf-8")
    print(json.dumps({"package": out_path.name, "members": members,
                      "bytes": out_path.stat().st_size, "sha256": digest}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
