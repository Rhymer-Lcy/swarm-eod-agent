#!/usr/bin/env python3
"""Repository hygiene and publication-readiness scanner.

Scans tracked content (a revision, the index, every blob in the history, or a
plain directory such as a review-package staging area) for material that must
not be published or shared:

* forbidden tracked paths (local archives, review packages, the real team
  identity file, binary archives, key files);
* secrets (private-key blocks and common access-token formats);
* private infrastructure details (RFC 1918 / CGNAT IPv4 addresses, Windows
  user-profile paths) and e-mail addresses outside an allow-list;
* owner-defined private patterns, read at run time from the untracked file
  ``_local/config/private-patterns.txt`` (one regular expression per line), so
  that this tracked script never carries the strings it protects;
* unusually large blobs.

With ``--history`` the commit messages, author and committer identities and
tag annotations of every ref are scanned as well, because a push publishes
them together with the files.

Files byte-identical to the supplied competition baseline (tag
``baseline-supplied-5.1.3``) are classified separately: their content was
supplied as-is and is reviewed under the redistribution question, not edited.
Secrets and owner-defined private patterns are errors wherever they occur.

Matched text is never printed in full: findings show the rule, the location
and a masked excerpt. Exit status is 1 when any error is found, else 0.

Standard library only; Python 3.8 or newer.
"""

from __future__ import annotations

import argparse
import itertools
import json
import re
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_TAG = "baseline-supplied-5.1.3"
PRIVATE_PATTERNS_FILE = Path("_local/config/private-patterns.txt")
LARGE_BLOB_BYTES = 1 << 20
BINARY_SNIFF_BYTES = 8000

FORBIDDEN_PATHS: Sequence[Tuple[str, str]] = (
    (r"^_local/", "local-only directory"),
    (r"^_review_packages/", "review-package directory"),
    (r"^src/swarm_rescue/solutions/team_info\.yml$", "real team identity file"),
    (r"\.(zip|pdf|npz|npy|tar|tgz|7z|avi|mp4)$", "archive, document or recording"),
    (r"\.(pem|key|p12|pfx)$", "key or certificate file"),
    (r"(^|/)id_(rsa|ed25519|ecdsa|dsa)", "SSH key file"),
    (r"(^|/)\.env(\.[^/]*)?$", "environment file"),
    (r"(^|/)\.netrc$", "credential file"),
)

SECRET_RULES: Sequence[Tuple[str, str]] = (
    ("private-key-block", r"-----BEGIN (?:[A-Z]+ )?PRIVATE KEY-----"),
    ("github-token", r"\b(?:gh[pousr]_[A-Za-z0-9]{36,}|github_pat_[A-Za-z0-9_]{40,})\b"),
    ("aws-access-key", r"\bAKIA[0-9A-Z]{16}\b"),
    ("api-secret-key", r"\bsk-(?:ant-)?[A-Za-z0-9_-]{24,}\b"),
    ("slack-token", r"\bxox[abprs]-[A-Za-z0-9-]{10,}\b"),
)

_OCTET = r"(?:25[0-5]|2[0-4]\d|1\d\d|[1-9]?\d)"
INFRA_RULES: Sequence[Tuple[str, str]] = (
    ("private-ipv4",
     r"(?<![\d.])(?:10\.{o}\.{o}\.{o}|172\.(?:1[6-9]|2\d|3[01])\.{o}\.{o}"
     r"|192\.168\.{o}\.{o}|100\.(?:6[4-9]|[7-9]\d|1[01]\d|12[0-7])\.{o}\.{o})(?![\d.])"
     .format(o=_OCTET)),
    ("user-profile-path", r"\b[A-Za-z]:[\\/]+(?:Users|Documents and Settings)[\\/]+[^\\/\s]+"),
    ("unix-home-path", r"/home/[A-Za-z0-9_.-]+/"),
)

EMAIL_RULE = r"\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}\b"
EMAIL_ALLOW = (
    re.compile(r"@users\.noreply\.github\.com$", re.I),
    re.compile(r"^noreply@", re.I),
    # Upstream maintainer address published in the supplied pyproject.toml.
    re.compile(r"^emmanuel\.battesti@ensta\.fr$", re.I),
    # Documentation placeholders.
    re.compile(r"@example\.(?:com|org|net)$", re.I),
)


@dataclass
class Finding:
    severity: str          # ERROR, WARN or INFO
    rule: str
    path: str
    line: Optional[int]
    origin: str            # project, baseline or directory
    excerpt: str


@dataclass
class Blob:
    path: str
    sha: Optional[str]
    data: bytes


def _git(args: Sequence[str], *, repo: Path) -> bytes:
    return subprocess.run(
        ["git", "-C", str(repo), *args], check=True, capture_output=True
    ).stdout


def _tree_entries(rev: str, repo: Path) -> List[Tuple[str, str]]:
    """(path, blob sha) for every blob in a tree-ish."""
    out = _git(["ls-tree", "-r", "-z", rev], repo=repo)
    entries = []
    for rec in out.split(b"\0"):
        if not rec:
            continue
        meta, path = rec.split(b"\t", 1)
        _mode, typ, sha = meta.split()
        if typ == b"blob":
            entries.append((path.decode("utf-8"), sha.decode()))
    return entries


def _index_entries(repo: Path) -> List[Tuple[str, str]]:
    out = _git(["ls-files", "-s", "-z"], repo=repo)
    entries = []
    for rec in out.split(b"\0"):
        if not rec:
            continue
        meta, path = rec.split(b"\t", 1)
        _mode, sha, _stage = meta.split()
        entries.append((path.decode("utf-8"), sha.decode()))
    return entries


def _read_blobs(entries: Sequence[Tuple[str, str]], repo: Path) -> Iterator[Blob]:
    """Stream blob contents through one ``git cat-file --batch`` process."""
    if not entries:
        return
    proc = subprocess.Popen(
        ["git", "-C", str(repo), "cat-file", "--batch"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
    )
    assert proc.stdin is not None and proc.stdout is not None
    try:
        for path, sha in entries:
            proc.stdin.write(sha.encode() + b"\n")
            proc.stdin.flush()
            header = proc.stdout.readline().split()
            size = int(header[2])
            data = proc.stdout.read(size)
            proc.stdout.read(1)  # trailing newline
            yield Blob(path=path, sha=sha, data=data)
    finally:
        proc.stdin.close()
        proc.wait()


def _history_entries(repo: Path) -> List[Tuple[str, str]]:
    """Every distinct blob reachable from any ref, with the first path seen."""
    seen: Dict[str, str] = {}
    revs = _git(["rev-list", "--all"], repo=repo).decode().split()
    for rev in revs:
        for path, sha in _tree_entries(rev, repo):
            seen.setdefault(sha, path)
    return [(path, sha) for sha, path in seen.items()]


def history_metadata_blob(repo: Path) -> Blob:
    """Commit messages, author and committer identities, and tag annotations.

    These are published with the history but are not file contents, so they
    are scanned as one synthetic text under the path ``<git-metadata>``.
    """
    commits = _git(["log", "--all", "--format=commit %H%nauthor %an <%ae>%n"
                    "committer %cn <%ce>%n%B%n"], repo=repo)
    tags = _git(["for-each-ref", "refs/tags", "--format=tag %(refname)%n"
                 "tagger %(taggername) %(taggeremail)%n%(contents)%n"], repo=repo)
    return Blob(path="<git-metadata>", sha=None, data=commits + tags)


def _directory_blobs(root: Path) -> Iterator[Blob]:
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        yield Blob(path=path.relative_to(root).as_posix(), sha=None, data=path.read_bytes())


def load_private_patterns(repo: Path) -> List[re.Pattern]:
    path = repo / PRIVATE_PATTERNS_FILE
    patterns: List[re.Pattern] = []
    if not path.is_file():
        return patterns
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line and not line.startswith("#"):
            patterns.append(re.compile(line))
    return patterns


def _mask(text: str, start: int, end: int, width: int = 40) -> str:
    left = text[max(0, start - width):start]
    right = text[end:end + width]
    hidden = "*" * min(8, max(3, end - start))
    return (left + "[" + hidden + "]" + right).replace("\t", " ").strip()


def _line_of(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def scan_blobs(
    blobs: Iterable[Blob],
    *,
    baseline: Dict[str, str],
    private_patterns: Sequence[re.Pattern],
    directory_mode: bool = False,
) -> Tuple[List[Finding], int]:
    findings: List[Finding] = []
    count = 0
    secret_res = [(name, re.compile(rx)) for name, rx in SECRET_RULES]
    infra_res = [(name, re.compile(rx)) for name, rx in INFRA_RULES]
    email_re = re.compile(EMAIL_RULE)
    for blob in blobs:
        count += 1
        if directory_mode:
            origin = "directory"
        elif blob.sha is not None and baseline.get(blob.path) == blob.sha:
            origin = "baseline"
        else:
            origin = "project"
        for rx, reason in FORBIDDEN_PATHS:
            if re.search(rx, blob.path):
                # The supplied baseline ships a demonstration team_info.yml; that
                # exact blob is public sample data. Any other content is an error.
                severity = "INFO" if origin == "baseline" else "ERROR"
                findings.append(Finding(severity, "forbidden-path", blob.path, None, origin, reason))
        if len(blob.data) > LARGE_BLOB_BYTES:
            findings.append(Finding("WARN", "large-blob", blob.path, None, origin,
                                    f"{len(blob.data)} bytes"))
        if b"\0" in blob.data[:BINARY_SNIFF_BYTES]:
            continue
        text = blob.data.decode("utf-8", errors="replace")
        for name, rx in secret_res:
            for m in rx.finditer(text):
                findings.append(Finding("ERROR", "secret:" + name, blob.path,
                                        _line_of(text, m.start()), origin,
                                        _mask(text, m.start(), m.end())))
        for index, rx in enumerate(private_patterns, start=1):
            for m in rx.finditer(text):
                findings.append(Finding("ERROR", f"private-pattern#{index}", blob.path,
                                        _line_of(text, m.start()), origin,
                                        _mask(text, m.start(), m.end())))
        for name, rx in infra_res:
            for m in rx.finditer(text):
                severity = "INFO" if origin == "baseline" else "ERROR"
                findings.append(Finding(severity, "infra:" + name, blob.path,
                                        _line_of(text, m.start()), origin,
                                        _mask(text, m.start(), m.end())))
        for m in email_re.finditer(text):
            address = m.group(0)
            if any(allow.search(address) for allow in EMAIL_ALLOW):
                continue
            severity = "INFO" if origin == "baseline" else "WARN"
            findings.append(Finding(severity, "email-address", blob.path,
                                    _line_of(text, m.start()), origin,
                                    _mask(text, m.start(), m.end())))
    return findings, count


def _baseline_map(repo: Path) -> Dict[str, str]:
    try:
        return dict(_tree_entries(BASELINE_TAG, repo))
    except subprocess.CalledProcessError:
        return {}


def run(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--rev", default=None, help="scan the tree of this revision (default HEAD)")
    target.add_argument("--index", action="store_true", help="scan the staged tree")
    target.add_argument("--history", action="store_true",
                        help="scan every distinct blob reachable from any ref")
    target.add_argument("--directory", type=Path, default=None,
                        help="scan a plain directory (e.g. a review-package staging area)")
    parser.add_argument("--repo", type=Path, default=REPO_ROOT)
    parser.add_argument("--json", type=Path, default=None, help="write the findings as JSON")
    parser.add_argument("--quiet", action="store_true", help="print the summary only")
    args = parser.parse_args(argv)

    repo = args.repo.resolve()
    private_patterns = load_private_patterns(repo)
    baseline = _baseline_map(repo)

    if args.directory is not None:
        mode = f"directory:{args.directory.name}"
        blobs: Iterable[Blob] = _directory_blobs(args.directory.resolve())
    elif args.index:
        mode = "index"
        blobs = _read_blobs(_index_entries(repo), repo)
    elif args.history:
        mode = "history"
        blobs = itertools.chain(_read_blobs(_history_entries(repo), repo),
                                [history_metadata_blob(repo)])
    else:
        rev = args.rev or "HEAD"
        mode = f"rev:{rev}"
        blobs = _read_blobs(_tree_entries(rev, repo), repo)

    findings, scanned = scan_blobs(
        blobs,
        baseline=baseline,
        private_patterns=private_patterns,
        directory_mode=args.directory is not None,
    )
    counts = {sev: sum(1 for f in findings if f.severity == sev) for sev in ("ERROR", "WARN", "INFO")}
    summary = {
        "mode": mode,
        "files_scanned": scanned,
        "baseline_tag_found": bool(baseline),
        "private_patterns_loaded": len(private_patterns),
        "counts": counts,
    }
    if not args.quiet:
        for f in findings:
            if f.severity != "INFO":
                loc = f"{f.path}:{f.line}" if f.line else f.path
                print(f"{f.severity} {f.rule} [{f.origin}] {loc}: {f.excerpt}")
    info_by_rule: Dict[str, int] = {}
    for f in findings:
        if f.severity == "INFO":
            info_by_rule[f.rule] = info_by_rule.get(f.rule, 0) + 1
    print(json.dumps({**summary, "info_by_rule": info_by_rule}, sort_keys=True))
    if args.json is not None:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(
            {**summary, "findings": [asdict(f) for f in findings]}, indent=1, sort_keys=True
        ) + "\n", encoding="utf-8")
    if scanned == 0:
        print("ERROR no files scanned", file=sys.stderr)
        return 1
    return 1 if counts["ERROR"] else 0


if __name__ == "__main__":
    raise SystemExit(run())
