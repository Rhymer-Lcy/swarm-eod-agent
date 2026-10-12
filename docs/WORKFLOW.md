# Git workflow, publication controls and review packages

## Repository conventions

* Branch `main`; commits go straight to `main` unless a task calls for an
  isolated branch or worktree. Never `git stash` work in progress; use a
  worktree instead.
* Commit messages: Conventional Commits, one line, at most 72 ASCII
  characters, lowercase imperative, `type(scope): subject`, where the scope is
  the top-level directory touched (`devtools`, `docs`, `environments`,
  `solutions`, ...). Repository-wide changes use `chore:` without a scope.
  No message body and no co-author trailers. One atomic change per commit.
* Author identity is set repository-locally to the GitHub account's noreply
  address so that no personal mailbox enters the history.
* Line endings: `.gitattributes` stores and checks out text as LF on every
  platform (`* text=auto eol=lf`), overriding Git for Windows'
  `core.autocrlf=true`; the supplied baseline is all-LF, so the checkout is
  byte-identical to the archive.
* The baseline's `.gitignore` is kept as supplied; project rules are only
  appended after a banner. `docs/` is re-included explicitly because the
  baseline ignores `/docs`. A repository-local `.git/info/exclude` repeats
  `/_local/` and `/_review_packages/` as a second guard.
* Before every commit: inspect the staged file list and diff, then run
  `python devtools/check_repo_hygiene.py --index`.
* The tag `baseline-supplied-5.1.3` marks the pristine import. Project diffs
  and the review package are computed against it.

## Public and private material

| Tracked (public once approved) | Local only (`_local/`, ignored) |
|---|---|
| Baseline source as supplied (subject to the redistribution question) | Original ZIP and PDF with their manifest |
| `docs/`, `devtools/`, `environments/`, `local project instructions` | Real `team_info.yml` (at its required path, ignored) |
| Placeholder identity template | Server access notes, private patterns, wheels, bundles, logs, run outputs |
| | `_review_packages/` (generated ZIPs) |

Addresses, host names, accounts, workstation paths and the registered team
identity are never written into tracked files. The owner's list of such
strings lives in `_local/config/private-patterns.txt` (one regular expression
per line) and is enforced by the hygiene scanner without being stored in the
repository.

## Hygiene scanner

```bash
python devtools/check_repo_hygiene.py              # tree at HEAD
python devtools/check_repo_hygiene.py --index      # staged tree
python devtools/check_repo_hygiene.py --history    # every blob in every ref
python devtools/check_repo_hygiene.py --directory <dir>
```

Errors: forbidden tracked paths (local directories, the real identity file,
archives, PDFs, key files), secrets (private-key blocks, common token
formats), private IPv4 ranges and user-profile or home paths in project files,
and any owner-defined private pattern. Warnings: unusual e-mail addresses,
blobs over 1 MiB. Findings in files byte-identical to the supplied baseline are
informational (six upstream author home paths in `src/swarm_rescue/tools/` and
the demonstration `team_info.yml` in the import commit). The exit status is 1
on any error. Planted-error tests in `devtools/tests/` show that each rule fires.

## Publication gate (GitHub)

Target: a public repository `swarm-eod-agent` under the owner's GitHub
account, reached over the existing authenticated GitHub CLI. Before the first
push all of the following must hold, and the owner must give an explicit go:

1. Redistribution of the supplied baseline and publication of participant work
   are confirmed as permitted (open: see [PROVENANCE.md](PROVENANCE.md)).
2. `check_repo_hygiene.py --history` reports no errors, and every warning has
   been read.
3. The tracked file list contains no archive, PDF, identity file, log or
   generated output, and no blob is unexpectedly large.
4. The commit list to be published has been reviewed as a set (message
   convention, author identity, content).
5. The remote name is free or is an empty repository owned by the account; an
   existing repository is never overwritten, force-pushed or repurposed.

Approved procedure: `gh repo create <owner>/swarm-eod-agent --public
--source . --remote origin --push` (or create the repository, add `origin`
with the SSH or HTTPS URL, then `git push -u origin main` and
`git push origin baseline-supplied-5.1.3`), then verify with
`git ls-remote origin` and
`gh api repos/<owner>/swarm-eod-agent/commits --jq '.[0].author.login'`.
Mistakes are corrected with new commits, never by force-pushing.

**Status at the end of Phase 0: HELD.** Item 1 is unresolved. Nothing has
been created on or pushed to GitHub.

## Review packages

`devtools/make_review_package.py` writes a self-contained ZIP to
`_review_packages/` for manual upload to an external reviewer:

```bash
python devtools/make_review_package.py --phase phase0 \
    --scope-file <scope.md> --evidence-dir <sanitized evidence dir>
```

Contents: `README.md`, `MANIFEST.json` (phase, timestamps in UTC and UTC+8,
commit, tree, dirty state), `scope.md`, `git/` (status, refs, log and the full
project diff against the baseline tag), `source/` (each added or modified file
at HEAD, without duplicating the baseline), `baseline/integrity.json`,
`baseline/files/` (baseline files named with `--baseline-file`, for example
the scoring code that a finding cites),
`hygiene/` (scans of HEAD and of the history), `evidence/` (sanitized text
reports), and `SHA256SUMS`.

Gates: a dirty tree is refused unless `--allow-dirty` (then recorded and the
name gets `-dirty`); an existing package is never overwritten; evidence must
be text of known types under 2 MiB; the build stops if the repository scans
report errors or if the staged package contains anything the scanner flags,
including the owner's private patterns. After writing, the ZIP is reopened,
CRC-checked and compared member by member with `SHA256SUMS`, and a `.sha256`
sidecar is written. Names follow
`swarm-eod-agent_<phase>_<UTC yyyymmddThhmmssZ>_<short commit>[-dirty].zip`.

Evidence is sanitized before it is placed in the evidence directory:
server paths become `<server-repo>`, `<server-conda>` or `<home>`, and no host
address or account name is included. The packages leave the workstation only
by the owner's manual upload.
