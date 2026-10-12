# Source provenance and licensing

Dates are business dates in UTC+8.

## Supplied originals

| Artifact | SHA-256 | Size | Status |
|---|---|---|---|
| `swarm-eod-main.zip`, the competition baseline | `b4db0f661884744eca5b52f30122bbc85d7407c56455a0627420652e18dac8b6` | 1,058,951 bytes | Archived locally, not tracked, not redistributed as an archive |
| `评分规则.pdf`, the scoring rules | `924595248194328b7475415c3f918326e1c8f4b0d4d029fe639ebc262db9ecc1` | 956,924 bytes | Archived locally, not tracked, not redistributed |

Both values were supplied by the owner as independently verified and were
re-computed on 2026-10-12 before the files were moved (copy, verify, delete)
into the ignored `_local/source-archives/` directory under checksum-suffixed
names, with a machine-readable manifest. Neither file was modified.

The baseline ZIP was downloaded by the owner from Gitee as a branch archive.
**The download URL was not recorded and is unknown.** The ZIP comment holds
`53e129bd7d2efc82c29deac2ecd8c57a536e74e7`; by the `git archive` convention this is
the commit ID of the archived tree, but it has not been verified against any
remote. No Gitee remote is configured in this repository, and the project is
not presented as a mirror of any repository.

## Layers of authorship

| Layer | Content | Origin and terms |
|---|---|---|
| 1. Upstream simulator | Swarm-Rescue by Emmanuel Battesti, built on simple-playgrounds | `LICENSE`: MIT, Copyright (c) 2021 Emmanuel Battesti, with an attribution notice for simple-playgrounds (MIT, Copyright (c) 2021 Michael Garcia Ortiz). Upstream: https://github.com/emmanuel-battesti/swarm-rescue (the URL is the one given in `pyproject.toml`). |
| 2. Competition baseline | `swarm-rescue` 5.1.3 as supplied: the EOD adaptation (bombs, disposal centers, red/blue roles, scoring), Map01-Map05, Chinese participant guides, configuration, checker and examples | Supplied by the competition organizer through Gitee. Authorship of the competition-specific changes is not stated. The only license file is the MIT `LICENSE` above; whether it was meant to cover the organizer's additions is not stated. |
| 3. This project's agent | Future code under `src/swarm_rescue/solutions/` | Not written in Phase 0. |
| 4. Project infrastructure | `docs/`, `devtools/`, `environments/`, `local project instructions`, `.gitattributes`, the appended part of `.gitignore` | Written for this project. |

The repository history keeps the layers apart: the commit tagged
`baseline-supplied-5.1.3` contains exactly the 202 files of the archive (bytes
and executable bits verified against the ZIP); every later commit is project
work. `devtools/make_review_package.py` reports, file by file, which baseline
files are unchanged, modified (only `.gitignore`, by appending) or removed from
tracking (only `src/swarm_rescue/solutions/team_info.yml`, the demonstration
identity).

The scoring PDF is summarized in English in
[COMPETITION_RULES.md](COMPETITION_RULES.md) with attribution; the PDF itself
remains the authority. The baseline's Chinese documents are kept verbatim and
are not translated in place.

## Redistribution: open questions (publication is on hold)

1. Whether the organizer permits participants to publish the supplied
   baseline, in particular the layer-2 material (guides, maps, configuration,
   checker), even though the archive carries an MIT license file.
2. Whether competition rules restrict publishing participant solutions before
   or during the evaluated competition. No such rule appears in the supplied
   material; no other rules document was available to this project.
3. Whether the scoring rules may be summarized publicly (they are not
   reproduced, only summarized).

Until these are answered, the GitHub repository is not created or pushed. If
the baseline may not be redistributed, a public repository would have to hold
only layers 3 and 4, with instructions to obtain the baseline from the
organizer and verify it by SHA-256.
