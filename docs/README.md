# swarm-eod-agent: project documentation

Engineering workspace for an autonomous multi-drone explosive ordnance
disposal (EOD) competition built on the `swarm-rescue` simulator. The
repository holds the competition baseline exactly as supplied, plus this
project's documentation and tooling. The competition agent itself is future
work (Phase 1 onward).

The baseline's own `README.md`, `INSTALL.md` and `doc/` (Chinese participant
guides) are kept verbatim and remain authoritative for the simulator and the
submission interface.

## Documents

| File | Content |
|---|---|
| [PROVENANCE.md](PROVENANCE.md) | Supplied originals and their SHA-256, authorship layers, licensing, open redistribution questions |
| [COMPETITION_RULES.md](COMPETITION_RULES.md) | Scoring for both roles as implemented, constraints on agents, register of document/code discrepancies |
| [SUBMISSION.md](SUBMISSION.md) | Entry points, modification boundaries, package layout, checker behaviour, team-identity workflow, image mode |
| [ENVIRONMENT.md](ENVIRONMENT.md) | Python 3.11 environments, OpenGL findings, remote execution rules, test tiers and results, open risks |
| [WORKFLOW.md](WORKFLOW.md) | Git conventions, public/private separation, hygiene scanner, publication gate, review packages |
| [PHASE0_REPORT.md](PHASE0_REPORT.md) | Phase 0 completion report and entry conditions for the next phase |

## Repository layout

```
.                         baseline root, as supplied (pyproject.toml, README.md, LICENSE, ...)
├── src/swarm_rescue/     simulator package (framework: do not modify)
│   └── solutions/        the only place for competition agent code
├── config/               official local evaluation plans (do not modify)
├── doc/                  official participant and developer guides (Chinese)
├── tests/                baseline test suite
├── scripts/, *.sh        official checker and packaging scripts
├── docs/                 this project's documentation (English)
├── devtools/             project tooling: hygiene scanner, review packages, OpenGL probe, tests
├── environments/         conda specification and resolved package locks
├── _local/               ignored: original archives, private notes, logs, runs
└── _review_packages/     ignored: generated review ZIPs
```

## First steps on a fresh clone

```bash
conda env create -f environments/swarm-eod-agent.conda.yml
conda activate swarm-eod-agent
python -m pip install -e ".[dev]"
cp devtools/templates/team_info.example.yml src/swarm_rescue/solutions/team_info.yml
python -m pytest -q -p no:cacheprovider devtools/tests
./check_submission.sh
```

GL-dependent tests and simulations need an OpenGL context; on a headless
Linux host with an NVIDIA driver set `PYGLET_HEADLESS=True` (see
[ENVIRONMENT.md](ENVIRONMENT.md)).
