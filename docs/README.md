# swarm-eod-agent: project documentation

Engineering workspace for an autonomous multi-drone explosive ordnance
disposal (EOD) competition built on the `swarm-rescue` simulator. The
repository holds the competition baseline exactly as supplied, plus this
project's agents, documentation and tooling. Phase 1 (2026-10-13) added the
first functional candidates for both roles, B0.1 (blue, rescue) and R0.1
(red, place), in `src/swarm_rescue/solutions/eod/`; see
[PHASE1_REPORT.md](PHASE1_REPORT.md). Phase 2 (2026-10-13) replaced them with
B1 and R1 after paired comparisons and kept the Phase 1 agents as a frozen
comparator; see [PHASE2_REPORT.md](PHASE2_REPORT.md). Phase 3 (2026-10-14)
fixed delivery to partly walled disposal centers (B2) and red placement (R2),
again against frozen comparators; see [PHASE3_REPORT.md](PHASE3_REPORT.md)
for the status and limits. Phase 4 (2026-10-14) validated the frozen B2 and
R2 on new layouts and synthetic custom geometry, found a hidden-center
failure, rejected the candidate B3 under its preregistered rule and kept B2
and R2; see [PHASE4_REPORT.md](PHASE4_REPORT.md).

The baseline's own `README.md`, `INSTALL.md` and `doc/` (Chinese participant
guides) are kept verbatim and remain authoritative for the simulator and the
submission interface.

## Documents

| File | Content |
|---|---|
| [PROVENANCE.md](PROVENANCE.md) | Supplied originals and their SHA-256, authorship layers, licensing, publication decision |
| [COMPETITION_RULES.md](COMPETITION_RULES.md) | Scoring for both roles as implemented, constraints on agents, register of document/code discrepancies |
| [ORGANIZER_QUESTIONS.md](ORGANIZER_QUESTIONS.md) | Draft clarification requests to the organizer (not sent) |
| [SUBMISSION.md](SUBMISSION.md) | Entry points, modification boundaries, package layout, checker behaviour, team-identity workflow, image mode |
| [ENVIRONMENT.md](ENVIRONMENT.md) | Python 3.11 environments, OpenGL findings, remote execution rules, test tiers and results, open risks |
| [WORKFLOW.md](WORKFLOW.md) | Git conventions, public/private separation, hygiene scanner, publication gate, review packages |
| [PHASE0_REPORT.md](PHASE0_REPORT.md) | Phase 0 completion report and entry conditions for the next phase |
| [AGENT_DESIGN.md](AGENT_DESIGN.md) | Verified simulator facts, agent architecture, blue and red policies, known defects |
| [RESEARCH.md](RESEARCH.md) | Method families assessed, selection and rejected alternatives, verified references |
| [EXPERIMENTS.md](EXPERIMENTS.md) | Harness, benchmark partitions, run ledger, results with their uncertainty, failure analysis |
| [PHASE1_REPORT.md](PHASE1_REPORT.md) | Phase 1 completion report and the next iteration |
| [PHASE2_PLAN.md](PHASE2_PLAN.md) | Phase 2 evaluation plan, frozen before its first round: arms, scenarios, seeds, metrics, decision rules, round allocation |
| [PHASE2_REPORT.md](PHASE2_REPORT.md) | Phase 2 completion report: estimator fix, B1 and R1, parity, budget, generalization, open defects |
| [PHASE3_PLAN.md](PHASE3_PLAN.md) | Phase 3 evaluation plan, frozen before its first round, with dated addenda |
| [PHASE3_REPORT.md](PHASE3_REPORT.md) | Phase 3 completion report: Map03 delivery root cause, B2, red diagnosis and R2, budget, limits |
| [PHASE4_PLAN.md](PHASE4_PLAN.md) | Phase 4 validation protocol (partitions, custom-geometry scenarios, budget, decision rules), frozen before evaluation, with dated addenda |
| [PHASE4_REPORT.md](PHASE4_REPORT.md) | Phase 4 completion report: generalization results, Risks A-C, the B3 decision, reproducibility, limits |

## Repository layout

```
.                         baseline root, as supplied (pyproject.toml, README.md, LICENSE, ...)
├── src/swarm_rescue/     simulator package (framework: do not modify)
│   └── solutions/        the only place for competition agent code
│       ├── my_drone_eval.py   entry point (selects the eod agents)
│       └── eod/               blue and red agents and their components
├── config/               official local evaluation plans (do not modify)
├── doc/                  official participant and developer guides (Chinese)
├── tests/                baseline test suite
├── scripts/, *.sh        official checker and packaging scripts
├── docs/                 this project's documentation (English)
├── devtools/             project tooling: hygiene scanner, review packages, OpenGL probe, tests
│   ├── simlab/           simulation harness, probes, fixtures, offline approximation,
│   │                     Phase 4 custom-geometry scenarios (p4_scenarios.py)
│   └── comparators/      frozen agent packages (B0.1/R0.1, B1/R1, B2/R2, B3)
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
