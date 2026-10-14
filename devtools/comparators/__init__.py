"""Frozen comparator agents for paired evaluation.

Each subpackage is a byte-identical copy of the agent package
``src/swarm_rescue/solutions/eod`` at a recorded commit, so a baseline
candidate can run in the same harness, process and simulator version as the
current development candidate without checking out old code. The copies are
never edited: ``<name>.json`` records the source commit and the SHA-256 of
every file, and ``devtools/tests/test_comparators.py`` verifies both against
Git history. A comparator is selected in the runner with
``--agent devtools.comparators.<name>.shell:EodRescueDrone`` (or
``EodPlaceDrone``).

* ``p1_ddc6897``: B0.1 (blue) and R0.1 (red), the Phase 1 candidates at
  ``ddc6897``.
* ``p2_540054b``: B1 (blue) and R1 (red), the Phase 2 candidates at
  ``540054b``.
* ``p3_067818a``: B2 (blue) and R2 (red), adopted in Phase 3, at the
  Phase 3 final commit ``067818a`` (the blue files are identical to those of
  the Phase 3 blue rounds at ``bb569c3``, the whole package to the R2 freeze
  at ``5e0803d``). Frozen for Phase 4; any later change is a new candidate
  with a new identifier.
* ``p4_ce09cb7``: the Phase 4 blue candidate B3 (B2 plus a search for an
  unknown disposal center while carrying) with R2, at ``ce09cb7``. Not
  adopted (docs/PHASE4_PLAN.md, addendum 3); kept for its tests and as a
  starting point.

Nothing under ``solutions/`` may import this package.
"""
