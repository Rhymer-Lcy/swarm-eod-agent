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

Nothing under ``solutions/`` may import this package.
"""
