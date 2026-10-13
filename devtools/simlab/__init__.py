"""Development-only simulation laboratory for swarm-eod-agent.

Nothing in this package is part of a competition submission: it lives outside
``src/swarm_rescue/solutions/`` and is never packaged. It contains

* ``runner``: runs single rounds through the official ``Launcher`` and its
  score managers and appends machine-readable records to a run ledger;
* ``diagnostics``: an evaluation-only wrapper that logs ground truth for
  later analysis without feeding anything back to the agent;
* ``fixtures``: extraction of walls-only truth grids from the supplied maps;
* ``probe``: scripted characterization agents (dynamics, sensors, grasping).

Ground truth appears only here. Agent code under ``solutions/`` never imports
this package.
"""
