"""Development-only simulation laboratory for swarm-eod-agent.

Nothing in this package is part of a competition submission: it lives outside
``src/swarm_rescue/solutions/`` and is never packaged. It contains

* ``runner``: runs single rounds through the official ``Launcher`` and its
  score managers and appends machine-readable records to a run ledger, with
  a round budget;
* ``occupancy`` and ``remote_run.sh``: the shared-host occupancy gate and the
  bounded run wrapper;
* ``diagnostics``: an evaluation-only wrapper that logs ground truth (and a
  read-only snapshot of the agent's internal state) for later analysis
  without feeding anything back to the agent;
* ``summarize``: the ledger summary used in reports;
* ``fixtures``: extraction of walls-only truth grids from the supplied maps;
* ``probe`` and ``analyze_probe``: scripted characterization agents
  (dynamics, sensors, grasping) and their offline analysis;
* ``layouts``: synthetic bomb layouts for blue-team tests, labelled synthetic;
* ``offline_sim`` and ``offline_eval``: a GL-free kinematic approximation of
  the simulator and a driver that runs the brains in it (development
  signals only, never competition evidence).

Ground truth appears only here. Agent code under ``solutions/`` never imports
this package.
"""
