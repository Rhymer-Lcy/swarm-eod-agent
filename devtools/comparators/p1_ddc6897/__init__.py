"""swarm-eod-agent competition agents (blue rescue and red placement).

Layout: ``shell`` holds the ``DroneAbstract`` subclasses that talk to the
simulator through the official sensor and actuator API only; everything else
is a pure-Python "brain" (``blue``, ``red``) built from shared components
(``estimator``, ``grid``, ``motion``, ``comms``, ``wallmap``) that consume an
``Observation`` and return a ``Command``. The brains never see simulator
objects, which keeps them testable without an OpenGL context and makes the
information boundary explicit.
"""
