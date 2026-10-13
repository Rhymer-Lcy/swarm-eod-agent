"""Physical constants and tunable parameters.

The motion constants were measured on the supplied simulator (swarm-rescue
5.1.3) with scripted probe runs and fit exactly (residual below 1e-4):
with ``u`` the commanded thrust vector in the world frame (the simulator
clips each component to [-1, 1] and rescales the vector to unit norm when it
is longer), the drone velocity ``v`` (px per step) and position ``p`` evolve
over one control step as

    v[t+1] = VEL_DECAY * v[t] + VEL_GAIN * u[t]
    p[t+1] = p[t] + POS_VEL * v[t] + POS_GAIN * u[t]

and the heading changes by ``ROT_GAIN * rotation_command`` per step. Positive
lateral thrust moves the drone to its left; positive rotation is
counter-clockwise. Sensor angles are counter-clockwise relative to the
heading. Carrying a bomb triples the moving mass, and the bomb pulls back
towards its spawn point; the measured carrying speed at full thrust is about
3.1 px per step.
"""

from __future__ import annotations

import math

# --- motion model (measured) -------------------------------------------------
VEL_DECAY = 0.95
VEL_GAIN = 0.57293
POS_VEL = 0.97729
POS_GAIN = 0.52908
ROT_GAIN = 0.17591
MAX_SPEED = VEL_GAIN / (1.0 - VEL_DECAY)          # about 11.46 px per step
CARRY_GAIN_FACTOR = 1.0 / 3.0                      # mass 150 instead of 50
CARRY_SPEED = 3.0                                  # px per step, full thrust

# --- geometry (measured) ---------------------------------------------------------
DRONE_RADIUS = 15.0        # centre-to-wall distance of a drone pushed into a wall
BOMB_RADIUS = 12.0
GRASP_REACH = 29.0         # centre distance at which a frontal grasp triggers
CARRY_OFFSET = 10.6        # carried bomb centre ahead of the drone centre
MIN_BOMB_SEPARATION = 24.0

# --- sensors (from the simulator source, confirmed by the probe) -----------------
LIDAR_RAYS = 181
LIDAR_RANGE = 300.0
LIDAR_NO_HIT = 285.0       # readings above this are treated as "no return"
LIDAR_NOISE = 2.5
SEMANTIC_RANGE = 200.0
SEMANTIC_BIAS = 2.5        # semantic distances carry N(2.5, 1) added noise
GPS_NOISE = 5.0
COMPASS_NOISE = math.radians(4.0)
COMM_RANGE = 250.0

# --- estimator --------------------------------------------------------------------
HEADING_GAIN_MIN = 0.004
POSITION_GAIN_BASE = 0.03
POSITION_GAIN_PER_SPEED = 0.006
DISTURBED_POSITION_GAIN = 0.15
ODOMETER_DIST_TOL = 0.6
ODOMETER_ANGLE_TOL = math.radians(3.5)

# --- mapping (coarse occupancy grid) ------------------------------------------
CELL = 8.0                 # coarse cell size in px
LOGODDS_HIT = 0.9
LOGODDS_FREE = -0.35
LOGODDS_MIN = -4.0
LOGODDS_MAX = 6.0
OCCUPIED_THRESHOLD = 1.0
FREE_THRESHOLD = -0.7
FREE_RANGE = 250.0         # free-space carving range (blue)
SCAN_PERIOD = 2            # occupancy update every N steps

# --- motion control ---------------------------------------------------------------
INFLATE_RADIUS = DRONE_RADIUS + 9.0
WAYPOINT_LOOKAHEAD = 60.0
TEAMMATE_REPEL_RADIUS = 48.0
WALL_REPEL_RADIUS = 28.0
STUCK_WINDOW = 40
STUCK_DISTANCE = 12.0
