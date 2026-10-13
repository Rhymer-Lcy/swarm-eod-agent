"""
Submission entry point used by the launcher.

``drone_class_for_mode`` returns the swarm-eod-agent drone class for a team
mode: ``EodRescueDrone`` for the blue team (rescue) and ``EodPlaceDrone`` for
the red team (place). The supplied example agents remain in this directory as
reference baselines and are not used here.
"""

from swarm_rescue.simulation.reporting.team_mode import TeamMode
from swarm_rescue.solutions.eod.shell import EodPlaceDrone, EodRescueDrone


def drone_class_for_mode(mode: str):
    """
    Return the concrete drone class for the given team mode.
    """
    team_mode = TeamMode.from_string(mode)
    if team_mode == TeamMode.PLACE:
        return EodPlaceDrone
    return EodRescueDrone
