"""Unit tests for scheduler-to-Seer navigation mappings."""

from action_msgs.msg import GoalStatus
import pytest

from rokae_motion.chassis_navigation import (
    make_navigation_goal,
    navigation_command_details,
    navigation_terminal_state,
)
from seer_interfaces.action import Navigate


@pytest.mark.parametrize(
    "command, expected",
    [
        ("LM1", ("LM1", "LM1", "ARRIVE_HOME")),
        ("lm2", ("LM2", "LM2", "ARRIVE_A")),
        (" LM3 ", ("LM3", "LM3", "ARRIVE_B")),
    ],
)
def test_navigation_command_details(command, expected) -> None:
    assert navigation_command_details(command) == expected


def test_navigation_command_details_rejects_unknown_command() -> None:
    with pytest.raises(ValueError, match="unsupported navigation command"):
        navigation_command_details("LM_UNKNOWN")


def test_make_navigation_goal_uses_station_action_mode() -> None:
    goal = make_navigation_goal("LM2")

    assert goal.mode == Navigate.Goal.MODE_STATION
    assert goal.target_id == "LM2"
    assert goal.source_id == "SELF_POSITION"
    assert goal.task_id.startswith("bt_")
    assert len(goal.task_id) == 35


@pytest.mark.parametrize(
    "status, success, expected",
    [
        (GoalStatus.STATUS_SUCCEEDED, True, "ARRIVE_HOME"),
        (GoalStatus.STATUS_SUCCEEDED, False, "NAVIGATION_FAILED"),
        (GoalStatus.STATUS_ABORTED, False, "NAVIGATION_FAILED"),
        (GoalStatus.STATUS_CANCELED, False, "NAVIGATION_CANCELED"),
    ],
)
def test_navigation_terminal_state_uses_action_result(
    status, success, expected
) -> None:
    assert navigation_terminal_state("LM1", status, success) == expected
