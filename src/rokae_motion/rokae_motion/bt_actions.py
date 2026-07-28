"""Concrete Rokae action implementations used by the behavior tree.

YAML parsing and action selection belong to ``bt_executor.py``. This module
maps each validated action type to its concrete execution behavior.
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Dict

if TYPE_CHECKING:
    from .bt_executor import ActionSpec
    from .executor_services import RokaeActionExecutor


class BehaviorAction(ABC):
    """Base class implemented by every executable YAML action type."""

    action_type: str

    @abstractmethod
    def execute(
        self, executor: "RokaeActionExecutor", action: "ActionSpec"
    ) -> None:
        """Execute one validated action."""


class MoveAbsJBehavior(BehaviorAction):
    """Execute absolute joint targets through the MoveAbsJ action servers."""

    action_type = "move_absj"

    def execute(
        self, executor: "RokaeActionExecutor", action: "ActionSpec"
    ) -> None:
        executor.move_absj(
            action_name=action.action_name,
            targets=action.parameters["targets"],
        )


class MoveLBehavior(BehaviorAction):
    """Execute Cartesian targets through the MoveL services."""

    action_type = "move_l"

    def execute(
        self, executor: "RokaeActionExecutor", action: "ActionSpec"
    ) -> None:
        executor.move_l(action.parameters)


class MoveLRelativeBehavior(BehaviorAction):
    """Translate from each arm's current TCP pose while preserving posture."""

    action_type = "move_l_relative"

    def execute(
        self, executor: "RokaeActionExecutor", action: "ActionSpec"
    ) -> None:
        executor.move_l_relative(action.parameters)


class VisionBehavior(BehaviorAction):
    """Trigger the detector and cache its selected vision points."""

    action_type = "vision"

    def execute(
        self, executor: "RokaeActionExecutor", action: "ActionSpec"
    ) -> None:
        executor.trigger_vision(action.parameters)


class MoveLVisionBehavior(BehaviorAction):
    """Resolve one cached vision motion mode and execute MoveL."""

    action_type = "move_l_vision"

    def execute(
        self, executor: "RokaeActionExecutor", action: "ActionSpec"
    ) -> None:
        executor.move_l_vision(action.parameters)


class HandBehavior(BehaviorAction):
    """Control one or both Linker Hands through the arm end CAN buses."""

    action_type = "hand"

    def execute(
        self, executor: "RokaeActionExecutor", action: "ActionSpec"
    ) -> None:
        executor.control_hand(action.parameters)


class NavigateBehavior(BehaviorAction):
    """Send one scheduler station command and wait for chassis arrival."""

    action_type = "navigate"

    def execute(
        self, executor: "RokaeActionExecutor", action: "ActionSpec"
    ) -> None:
        executor.navigate(action.parameters)


class WaitBehavior(BehaviorAction):
    """Pause the sequence while keeping ROS callbacks responsive."""

    action_type = "wait"

    def execute(
        self, executor: "RokaeActionExecutor", action: "ActionSpec"
    ) -> None:
        executor.wait(action.parameters["duration_s"])


BEHAVIORS: Dict[str, BehaviorAction] = {
    behavior.action_type: behavior
    for behavior in (
        MoveAbsJBehavior(),
        MoveLBehavior(),
        MoveLRelativeBehavior(),
        VisionBehavior(),
        MoveLVisionBehavior(),
        HandBehavior(),
        NavigateBehavior(),
        WaitBehavior(),
    )
}


def execute_behavior(
    executor: "RokaeActionExecutor", action: "ActionSpec"
) -> None:
    """Dispatch one validated action to its concrete behavior."""
    try:
        behavior = BEHAVIORS[action.action_type]
    except KeyError as exc:
        raise ValueError(
            f"no behavior implementation for {action.action_type!r}"
        ) from exc
    behavior.execute(executor, action)
