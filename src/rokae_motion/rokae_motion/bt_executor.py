"""Load, validate, select and execute a Rokae behavior-tree YAML file."""

from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Optional, Sequence

import rclpy
import yaml

from .bt_actions import BEHAVIORS, execute_behavior
from .bt_runner_config import RunnerConfig
from .executor_services import RokaeActionExecutor
from .ros_dual_moveabsj_client import ProgramError, validate_joint_target


SUPPORTED_ACTION_TYPES = set(BEHAVIORS)
ARM_NAMES = ("left", "right")


class BehaviorTreeError(RuntimeError):
    """Raised when a behavior-tree document or selection is invalid."""


@dataclass(frozen=True)
class ActionSpec:
    """One validated action plus its position in the tree."""

    task_id: str
    task_name: str
    location_id: str
    location_name: str
    behavior_id: str
    behavior_name: str
    action_id: str
    action_name: str
    action_type: str
    enabled: bool
    optional: bool
    parameters: Dict[str, Any]

    @property
    def path(self) -> str:
        return (
            f"{self.task_id}/{self.location_id}/"
            f"{self.behavior_id}/{self.action_id}"
        )


@dataclass(frozen=True)
class BehaviorTree:
    """A validated, flattened behavior-tree document."""

    source: Path
    execution_locked: bool
    actions: Sequence[ActionSpec]


@dataclass(frozen=True)
class TreeSelection:
    """Optional filters for each behavior-tree level."""

    task_id: Optional[str] = None
    location_id: Optional[str] = None
    behavior_id: Optional[str] = None
    action_id: Optional[str] = None


def _mapping(value: Any, path: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise BehaviorTreeError(f"{path} must be a mapping")
    return value


def _items(value: Any, path: str) -> List[Any]:
    if not isinstance(value, list) or not value:
        raise BehaviorTreeError(f"{path} must be a non-empty list")
    return value


def _text(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise BehaviorTreeError(f"{path} must be a non-empty string")
    return value.strip()


def _boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise BehaviorTreeError(f"{path} must be true or false")
    return value


def _number(
    value: Any, path: str, minimum: float, maximum: float
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BehaviorTreeError(f"{path} must be a number")
    number = float(value)
    if not math.isfinite(number) or number < minimum or number > maximum:
        raise BehaviorTreeError(
            f"{path} must be finite and in [{minimum}, {maximum}]"
        )
    return number


def _unique_ids(items: Iterable[Any], path: str) -> None:
    identifiers = set()
    for index, raw_item in enumerate(items):
        item = _mapping(raw_item, f"{path}[{index}]")
        identifier = _text(item.get("id"), f"{path}[{index}].id")
        if identifier in identifiers:
            raise BehaviorTreeError(
                f"{path} contains duplicate id {identifier!r}"
            )
        identifiers.add(identifier)


def _validate_move_absj(action: Dict[str, Any], path: str) -> Dict[str, Any]:
    raw_targets = _mapping(action.get("targets"), f"{path}.targets")
    targets: Dict[str, List[float]] = {}
    for side in ARM_NAMES:
        if side not in raw_targets:
            continue
        try:
            targets[side] = validate_joint_target(side, raw_targets[side])
        except RuntimeError as exc:
            raise BehaviorTreeError(f"{path}.targets: {exc}") from exc
    if not targets:
        raise BehaviorTreeError(
            f"{path}.targets must contain left and/or right"
        )
    return {"targets": targets}


def _validate_pose(value: Any, path: str) -> List[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise BehaviorTreeError(f"{path} must contain exactly 6 numbers")
    return [
        _number(number, f"{path}[{index}]", -100.0, 100.0)
        for index, number in enumerate(value)
    ]


def _validate_move_l(action: Dict[str, Any], path: str) -> Dict[str, Any]:
    raw_targets = _mapping(action.get("targets"), f"{path}.targets")
    targets: Dict[str, Dict[str, Any]] = {}
    for side in ARM_NAMES:
        if side not in raw_targets:
            continue
        raw_target = _mapping(
            raw_targets[side], f"{path}.targets.{side}"
        )
        targets[side] = {
            "pose": _validate_pose(
                raw_target.get("pose"), f"{path}.targets.{side}.pose"
            ),
            "elbow": _number(
                raw_target.get("elbow"),
                f"{path}.targets.{side}.elbow",
                -2.0 * math.pi,
                2.0 * math.pi,
            ),
        }
    if not targets:
        raise BehaviorTreeError(
            f"{path}.targets must contain left and/or right"
        )
    return {
        "targets": targets,
        "speed_mm_s": _number(
            action.get("speed_mm_s"),
            f"{path}.speed_mm_s",
            0.001,
            4000.0,
        ),
        "zone_mm": _number(
            action.get("zone_mm", 0.0),
            f"{path}.zone_mm",
            0.0,
            200.0,
        ),
        "response_timeout_s": _number(
            action.get("response_timeout_s", 70.0),
            f"{path}.response_timeout_s",
            1.0,
            600.0,
        ),
    }


def _validate_relative_translation(
    value: Any, path: str
) -> List[float]:
    offset = _mapping(value, path)
    return [
        _number(
            offset.get(axis, 0.0),
            f"{path}.{axis}",
            -1.0,
            1.0,
        )
        for axis in ("dx", "dy", "dz")
    ]


def _validate_absolute_orientation(
    value: Any, path: str
) -> Optional[List[Optional[float]]]:
    offset = _mapping(value, path)
    raw_orientation = offset.get("orientation")
    if raw_orientation is None:
        orientation = offset
        orientation_path = path
    else:
        orientation = _mapping(
            raw_orientation, f"{path}.orientation"
        )
        orientation_path = f"{path}.orientation"

    orientation_keys = ("rx", "ry", "rz")
    if not any(key in orientation for key in orientation_keys):
        return None
    return [
        (
            _number(
                orientation[key],
                f"{orientation_path}.{key}",
                -2.0 * math.pi,
                2.0 * math.pi,
            )
            if key in orientation
            else None
        )
        for key in orientation_keys
    ]


def _validate_move_l_relative(
    action: Dict[str, Any], path: str
) -> Dict[str, Any]:
    raw_offset = _mapping(
        action.get("relative_offset"), f"{path}.relative_offset"
    )
    rotate_with_waist = _boolean(
        raw_offset.get("rotate_with_waist", False),
        f"{path}.relative_offset.rotate_with_waist",
    )
    if rotate_with_waist:
        raise BehaviorTreeError(
            f"{path}.relative_offset.rotate_with_waist is unsupported "
            "because the Rokae arm has no waist frame"
        )

    offsets: Dict[str, List[float]] = {}
    orientations: Dict[str, List[Optional[float]]] = {}
    side_keys = {
        "left": ("left", "left_arm"),
        "right": ("right", "right_arm"),
    }
    has_side_key = any(
        key in raw_offset
        for aliases in side_keys.values()
        for key in aliases
    )
    if has_side_key:
        for side, aliases in side_keys.items():
            selected_key = next(
                (key for key in aliases if key in raw_offset), None
            )
            if selected_key is not None:
                side_value = raw_offset[selected_key]
                offsets[side] = _validate_relative_translation(
                    side_value,
                    f"{path}.relative_offset.{selected_key}",
                )
                orientation = _validate_absolute_orientation(
                    side_value,
                    f"{path}.relative_offset.{selected_key}",
                )
                if orientation is not None:
                    orientations[side] = orientation
    else:
        common = _validate_relative_translation(
            raw_offset, f"{path}.relative_offset"
        )
        offsets = {"left": common, "right": list(common)}
        orientation = _validate_absolute_orientation(
            raw_offset, f"{path}.relative_offset"
        )
        if orientation is not None:
            orientations = {
                "left": orientation,
                "right": list(orientation),
            }

    return {
        "offsets": offsets,
        "orientations": orientations,
        "speed_mm_s": _number(
            action.get("speed_mm_s"),
            f"{path}.speed_mm_s",
            0.001,
            4000.0,
        ),
        "zone_mm": _number(
            action.get("zone_mm", 0.0),
            f"{path}.zone_mm",
            0.0,
            200.0,
        ),
        "response_timeout_s": _number(
            action.get("response_timeout_s", 70.0),
            f"{path}.response_timeout_s",
            1.0,
            600.0,
        ),
    }


def _validate_wait(action: Dict[str, Any], path: str) -> Dict[str, Any]:
    return {
        "duration_s": _number(
            action.get("duration_s"),
            f"{path}.duration_s",
            0.0,
            3600.0,
        )
    }


def _validate_action(
    raw_action: Any,
    path: str,
    task: Dict[str, str],
    location: Dict[str, str],
    behavior: Dict[str, str],
) -> ActionSpec:
    action = _mapping(raw_action, path)
    action_id = _text(action.get("id"), f"{path}.id")
    action_name = _text(action.get("name", action_id), f"{path}.name")
    action_type = _text(action.get("type"), f"{path}.type")
    if action_type not in SUPPORTED_ACTION_TYPES:
        supported = ", ".join(sorted(SUPPORTED_ACTION_TYPES))
        raise BehaviorTreeError(
            f"{path}.type {action_type!r} is unsupported; use {supported}"
        )
    enabled = _boolean(action.get("enabled", True), f"{path}.enabled")
    optional = _boolean(action.get("optional", False), f"{path}.optional")

    # Disabled actions may intentionally contain placeholders in a template.
    parameters: Dict[str, Any] = {}
    if enabled:
        validators = {
            "move_absj": _validate_move_absj,
            "move_l": _validate_move_l,
            "move_l_relative": _validate_move_l_relative,
            "wait": _validate_wait,
        }
        parameters = validators[action_type](action, path)

    return ActionSpec(
        task_id=task["id"],
        task_name=task["name"],
        location_id=location["id"],
        location_name=location["name"],
        behavior_id=behavior["id"],
        behavior_name=behavior["name"],
        action_id=action_id,
        action_name=action_name,
        action_type=action_type,
        enabled=enabled,
        optional=optional,
        parameters=parameters,
    )


def load_behavior_tree(path: Path) -> BehaviorTree:
    """Load and completely validate one behavior-tree YAML file."""
    source = path.expanduser().resolve()
    try:
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
    except OSError as exc:
        raise BehaviorTreeError(f"cannot read {source}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise BehaviorTreeError(f"invalid YAML in {source}: {exc}") from exc

    root = _mapping(document, "root")
    version = root.get("version")
    if isinstance(version, bool) or version != 1:
        raise BehaviorTreeError("root.version must be 1")
    execution_locked = _boolean(
        root.get("execution_locked", True), "root.execution_locked"
    )

    raw_tasks = _items(root.get("tasks"), "root.tasks")
    _unique_ids(raw_tasks, "root.tasks")
    actions: List[ActionSpec] = []
    for task_index, raw_task in enumerate(raw_tasks):
        task_path = f"root.tasks[{task_index}]"
        task_mapping = _mapping(raw_task, task_path)
        task = {
            "id": _text(task_mapping.get("id"), f"{task_path}.id"),
            "name": _text(
                task_mapping.get("name", task_mapping.get("id")),
                f"{task_path}.name",
            ),
        }
        raw_locations = _items(
            task_mapping.get("locations"), f"{task_path}.locations"
        )
        _unique_ids(raw_locations, f"{task_path}.locations")
        for location_index, raw_location in enumerate(raw_locations):
            location_path = f"{task_path}.locations[{location_index}]"
            location_mapping = _mapping(raw_location, location_path)
            location = {
                "id": _text(
                    location_mapping.get("id"), f"{location_path}.id"
                ),
                "name": _text(
                    location_mapping.get(
                        "name", location_mapping.get("id")
                    ),
                    f"{location_path}.name",
                ),
            }
            raw_behaviors = _items(
                location_mapping.get("behaviors"),
                f"{location_path}.behaviors",
            )
            _unique_ids(raw_behaviors, f"{location_path}.behaviors")
            for behavior_index, raw_behavior in enumerate(raw_behaviors):
                behavior_path = (
                    f"{location_path}.behaviors[{behavior_index}]"
                )
                behavior_mapping = _mapping(raw_behavior, behavior_path)
                behavior = {
                    "id": _text(
                        behavior_mapping.get("id"), f"{behavior_path}.id"
                    ),
                    "name": _text(
                        behavior_mapping.get(
                            "name", behavior_mapping.get("id")
                        ),
                        f"{behavior_path}.name",
                    ),
                }
                raw_actions = _items(
                    behavior_mapping.get("actions"),
                    f"{behavior_path}.actions",
                )
                # Action IDs are reusable labels rather than mapping keys.
                # The list order is authoritative and is preserved below.
                for action_index, raw_action in enumerate(raw_actions):
                    action_path = (
                        f"{behavior_path}.actions[{action_index}]"
                    )
                    actions.append(
                        _validate_action(
                            raw_action,
                            action_path,
                            task,
                            location,
                            behavior,
                        )
                    )

    return BehaviorTree(
        source=source,
        execution_locked=execution_locked,
        actions=actions,
    )


def select_actions(
    tree: BehaviorTree, selection: TreeSelection
) -> List[ActionSpec]:
    """Return all matches in YAML order, including duplicate action IDs."""
    actions = [
        action
        for action in tree.actions
        if (
            selection.task_id is None
            or action.task_id == selection.task_id
        )
        and (
            selection.location_id is None
            or action.location_id == selection.location_id
        )
        and (
            selection.behavior_id is None
            or action.behavior_id == selection.behavior_id
        )
        and (
            selection.action_id is None
            or action.action_id == selection.action_id
        )
    ]
    if not actions:
        filters = [
            f"{name}={value!r}"
            for name, value in (
                ("task_id", selection.task_id),
                ("location_id", selection.location_id),
                ("behavior_id", selection.behavior_id),
                ("action_id", selection.action_id),
            )
            if value is not None
        ]
        description = ", ".join(filters) if filters else "no filters"
        raise BehaviorTreeError(
            f"selection matched no actions ({description})"
        )
    return actions


def describe_actions(actions: Sequence[ActionSpec]) -> str:
    """Create a stable dry-run summary."""
    lines = [f"Selected {len(actions)} action(s):"]
    for index, action in enumerate(actions, start=1):
        state = "enabled" if action.enabled else "DISABLED"
        optional = ", optional" if action.optional else ""
        lines.append(
            f"  {index}. {action.path}: {action.action_type} "
            f"({state}{optional}) - {action.action_name}"
        )
    return "\n".join(lines)


class BehaviorTreeExecutor:
    """Own behavior-tree validation, selection and execution flow."""

    def __init__(self, config: RunnerConfig) -> None:
        self.config = config

    def load_tree(self, path: Path) -> BehaviorTree:
        """Load and validate a YAML tree."""
        return load_behavior_tree(path)

    def execute_selected(
        self, tree: BehaviorTree, selection: TreeSelection
    ) -> int:
        """Preview or execute the selected subtree according to policy."""
        actions = select_actions(tree, selection)
        return self.execute_actions(tree, actions)

    def execute_actions(
        self, tree: BehaviorTree, actions: Sequence[ActionSpec]
    ) -> int:
        """Preview or execute one already ordered action plan."""
        planned_actions = list(actions)
        if not planned_actions:
            raise BehaviorTreeError("action plan must not be empty")

        print(f"Tree: {tree.source}")
        print(describe_actions(planned_actions))

        if not self.config.execute:
            print("Dry run only: no robot command was sent.")
            return 0
        if tree.execution_locked:
            print(
                "Execution blocked: set execution_locked: false only after "
                "replacing and checking all enabled targets.",
                file=sys.stderr,
            )
            return 2

        enabled_actions = [
            action for action in planned_actions if action.enabled
        ]
        if not enabled_actions:
            print("No enabled action selected; nothing to execute.")
            return 0
        if not self._confirm_execution(len(enabled_actions)):
            print("Cancelled; no robot command was sent.")
            return 0
        return self._run_actions(enabled_actions)

    def _confirm_execution(self, action_count: int) -> bool:
        if not self.config.require_confirmation:
            return True
        print(
            "EXECUTION ENABLED. Confirm targets use current robot feedback, "
            "the workspace is clear, and the E-stop is accessible."
        )
        phrase = self.config.confirmation_phrase
        confirmation = input(
            f"Type {phrase} to run {action_count} enabled action(s): "
        )
        return confirmation == phrase

    def _run_actions(self, actions: Sequence[ActionSpec]) -> int:
        rclpy.init()
        node: Optional[RokaeActionExecutor] = None
        try:
            node = RokaeActionExecutor()
            for index, action in enumerate(actions, start=1):
                node.get_logger().info(
                    f"[{index}/{len(actions)}] starting "
                    f"{action.path}: {action.action_name}"
                )
                try:
                    execute_behavior(node, action)
                except (ProgramError, ValueError) as exc:
                    should_continue = (
                        action.optional
                        or self.config.continue_on_action_fail
                    )
                    if should_continue:
                        node.get_logger().error(
                            f"action {action.path} failed: {exc}; continuing"
                        )
                        continue
                    raise
            node.get_logger().info(
                "selected behavior-tree actions completed"
            )
            return 0
        except KeyboardInterrupt:
            if node is not None:
                node.get_logger().warning(
                    "Ctrl+C received; cancelling active MoveAbsJ goals"
                )
                node.cancel_active()
            return 130
        except (ProgramError, ValueError) as exc:
            if node is not None:
                node.get_logger().error(str(exc))
                node.cancel_active()
            else:
                print(f"Execution error: {exc}", file=sys.stderr)
            return 1
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
