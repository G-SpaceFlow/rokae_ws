"""Load, validate, select and execute a Rokae behavior-tree YAML file."""

from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
import yaml

from .bt_actions import BEHAVIORS, execute_behavior
from .bt_runner_config import RunnerConfig
from .executor_services import RokaeActionExecutor
from .moveabsj_action_client import ProgramError, validate_joint_target
from .vision_motion import quaternion_to_rpy


SUPPORTED_ACTION_TYPES = set(BEHAVIORS)
ARM_NAMES = ("left", "right")
HAND_COMMANDS = {
    "open",
    "half",
    "close",
    "position",
    "motors",
    "joints",
    "speed",
    "pressure",
}


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
    if any(key in orientation for key in orientation_keys):
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

    quaternion_keys = (
        ("x", "y", "z", "w"),
        ("ox", "oy", "oz", "ow"),
    )
    for keys in quaternion_keys:
        if not any(key in orientation for key in keys):
            continue
        if not all(key in orientation for key in keys):
            raise BehaviorTreeError(
                f"{orientation_path} quaternion must contain "
                + ", ".join(keys)
            )
        quaternion = [
            _number(
                orientation[key],
                f"{orientation_path}.{key}",
                -1.0,
                1.0,
            )
            for key in keys
        ]
        try:
            return list(quaternion_to_rpy(*quaternion))
        except ValueError as exc:
            raise BehaviorTreeError(
                f"{orientation_path}: {exc}"
            ) from exc
    return None


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


def _integer(
    value: Any, path: str, minimum: int, maximum: int
) -> int:
    number = _number(value, path, float(minimum), float(maximum))
    if not number.is_integer():
        raise BehaviorTreeError(f"{path} must be an integer")
    return int(number)


def _text_list(value: Any, path: str) -> List[str]:
    if not isinstance(value, list):
        raise BehaviorTreeError(f"{path} must be a list")
    return [
        _text(item, f"{path}[{index}]")
        for index, item in enumerate(value)
    ]


def _validate_vision_source(
    value: Any, path: str, *, default_key: Optional[str] = None
) -> Dict[str, Any]:
    source = _mapping(value, path)
    key_value = source.get(
        "base_key", source.get("key", source.get("cache_key", default_key))
    )
    key = _text(key_value, f"{path}.base_key")
    raw_points = source.get(
        "points", source.get("point_names", source.get("point"))
    )
    if isinstance(raw_points, str):
        raw_points = [raw_points]
    points = _text_list(raw_points, f"{path}.points")
    if len(points) != 1:
        raise BehaviorTreeError(
            f"{path}.points must contain exactly one point name"
        )
    return {"key": key, "points": points}


def _validate_vision_action(
    action: Dict[str, Any], path: str
) -> Dict[str, Any]:
    mode = _integer(
        action.get("motion_mode", 1),
        f"{path}.motion_mode",
        1,
        6,
    )
    raw_points = action.get("points", action.get("point_names"))
    if isinstance(raw_points, str):
        raw_points = [raw_points]
    points = _text_list(raw_points, f"{path}.points")
    if mode == 1 and len(points) < 2:
        raise BehaviorTreeError(
            f"{path}.points needs at least two point names for "
            "motion_mode 1"
        )
    if mode != 1 and len(points) != 1:
        raise BehaviorTreeError(
            f"{path}.points must contain exactly one point name for "
            f"motion_mode {mode}"
        )

    raw_labels = action.get("labels", action.get("label_filter", []))
    if isinstance(raw_labels, str):
        raw_labels = [raw_labels]
    labels = _text_list(raw_labels, f"{path}.labels")
    parameters = {
        "key": _text(action.get("key"), f"{path}.key"),
        "trigger_value": _integer(
            action.get("trigger_value", 2),
            f"{path}.trigger_value",
            0,
            100,
        ),
        "labels": labels,
        "motion_mode": mode,
        "point_names": points,
        "response_timeout_s": _number(
            action.get("response_timeout_s", 10.0),
            f"{path}.response_timeout_s",
            1.0,
            120.0,
        ),
    }
    if "offset_id" in action:
        parameters["offset_id"] = _text(
            action["offset_id"], f"{path}.offset_id"
        )
    return parameters


def _reject_waist_rotation(value: Dict[str, Any], path: str) -> None:
    for key in (
        "rotate_with_waist",
        "rotate_orientation_with_waist",
    ):
        if key not in value:
            continue
        enabled = _boolean(value[key], f"{path}.{key}")
        if enabled:
            raise BehaviorTreeError(
                f"{path}.{key} is unsupported; this Rokae workflow "
                "does not include a waist"
            )


def _relative_side_values(
    raw_offset: Dict[str, Any],
    path: str,
    *,
    single_side_default: bool = False,
) -> Tuple[
    Dict[str, List[float]],
    Dict[str, List[Optional[float]]],
]:
    _reject_waist_rotation(raw_offset, path)
    offsets: Dict[str, List[float]] = {}
    orientations: Dict[str, List[Optional[float]]] = {}
    aliases = {
        "left": ("left", "left_arm"),
        "right": ("right", "right_arm"),
    }
    selected = {
        side: next(
            (key for key in keys if key in raw_offset), None
        )
        for side, keys in aliases.items()
    }
    if any(key is not None for key in selected.values()):
        for side, key in selected.items():
            if key is None:
                continue
            offsets[side] = _validate_relative_translation(
                raw_offset[key], f"{path}.{key}"
            )
            orientation = _validate_absolute_orientation(
                raw_offset[key], f"{path}.{key}"
            )
            if orientation is not None:
                orientations[side] = orientation
        return offsets, orientations

    common = _validate_relative_translation(raw_offset, path)
    orientation = _validate_absolute_orientation(raw_offset, path)
    if single_side_default:
        side = str(
            raw_offset.get("hand", raw_offset.get("arm", "left"))
        ).strip().lower()
        side = "right" if side in ("right", "right_arm", "r") else "left"
        offsets[side] = common
        if orientation is not None:
            orientations[side] = orientation
    else:
        offsets = {"left": common, "right": list(common)}
        if orientation is not None:
            orientations = {
                "left": orientation,
                "right": list(orientation),
            }
    return offsets, orientations


def _translation_overrides(
    value: Any, path: str
) -> List[Optional[float]]:
    offset = _mapping(value, path)
    return [
        (
            _number(
                offset[axis],
                f"{path}.{axis}",
                -1.0,
                1.0,
            )
            if axis in offset
            else None
        )
        for axis in ("dx", "dy", "dz")
    ]


def _visual_delta_overrides(
    raw_offset: Dict[str, Any], path: str
) -> Dict[str, List[Optional[float]]]:
    _reject_waist_rotation(raw_offset, path)
    common = _translation_overrides(raw_offset, path)
    result = {"left": list(common), "right": list(common)}
    for side, aliases in {
        "left": ("left", "left_arm"),
        "right": ("right", "right_arm"),
    }.items():
        key = next(
            (name for name in aliases if name in raw_offset), None
        )
        if key is None:
            continue
        side_values = _translation_overrides(
            raw_offset[key], f"{path}.{key}"
        )
        result[side] = [
            (
                side_values[index]
                if side_values[index] is not None
                else common[index]
            )
            for index in range(3)
        ]
    return result


def _visual_orientations(
    raw_offset: Dict[str, Any], path: str
) -> Dict[str, List[Optional[float]]]:
    _, orientations = _relative_side_values(raw_offset, path)
    return orientations


def _validate_skip_threshold(
    action: Dict[str, Any],
    raw_offset: Dict[str, Any],
    path: str,
) -> Tuple[Optional[float], List[int]]:
    config = action.get(
        "skip_if_distance_less_than",
        raw_offset.get(
            "skip_if_distance_less_than",
            action.get("distance_threshold"),
        ),
    )
    if config is None:
        return None, [0, 1]
    if isinstance(config, dict):
        threshold = _number(
            config.get("threshold"),
            f"{path}.skip_if_distance_less_than.threshold",
            0.0,
            10.0,
        )
        raw_axes = config.get("axes", config.get("axis", ["x", "y"]))
    else:
        threshold = _number(
            config,
            f"{path}.skip_if_distance_less_than",
            0.0,
            10.0,
        )
        raw_axes = ["x", "y"]
    if isinstance(raw_axes, str):
        raw_axes = [raw_axes]
    axes = _text_list(
        raw_axes, f"{path}.skip_if_distance_less_than.axes"
    )
    axis_map = {"x": 0, "y": 1, "z": 2}
    if not axes or any(axis.lower() not in axis_map for axis in axes):
        raise BehaviorTreeError(
            f"{path}.skip_if_distance_less_than.axes must contain "
            "x, y and/or z"
        )
    return threshold, [axis_map[axis.lower()] for axis in axes]


def _validate_visual_motion_options(
    action: Dict[str, Any], path: str
) -> Dict[str, float]:
    return {
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


def _validate_move_l_vision(
    action: Dict[str, Any], path: str
) -> Dict[str, Any]:
    mode = _integer(
        action.get("motion_mode"),
        f"{path}.motion_mode",
        1,
        6,
    )
    parameters: Dict[str, Any] = {"motion_mode": mode}
    raw_offset = _mapping(
        action.get("relative_offset", {}),
        f"{path}.relative_offset",
    )
    _reject_waist_rotation(raw_offset, f"{path}.relative_offset")

    if mode in (1, 2, 3):
        source = {
            "base_key": action.get("base_key"),
            "points": action.get("points", action.get("point_names")),
        }
        source_path = f"{path}.source"
        source_mapping = _mapping(source, source_path)
        key = _text(
            source_mapping.get("base_key"),
            f"{path}.base_key",
        )
        raw_points = source_mapping.get("points")
        if isinstance(raw_points, str):
            raw_points = [raw_points]
        points = _text_list(raw_points, f"{path}.points")
        required_count = 2 if mode == 1 else 1
        if (
            (mode == 1 and len(points) < required_count)
            or (mode != 1 and len(points) != required_count)
        ):
            raise BehaviorTreeError(
                f"{path}.points has the wrong count for motion_mode {mode}"
            )
        parameters["source"] = {
            "key": key,
            "points": points[:2] if mode == 1 else points,
        }

    if mode == 2:
        midpoint_value = raw_offset
        midpoint_path = f"{path}.relative_offset"
        for key in ("midpoint", "center", "left", "left_arm"):
            if key in raw_offset:
                midpoint_value = raw_offset[key]
                midpoint_path = f"{path}.relative_offset.{key}"
                break
        parameters["midpoint_offset"] = _validate_relative_translation(
            midpoint_value, midpoint_path
        )
        parameters["orientations"] = _visual_orientations(
            raw_offset, f"{path}.relative_offset"
        )
    elif mode == 3:
        offsets, orientations = _relative_side_values(
            raw_offset,
            f"{path}.relative_offset",
            single_side_default=True,
        )
        if not offsets:
            raise BehaviorTreeError(
                f"{path}.relative_offset must select at least one arm"
            )
        parameters["offsets"] = offsets
        parameters["orientations"] = orientations
    elif mode in (4, 5):
        point_delta = _mapping(
            action.get("point_delta"), f"{path}.point_delta"
        )
        source_names = (
            ("target", "current")
            if mode == 4
            else (
                "left_current",
                "right_current",
                "left_target",
                "right_target",
            )
        )
        parameters["point_sources"] = {
            name: _validate_vision_source(
                point_delta.get(name),
                f"{path}.point_delta.{name}",
            )
            for name in source_names
        }
        parameters["delta_overrides"] = _visual_delta_overrides(
            raw_offset, f"{path}.relative_offset"
        )
        parameters["orientations"] = _visual_orientations(
            raw_offset, f"{path}.relative_offset"
        )
        threshold, axes = _validate_skip_threshold(
            action, raw_offset, path
        )
        parameters["skip_threshold"] = threshold
        parameters["skip_axes"] = axes
    elif mode == 6:
        mode_source = _mapping(
            action.get("mode_source"), f"{path}.mode_source"
        )
        parameters["scalar_source"] = _validate_vision_source(
            mode_source, f"{path}.mode_source"
        )
        raw_axis = mode_source.get(
            "index", mode_source.get("axis", 1)
        )
        axis_aliases = {
            "0": 0,
            "x": 0,
            "dx": 0,
            "1": 1,
            "y": 1,
            "dy": 1,
            "2": 2,
            "z": 2,
            "dz": 2,
        }
        axis_key = str(raw_axis).strip().lower()
        if axis_key not in axis_aliases:
            raise BehaviorTreeError(
                f"{path}.mode_source.index must be 0/dx, 1/dy "
                "or 2/dz"
            )
        parameters["axis_index"] = axis_aliases[axis_key]
        parameters["initial_point"] = _number(
            mode_source.get("initial_point", 0.0),
            f"{path}.mode_source.initial_point",
            -10.0,
            10.0,
        )
        raw_hands = mode_source.get("hands", ["left", "right"])
        if isinstance(raw_hands, str):
            raw_hands = (
                ["left", "right"]
                if raw_hands.lower() in ("both", "all", "dual")
                else [raw_hands]
            )
        parameters["hands"] = _hand_sides(
            raw_hands, f"{path}.mode_source.hands"
        )
        offsets, orientations = _relative_side_values(
            raw_offset, f"{path}.relative_offset"
        )
        parameters["offsets"] = {
            side: offsets.get(side, [0.0, 0.0, 0.0])
            for side in ARM_NAMES
        }
        parameters["orientations"] = orientations
        threshold, _ = _validate_skip_threshold(
            mode_source,
            raw_offset,
            f"{path}.mode_source",
        )
        parameters["skip_threshold"] = threshold

    parameters.update(_validate_visual_motion_options(action, path))
    return parameters


def _hand_byte(value: Any, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BehaviorTreeError(
            f"{path} must be an integer in [0, 255]"
        )
    number = float(value)
    if (
        not math.isfinite(number)
        or not number.is_integer()
        or number < 0
        or number > 255
    ):
        raise BehaviorTreeError(
            f"{path} must be an integer in [0, 255]"
        )
    return int(number)


def _hand_motor_values(value: Any, path: str) -> List[int]:
    if not isinstance(value, list) or len(value) != 6:
        raise BehaviorTreeError(
            f"{path} must contain exactly 6 motor bytes"
        )
    return [
        _hand_byte(number, f"{path}[{index}]")
        for index, number in enumerate(value)
    ]


def _hand_sides(value: Any, path: str) -> List[str]:
    if not isinstance(value, list) or not value:
        raise BehaviorTreeError(
            f"{path} must be a non-empty list containing left and/or right"
        )
    sides: List[str] = []
    for index, raw_side in enumerate(value):
        side = _text(raw_side, f"{path}[{index}]").lower()
        if side not in ARM_NAMES:
            raise BehaviorTreeError(
                f"{path}[{index}] must be left or right"
            )
        if side in sides:
            raise BehaviorTreeError(
                f"{path} contains duplicate side {side!r}"
            )
        sides.append(side)
    return sides


def _validate_hand(action: Dict[str, Any], path: str) -> Dict[str, Any]:
    has_q = "q" in action
    has_targets = "targets" in action
    if has_q and has_targets:
        raise BehaviorTreeError(
            f"{path} must use q or targets, not both"
        )

    if has_q:
        command = _text(
            action.get("command", "motors"), f"{path}.command"
        ).lower()
        if command not in ("motors", "joints"):
            raise BehaviorTreeError(
                f"{path}.q is only valid with command motors or joints"
            )
        raw_q = action["q"]
        if not isinstance(raw_q, list) or len(raw_q) != 12:
            raise BehaviorTreeError(
                f"{path}.q must contain exactly 12 bytes: "
                "left M1-M6 followed by right M1-M6"
            )
        q = [
            _hand_byte(number, f"{path}.q[{index}]")
            for index, number in enumerate(raw_q)
        ]
        requests = {
            "left": {"command": "motors", "values": q[:6]},
            "right": {"command": "motors", "values": q[6:]},
        }
    else:
        command = _text(
            action.get("command", "motors"), f"{path}.command"
        ).lower()
        if command not in HAND_COMMANDS:
            supported = ", ".join(sorted(HAND_COMMANDS))
            raise BehaviorTreeError(
                f"{path}.command {command!r} is unsupported; "
                f"use {supported}"
            )

        requests: Dict[str, Dict[str, Any]] = {}
        if command in ("open", "half", "close", "pressure"):
            if has_targets:
                raise BehaviorTreeError(
                    f"{path}.targets is not used by command {command}"
                )
            sides = _hand_sides(
                action.get("hands", list(ARM_NAMES)), f"{path}.hands"
            )
            requests = {
                side: {"command": command, "values": [0] * 6}
                for side in sides
            }
        else:
            raw_targets = _mapping(
                action.get("targets"), f"{path}.targets"
            )
            unknown_sides = set(raw_targets) - set(ARM_NAMES)
            if unknown_sides:
                unknown = ", ".join(sorted(str(x) for x in unknown_sides))
                raise BehaviorTreeError(
                    f"{path}.targets contains unsupported side(s): "
                    f"{unknown}"
                )
            for side in ARM_NAMES:
                if side not in raw_targets:
                    continue
                target_path = f"{path}.targets.{side}"
                if command in ("position", "speed"):
                    value = _hand_byte(raw_targets[side], target_path)
                    values = [value, 0, 0, 0, 0, 0]
                else:
                    values = _hand_motor_values(
                        raw_targets[side], target_path
                    )
                requests[side] = {
                    "command": (
                        "motors" if command == "joints" else command
                    ),
                    "values": values,
                }
            if not requests:
                raise BehaviorTreeError(
                    f"{path}.targets must contain left and/or right"
                )

    return {
        "requests": requests,
        "response_timeout_s": _number(
            action.get("response_timeout_s", 5.0),
            f"{path}.response_timeout_s",
            1.0,
            60.0,
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
            "hand": _validate_hand,
            "move_absj": _validate_move_absj,
            "move_l": _validate_move_l,
            "move_l_relative": _validate_move_l_relative,
            "move_l_vision": _validate_move_l_vision,
            "vision": _validate_vision_action,
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
            if self.config.initialize_before_execute:
                node.get_logger().warning(
                    "initializing and powering on both arms before workflow"
                )
                node.initialize_robots()
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
