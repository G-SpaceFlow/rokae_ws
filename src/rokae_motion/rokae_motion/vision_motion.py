"""Pure motion-mode calculations for cached vision targets."""

from dataclasses import dataclass
import math
from typing import Callable, Dict, Optional, Sequence, Tuple


ARM_NAMES = ("left", "right")


@dataclass(frozen=True)
class CartesianTarget:
    """Position plus sparse absolute XYZ Euler orientation."""

    position: Tuple[float, float, float]
    orientation_rpy: Tuple[
        Optional[float], Optional[float], Optional[float]
    ] = (None, None, None)


@dataclass(frozen=True)
class ResolvedVisionMotion:
    """Targets produced by one motion_mode calculation."""

    targets: Dict[str, CartesianTarget]
    skipped_reason: Optional[str] = None


def quaternion_to_rpy(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float]:
    """Convert a normalized or non-normalized quaternion to XYZ Euler."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        raise ValueError("orientation quaternion must be non-zero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def pose_message_to_target(pose) -> CartesianTarget:
    """Convert geometry_msgs/Pose-like data to a complete target."""
    return CartesianTarget(
        position=(
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ),
        orientation_rpy=quaternion_to_rpy(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ),
    )


def current_pose_to_target(values: Sequence[float]) -> CartesianTarget:
    """Convert [x, y, z, rx, ry, rz] feedback to a sparse target seed."""
    if len(values) != 6:
        raise ValueError("current Cartesian state must contain 6 values")
    return CartesianTarget(
        position=tuple(float(value) for value in values[:3]),
        orientation_rpy=(None, None, None),
    )


def _add(
    first: Sequence[float], second: Sequence[float]
) -> Tuple[float, float, float]:
    return tuple(
        float(first[index]) + float(second[index])
        for index in range(3)
    )


def _subtract(
    first: Sequence[float], second: Sequence[float]
) -> Tuple[float, float, float]:
    return tuple(
        float(first[index]) - float(second[index])
        for index in range(3)
    )


def _midpoint(
    first: Sequence[float], second: Sequence[float]
) -> Tuple[float, float, float]:
    return tuple(
        (float(first[index]) + float(second[index])) * 0.5
        for index in range(3)
    )


def _orientation_for(
    parameters: dict, side: str
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    values = parameters.get("orientations", {}).get(side)
    if values is None:
        return None, None, None
    return tuple(values)


def _apply_axis_overrides(
    computed: Sequence[float], overrides: Sequence[Optional[float]]
) -> Tuple[float, float, float]:
    return tuple(
        (
            float(overrides[index])
            if overrides[index] is not None
            else float(computed[index])
        )
        for index in range(3)
    )


def _distance(
    values: Sequence[float], axes: Sequence[int]
) -> float:
    return math.sqrt(
        sum(float(values[index]) ** 2 for index in axes)
    )


TargetFetcher = Callable[
    [dict, int], Tuple[CartesianTarget, CartesianTarget]
]


def resolve_vision_motion(
    parameters: dict,
    current: Dict[str, CartesianTarget],
    fetch_target: TargetFetcher,
) -> ResolvedVisionMotion:
    """Resolve reference-project motion_mode 1 through 6 semantics."""
    mode = int(parameters["motion_mode"])
    if mode == 1:
        left, right = fetch_target(parameters["source"], 1)
        return ResolvedVisionMotion(
            targets={"left": left, "right": right}
        )

    if mode == 2:
        visual, _ = fetch_target(parameters["source"], 2)
        current_midpoint = _midpoint(
            current["left"].position, current["right"].position
        )
        target_midpoint = _add(
            visual.position, parameters["midpoint_offset"]
        )
        delta = _subtract(target_midpoint, current_midpoint)
        return ResolvedVisionMotion(
            targets={
                side: CartesianTarget(
                    position=_add(current[side].position, delta),
                    orientation_rpy=_orientation_for(
                        parameters, side
                    ),
                )
                for side in ARM_NAMES
            }
        )

    if mode == 3:
        visual, _ = fetch_target(parameters["source"], 3)
        return ResolvedVisionMotion(
            targets={
                side: CartesianTarget(
                    position=_add(
                        visual.position,
                        parameters["offsets"][side],
                    ),
                    orientation_rpy=_orientation_for(
                        parameters, side
                    ),
                )
                for side in parameters["offsets"]
            }
        )

    if mode == 4:
        target_point, _ = fetch_target(
            parameters["point_sources"]["target"], 4
        )
        current_point, _ = fetch_target(
            parameters["point_sources"]["current"], 4
        )
        computed = _subtract(
            target_point.position, current_point.position
        )
        threshold = parameters.get("skip_threshold")
        axes = parameters.get("skip_axes", [0, 1])
        distance = _distance(computed, axes)
        if threshold is not None and distance < threshold:
            return ResolvedVisionMotion(
                targets={},
                skipped_reason=(
                    f"motion_mode=4 distance {distance:.4f} m "
                    f"is below threshold {threshold:.4f} m"
                ),
            )
        return ResolvedVisionMotion(
            targets={
                side: CartesianTarget(
                    position=_add(
                        current[side].position,
                        _apply_axis_overrides(
                            computed,
                            parameters["delta_overrides"][side],
                        ),
                    ),
                    orientation_rpy=_orientation_for(
                        parameters, side
                    ),
                )
                for side in ARM_NAMES
            }
        )

    if mode == 5:
        sources = parameters["point_sources"]
        left_current, _ = fetch_target(
            sources["left_current"], 5
        )
        right_current, _ = fetch_target(
            sources["right_current"], 5
        )
        left_target, _ = fetch_target(
            sources["left_target"], 5
        )
        right_target, _ = fetch_target(
            sources["right_target"], 5
        )
        computed = {
            "left": _subtract(
                left_target.position, left_current.position
            ),
            "right": _subtract(
                right_target.position, right_current.position
            ),
        }
        threshold = parameters.get("skip_threshold")
        axes = parameters.get("skip_axes", [0, 1])
        distances = {
            side: _distance(computed[side], axes)
            for side in ARM_NAMES
        }
        if (
            threshold is not None
            and all(distance < threshold for distance in distances.values())
        ):
            return ResolvedVisionMotion(
                targets={},
                skipped_reason=(
                    "motion_mode=5 left/right distances "
                    f"{distances['left']:.4f}/{distances['right']:.4f} m "
                    f"are below threshold {threshold:.4f} m"
                ),
            )
        return ResolvedVisionMotion(
            targets={
                side: CartesianTarget(
                    position=_add(
                        current[side].position,
                        _apply_axis_overrides(
                            computed[side],
                            parameters["delta_overrides"][side],
                        ),
                    ),
                    orientation_rpy=_orientation_for(
                        parameters, side
                    ),
                )
                for side in ARM_NAMES
            }
        )

    if mode == 6:
        scalar, _ = fetch_target(parameters["scalar_source"], 6)
        raw_value = float(scalar.position[0])
        correction = raw_value - float(parameters["initial_point"])
        threshold = parameters.get("skip_threshold")
        if threshold is not None and abs(correction) < threshold:
            return ResolvedVisionMotion(
                targets={},
                skipped_reason=(
                    f"motion_mode=6 correction {correction:.4f} m "
                    f"is below threshold {threshold:.4f} m"
                ),
            )

        axis = int(parameters["axis_index"])
        hands = set(parameters["hands"])
        targets: Dict[str, CartesianTarget] = {}
        for side in ARM_NAMES:
            offset = list(parameters["offsets"][side])
            if side in hands:
                offset[axis] += correction
            targets[side] = CartesianTarget(
                position=_add(current[side].position, offset),
                orientation_rpy=_orientation_for(parameters, side),
            )
        return ResolvedVisionMotion(targets=targets)

    raise ValueError(f"unsupported vision motion_mode {mode}")
