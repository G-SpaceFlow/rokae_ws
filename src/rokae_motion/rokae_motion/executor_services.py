"""Low-level ROS 2 action/service calls used by behavior implementations."""

from pathlib import Path
from typing import Any, Dict, List

from ament_index_python.packages import get_package_share_directory
from rokae_interfaces.srv import ControlHand
from rokae_interfaces.srv import GetCartesianState
from rokae_interfaces.srv import GetVisionTarget
from rokae_interfaces.srv import MoveL
from rokae_interfaces.srv import MoveLRelative
from rokae_interfaces.srv import MoveLTarget
from rokae_interfaces.srv import SetVisionOffset
from std_srvs.srv import Trigger
import yaml

from .moveabsj_action_client import (
    DualMoveAbsJClient,
    ProgramError,
)
from .vision_motion import (
    current_pose_to_target,
    pose_message_to_target,
    resolve_vision_motion,
)


MOVE_L_SERVICE_NAMES = {
    "left": "/left_arm/move_l",
    "right": "/right_arm/move_l",
}
MOVE_L_RELATIVE_SERVICE_NAMES = {
    "left": "/left_arm/move_l_relative",
    "right": "/right_arm/move_l_relative",
}
MOVE_L_TARGET_SERVICE_NAMES = {
    "left": "/left_arm/move_l_target",
    "right": "/right_arm/move_l_target",
}
CARTESIAN_STATE_SERVICE_NAMES = {
    "left": "/left_arm/get_cartesian_state",
    "right": "/right_arm/get_cartesian_state",
}
HAND_SERVICE_NAMES = {
    "left": "/left_arm/control_hand",
    "right": "/right_arm/control_hand",
}
INITIALIZE_ROBOTS_SERVICE_NAME = "/initialize_robots"
TRIGGER_VISION_SERVICE_NAME = "/bt_target_server/trigger_detect"
GET_VISION_TARGET_SERVICE_NAME = "/bt_target_server/get_target"
SET_VISION_OFFSET_SERVICE_NAME = "/bt_target_server/set_offset"


class RokaeActionExecutor(DualMoveAbsJClient):
    """Expose small ROS API wrappers without YAML or tree knowledge."""

    def __init__(self) -> None:
        super().__init__()
        self._move_l_clients = {
            side: self.create_client(MoveL, service_name)
            for side, service_name in MOVE_L_SERVICE_NAMES.items()
        }
        self._move_l_relative_clients = {
            side: self.create_client(MoveLRelative, service_name)
            for side, service_name
            in MOVE_L_RELATIVE_SERVICE_NAMES.items()
        }
        self._move_l_target_clients = {
            side: self.create_client(MoveLTarget, service_name)
            for side, service_name in MOVE_L_TARGET_SERVICE_NAMES.items()
        }
        self._cartesian_state_clients = {
            side: self.create_client(GetCartesianState, service_name)
            for side, service_name
            in CARTESIAN_STATE_SERVICE_NAMES.items()
        }
        self._hand_clients = {
            side: self.create_client(ControlHand, service_name)
            for side, service_name in HAND_SERVICE_NAMES.items()
        }
        self._initialize_robots_client = self.create_client(
            Trigger, INITIALIZE_ROBOTS_SERVICE_NAME
        )
        self._trigger_vision_client = self.create_client(
            GetVisionTarget, TRIGGER_VISION_SERVICE_NAME
        )
        self._get_vision_target_client = self.create_client(
            GetVisionTarget, GET_VISION_TARGET_SERVICE_NAME
        )
        self._set_vision_offset_client = self.create_client(
            SetVisionOffset, SET_VISION_OFFSET_SERVICE_NAME
        )
        self._vision_offsets = self._load_vision_offsets()

    @staticmethod
    def _load_vision_offsets() -> Dict[str, Dict[str, Any]]:
        """Load stable calibration entries used by vision actions."""
        try:
            config_path = (
                Path(get_package_share_directory("rokae_motion"))
                / "config"
                / "vision_offsets.yaml"
            )
            document = yaml.safe_load(
                config_path.read_text(encoding="utf-8")
            ) or {}
            return {
                str(entry["id"]): entry
                for entry in document.get("offsets", [])
            }
        except (KeyError, OSError, TypeError, yaml.YAMLError) as exc:
            raise ProgramError(
                f"cannot load vision_offsets.yaml: {exc}"
            ) from exc

    def move_absj(
        self, action_name: str, targets: Dict[str, List[float]]
    ) -> None:
        """Send one left, right or dual-arm MoveAbsJ step."""
        self.run_step({"name": action_name, **targets})

    def initialize_robots(self) -> None:
        """Power on both arms through the explicit op.cpp-style service."""
        if not self._initialize_robots_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise ProgramError(
                f"{INITIALIZE_ROBOTS_SERVICE_NAME} is unavailable; start "
                "ros_robot_initializer_service first"
            )

        future = self._initialize_robots_client.call_async(
            Trigger.Request()
        )
        self.spin_until_all_done([future], timeout_s=30.0)
        try:
            response = future.result()
        except Exception as exc:
            raise ProgramError(
                f"robot initialization service failed: {exc}"
            ) from exc
        if response is None:
            raise ProgramError(
                "robot initialization service returned no response"
            )
        if not response.success:
            raise ProgramError(
                f"robot initialization failed: {response.message}"
            )
        self.get_logger().info(response.message)

    def _set_vision_offset(self, offset_id: str) -> None:
        try:
            entry = self._vision_offsets[offset_id]
        except KeyError as exc:
            available = ", ".join(sorted(self._vision_offsets))
            raise ProgramError(
                f"unknown vision offset_id {offset_id!r}; "
                f"available: {available}"
            ) from exc
        if not self._set_vision_offset_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise ProgramError(
                f"{SET_VISION_OFFSET_SERVICE_NAME} is unavailable; "
                "start vision_target_server first"
            )

        request = SetVisionOffset.Request()
        request.offset_id = offset_id
        for field in (
            "left_dx",
            "left_dy",
            "left_dz",
            "right_dx",
            "right_dy",
            "right_dz",
            "left_ox",
            "left_oy",
            "left_oz",
            "left_ow",
            "right_ox",
            "right_oy",
            "right_oz",
            "right_ow",
        ):
            setattr(request, field, float(entry.get(field, 0.0)))
        future = self._set_vision_offset_client.call_async(request)
        self.spin_until_all_done([future], timeout_s=10.0)
        response = future.result()
        if response is None or not response.success:
            message = (
                "no response" if response is None else response.message
            )
            raise ProgramError(
                f"cannot activate vision offset {offset_id!r}: {message}"
            )
        self.get_logger().info(response.message)

    def trigger_vision(self, parameters: Dict[str, Any]) -> None:
        """Trigger the detector and store its result in bt_target_server."""
        offset_id = parameters.get("offset_id")
        if offset_id:
            self._set_vision_offset(offset_id)
        if not self._trigger_vision_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise ProgramError(
                f"{TRIGGER_VISION_SERVICE_NAME} is unavailable; start "
                "vision_target_server first"
            )

        request = GetVisionTarget.Request()
        request.key = parameters["key"]
        request.trigger_value = parameters["trigger_value"]
        request.labels = parameters["labels"]
        request.motion_mode = parameters["motion_mode"]
        request.point_names = parameters["point_names"]
        future = self._trigger_vision_client.call_async(request)
        self.spin_until_all_done(
            [future], timeout_s=parameters["response_timeout_s"]
        )
        response = future.result()
        if response is None:
            raise ProgramError("vision trigger returned no response")
        if not response.success:
            raise ProgramError(
                f"vision trigger failed: {response.message}"
            )
        self.get_logger().info(
            f"vision result cached: key={parameters['key']}, "
            f"mode={parameters['motion_mode']}, "
            f"points={parameters['point_names']}"
        )

    def _get_vision_target(self, source: Dict[str, Any], mode: int):
        if not self._get_vision_target_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise ProgramError(
                f"{GET_VISION_TARGET_SERVICE_NAME} is unavailable; start "
                "vision_target_server first"
            )
        request = GetVisionTarget.Request()
        request.key = source["key"]
        request.motion_mode = mode
        request.point_names = source["points"]
        future = self._get_vision_target_client.call_async(request)
        self.spin_until_all_done([future], timeout_s=10.0)
        response = future.result()
        if response is None:
            raise ProgramError(
                f"vision target {source['key']!r} returned no response"
            )
        if not response.success:
            raise ProgramError(response.message)
        try:
            return (
                pose_message_to_target(response.left_pose),
                pose_message_to_target(response.right_pose),
            )
        except ValueError as exc:
            raise ProgramError(
                f"invalid cached vision orientation: {exc}"
            ) from exc

    def _get_cartesian_states(self):
        for side, client in self._cartesian_state_clients.items():
            if not client.wait_for_service(timeout_sec=5.0):
                raise ProgramError(
                    f"{CARTESIAN_STATE_SERVICE_NAMES[side]} is "
                    "unavailable; start ros_movel_service first"
                )
        futures = {
            side: client.call_async(GetCartesianState.Request())
            for side, client in self._cartesian_state_clients.items()
        }
        self.spin_until_all_done(list(futures.values()), timeout_s=10.0)
        states = {}
        for side, future in futures.items():
            response = future.result()
            if response is None:
                raise ProgramError(
                    f"{side} Cartesian-state service returned no response"
                )
            if not response.success:
                raise ProgramError(
                    f"{side} Cartesian-state read failed: "
                    f"{response.message}"
                )
            states[side] = current_pose_to_target(response.pose)
        return states

    def move_l_vision(self, parameters: Dict[str, Any]) -> None:
        """Resolve and execute cached-vision motion_mode 1 through 6."""
        mode = parameters["motion_mode"]
        current = (
            {}
            if mode in (1, 3)
            else self._get_cartesian_states()
        )
        resolved = resolve_vision_motion(
            parameters,
            current,
            self._get_vision_target,
        )
        if resolved.skipped_reason:
            self.get_logger().info(resolved.skipped_reason)
            return

        for side in resolved.targets:
            if not self._move_l_target_clients[side].wait_for_service(
                timeout_sec=5.0
            ):
                raise ProgramError(
                    f"{MOVE_L_TARGET_SERVICE_NAMES[side]} is unavailable; "
                    "start ros_movel_service first"
                )
        futures = {}
        for side, target in resolved.targets.items():
            request = MoveLTarget.Request()
            request.position = list(target.position)
            request.orientation_override = [
                value is not None
                for value in target.orientation_rpy
            ]
            request.orientation_rpy = [
                0.0 if value is None else value
                for value in target.orientation_rpy
            ]
            request.speed_mm_s = parameters["speed_mm_s"]
            request.zone_mm = parameters["zone_mm"]
            futures[side] = self._move_l_target_clients[
                side
            ].call_async(request)
        self.spin_until_all_done(
            list(futures.values()),
            timeout_s=parameters["response_timeout_s"],
        )
        errors = []
        for side, future in futures.items():
            response = future.result()
            if response is None:
                errors.append(f"{side}: service returned no response")
            elif not response.success:
                errors.append(f"{side}: {response.message}")
            else:
                self.get_logger().info(
                    f"{side} vision MoveL completed: {response.message}"
                )
        if errors:
            raise ProgramError("; ".join(errors))

    def move_l(self, parameters: Dict[str, Any]) -> None:
        """Send one left, right or dual-arm MoveL request."""
        targets = parameters["targets"]
        for side in targets:
            service_name = MOVE_L_SERVICE_NAMES[side]
            if not self._move_l_clients[side].wait_for_service(
                timeout_sec=5.0
            ):
                raise ProgramError(
                    f"{service_name} is unavailable; start "
                    "ros_movel_service first"
                )

        futures: Dict[str, object] = {}
        for side, target in targets.items():
            request = MoveL.Request()
            request.pose = target["pose"]
            request.elbow = target["elbow"]
            request.speed_mm_s = parameters["speed_mm_s"]
            request.zone_mm = parameters["zone_mm"]
            futures[side] = self._move_l_clients[side].call_async(request)

        self.spin_until_all_done(
            list(futures.values()),
            timeout_s=parameters["response_timeout_s"],
        )
        errors = []
        for side, future in futures.items():
            try:
                response = future.result()
            except Exception as exc:  # rclpy propagates transport errors here
                errors.append(f"{side}: service call failed: {exc}")
                continue
            if response is None:
                errors.append(f"{side}: service returned no response")
            elif not response.success:
                errors.append(f"{side}: {response.message}")
            else:
                self.get_logger().info(
                    f"{side} MoveL completed: {response.message}"
                )
        if errors:
            raise ProgramError("; ".join(errors))

    def wait(self, duration_s: float) -> None:
        """Wait without starving ROS callbacks."""
        self.wait_between_steps(duration_s)

    def control_hand(self, parameters: Dict[str, Any]) -> None:
        """Send one left, right or dual-hand end-CAN command."""
        requests = parameters["requests"]
        for side in requests:
            service_name = HAND_SERVICE_NAMES[side]
            if not self._hand_clients[side].wait_for_service(
                timeout_sec=5.0
            ):
                raise ProgramError(
                    f"{service_name} is unavailable; start "
                    "ros_hand_service first"
                )

        futures: Dict[str, object] = {}
        for side, hand_command in requests.items():
            request = ControlHand.Request()
            request.command = hand_command["command"]
            request.values = hand_command["values"]
            futures[side] = self._hand_clients[side].call_async(request)

        self.spin_until_all_done(
            list(futures.values()),
            timeout_s=parameters["response_timeout_s"],
        )
        errors = []
        for side, future in futures.items():
            try:
                response = future.result()
            except Exception as exc:
                errors.append(f"{side}: hand service failed: {exc}")
                continue
            if response is None:
                errors.append(f"{side}: hand service returned no response")
            elif not response.success:
                errors.append(f"{side}: {response.message}")
            else:
                self.get_logger().info(response.message)
        if errors:
            raise ProgramError("; ".join(errors))

    def move_l_relative(self, parameters: Dict[str, Any]) -> None:
        """Translate from current TCP poses through relative MoveL services."""
        offsets = parameters["offsets"]
        orientations = parameters["orientations"]
        for side in offsets:
            service_name = MOVE_L_RELATIVE_SERVICE_NAMES[side]
            if not self._move_l_relative_clients[side].wait_for_service(
                timeout_sec=5.0
            ):
                raise ProgramError(
                    f"{service_name} is unavailable; start "
                    "ros_movel_service first"
                )

        futures: Dict[str, object] = {}
        for side, translation in offsets.items():
            request = MoveLRelative.Request()
            request.translation = translation
            orientation = orientations.get(side)
            if orientation is None:
                request.orientation_override = [False, False, False]
                request.orientation_rpy = [0.0, 0.0, 0.0]
            else:
                request.orientation_override = [
                    value is not None for value in orientation
                ]
                request.orientation_rpy = [
                    0.0 if value is None else value
                    for value in orientation
                ]
            request.speed_mm_s = parameters["speed_mm_s"]
            request.zone_mm = parameters["zone_mm"]
            futures[side] = self._move_l_relative_clients[
                side
            ].call_async(request)

        self.spin_until_all_done(
            list(futures.values()),
            timeout_s=parameters["response_timeout_s"],
        )
        errors = []
        for side, future in futures.items():
            try:
                response = future.result()
            except Exception as exc:
                errors.append(f"{side}: relative service failed: {exc}")
                continue
            if response is None:
                errors.append(
                    f"{side}: relative service returned no response"
                )
            elif not response.success:
                errors.append(f"{side}: {response.message}")
            else:
                self.get_logger().info(
                    f"{side} relative MoveL completed: {response.message}"
                )
        if errors:
            raise ProgramError("; ".join(errors))
