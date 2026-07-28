"""Low-level ROS 2 action/service calls used by behavior implementations."""

from typing import Any, Dict, List

from rokae_interfaces.srv import MoveL
from rokae_interfaces.srv import MoveLRelative

from .ros_dual_moveabsj_client import (
    DualMoveAbsJClient,
    ProgramError,
)


MOVE_L_SERVICE_NAMES = {
    "left": "/left_arm/move_l",
    "right": "/right_arm/move_l",
}
MOVE_L_RELATIVE_SERVICE_NAMES = {
    "left": "/left_arm/move_l_relative",
    "right": "/right_arm/move_l_relative",
}


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

    def move_absj(
        self, action_name: str, targets: Dict[str, List[float]]
    ) -> None:
        """Send one left, right or dual-arm MoveAbsJ step."""
        self.run_step({"name": action_name, **targets})

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
