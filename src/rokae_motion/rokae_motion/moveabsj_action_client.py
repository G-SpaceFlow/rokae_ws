"""Shared ROS 2 MoveAbsJ action client used by behavior-tree execution."""

import math
import time
from typing import Dict, List, Optional, Sequence

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_COUNT = 7
ACTION_NAMES = {
    "left": "/left_arm/move_absj",
    "right": "/right_arm/move_absj",
}


class ProgramError(RuntimeError):
    """Raised when a ROS motion request or result is invalid."""


def validate_joint_target(side: str, values: object) -> List[float]:
    if not isinstance(values, list) or len(values) != JOINT_COUNT:
        raise ProgramError(f"{side} target must contain exactly 7 numbers")

    target: List[float] = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProgramError(
                f"{side} J{index + 1} must be a number, got {value!r}"
            )
        number = float(value)
        if not math.isfinite(number):
            raise ProgramError(f"{side} J{index + 1} must be finite")
        target.append(number)
    return target


class DualMoveAbsJClient(Node):
    def __init__(self) -> None:
        super().__init__("rokae_action_executor")
        self._move_absj_action_clients = {
            side: ActionClient(self, FollowJointTrajectory, action_name)
            for side, action_name in ACTION_NAMES.items()
        }
        self._active_handles: Dict[str, object] = {}
        self._last_feedback_time = {"left": 0.0, "right": 0.0}

    def wait_for_servers(self, sides: Sequence[str]) -> None:
        for side in sides:
            action_name = ACTION_NAMES[side]
            if not self._move_absj_action_clients[
                side
            ].wait_for_server(timeout_sec=5.0):
                raise ProgramError(
                    f"{action_name} is unavailable; start "
                    "ros_moveabsj_action_server first"
                )

    def make_goal(
        self, side: str, target: Sequence[float]
    ) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            f"{side}_joint_{index}" for index in range(1, JOINT_COUNT + 1)
        ]
        point = JointTrajectoryPoint()
        point.positions = list(target)
        goal.trajectory.points = [point]
        return goal

    def feedback_callback(self, side: str, message: object) -> None:
        now = time.monotonic()
        if now - self._last_feedback_time[side] < 0.5:
            return
        self._last_feedback_time[side] = now

        errors = message.feedback.error.positions
        maximum_error = max((abs(value) for value in errors), default=0.0)
        self.get_logger().debug(
            f"{side} feedback: max joint error={maximum_error:.6f} rad"
        )

    def spin_until_all_done(
        self, futures: Sequence[object], timeout_s: Optional[float]
    ) -> None:
        deadline = (
            None if timeout_s is None else time.monotonic() + timeout_s
        )
        while rclpy.ok() and not all(future.done() for future in futures):
            if deadline is not None and time.monotonic() >= deadline:
                raise ProgramError(
                    "timed out waiting for ROS 2 action response"
                )
            rclpy.spin_once(self, timeout_sec=0.05)

    def cancel_handles(self, handles: Dict[str, object]) -> None:
        cancel_futures = []
        for side, handle in handles.items():
            if handle is None or not handle.accepted:
                continue
            self.get_logger().warning(f"requesting {side} goal cancellation")
            cancel_futures.append(handle.cancel_goal_async())
        if cancel_futures:
            try:
                self.spin_until_all_done(cancel_futures, timeout_s=5.0)
            except ProgramError as exc:
                self.get_logger().error(str(exc))

    def cancel_active(self) -> None:
        self.cancel_handles(dict(self._active_handles))

    def run_step(self, step: dict) -> None:
        sides = [side for side in ACTION_NAMES if side in step]
        self.wait_for_servers(sides)

        send_futures = {}
        for side in sides:
            goal = self.make_goal(side, step[side])
            send_futures[side] = self._move_absj_action_clients[
                side
            ].send_goal_async(
                goal,
                feedback_callback=lambda message, selected_side=side:
                self.feedback_callback(selected_side, message),
            )

        # Both requests have been queued before waiting on either one.
        self.spin_until_all_done(list(send_futures.values()), timeout_s=10.0)

        handles = {
            side: future.result() for side, future in send_futures.items()
        }
        rejected = [side for side, handle in handles.items()
                    if handle is None or not handle.accepted]
        if rejected:
            self.cancel_handles(handles)
            raise ProgramError(
                "goal rejected for: " + ", ".join(rejected)
            )

        self._active_handles = handles
        for side in sides:
            self.get_logger().debug(f"{side} MoveAbsJ goal accepted")

        result_futures = {
            side: handle.get_result_async()
            for side, handle in handles.items()
        }

        failure_detected = False
        while rclpy.ok() and not all(
            future.done() for future in result_futures.values()
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
            if failure_detected:
                continue
            for side, future in result_futures.items():
                if not future.done():
                    continue
                response = future.result()
                if (
                    response.status != GoalStatus.STATUS_SUCCEEDED
                    or response.result.error_code
                    != FollowJointTrajectory.Result.SUCCESSFUL
                ):
                    failure_detected = True
                    remaining = {
                        other_side: handles[other_side]
                        for other_side, other_future
                        in result_futures.items()
                        if not other_future.done()
                    }
                    self.cancel_handles(remaining)
                    break

        if not rclpy.ok():
            self.cancel_active()
            raise ProgramError("ROS 2 shutdown while goals were active")

        errors = []
        for side, future in result_futures.items():
            response = future.result()
            result = response.result
            if (
                response.status != GoalStatus.STATUS_SUCCEEDED
                or result.error_code
                != FollowJointTrajectory.Result.SUCCESSFUL
            ):
                errors.append(
                    f"{side}: status={response.status}, "
                    f"code={result.error_code}, "
                    f"message={result.error_string}"
                )
            else:
                self.get_logger().info(
                    f"{side} completed: {result.error_string}"
                )
        self._active_handles = {}

        if errors:
            raise ProgramError("; ".join(errors))

    def wait_between_steps(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, deadline - time.monotonic()),
            )
