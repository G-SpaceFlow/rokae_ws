#!/usr/bin/env python3
"""Bridge scheduler station commands to the Seer Navigate action."""

from typing import Dict, Tuple
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from seer_interfaces.action import Navigate
from std_msgs.msg import String
from std_srvs.srv import Trigger


TOPIC_CMD_CHASSIS = "/scheduler/cmd/chassis"
TOPIC_STATE_CHASSIS = "/chassis/state"
SEER_NAVIGATE_ACTION = "/seer/navigate"
SEER_CANCEL_NAVIGATION = "/seer/cancel_navigation"
BT_CANCEL_NAVIGATION = "/bt_navigation_server/cancel_navigation"

STATE_ARRIVE_A = "ARRIVE_A"
STATE_ARRIVE_B = "ARRIVE_B"
STATE_ARRIVE_HOME = "ARRIVE_HOME"
STATE_NAVIGATION_FAILED = "NAVIGATION_FAILED"
STATE_NAVIGATION_CANCELED = "NAVIGATION_CANCELED"

STATE_MAP: Dict[str, str] = {
    "LM1": STATE_ARRIVE_HOME,
    "LM2": STATE_ARRIVE_A,
    "LM3": STATE_ARRIVE_B,
}
ARRIVAL_STATES = frozenset(STATE_MAP.values())
FAILURE_STATES = frozenset(
    (STATE_NAVIGATION_FAILED, STATE_NAVIGATION_CANCELED)
)


def navigation_command_details(command: str) -> Tuple[str, str, str]:
    """Return the Seer station name and expected scheduler arrival state."""
    station = str(command).strip().upper()
    if station not in STATE_MAP:
        supported = ", ".join(STATE_MAP)
        raise ValueError(
            f"unsupported navigation command {command!r}; use {supported}"
        )
    return station, station, STATE_MAP[station]


def make_navigation_goal(station: str) -> Navigate.Goal:
    """Create one uniquely identified station-navigation goal."""
    goal = Navigate.Goal()
    goal.mode = Navigate.Goal.MODE_STATION
    goal.target_id = station
    goal.source_id = "SELF_POSITION"
    goal.task_id = f"bt_{uuid.uuid4().hex}"
    return goal


def navigation_terminal_state(
    station: str, goal_status: int, success: bool
) -> str:
    """Translate the ROS Action terminal result to a scheduler state."""
    if goal_status == GoalStatus.STATUS_SUCCEEDED and success:
        return STATE_MAP[station]
    if goal_status == GoalStatus.STATUS_CANCELED:
        return STATE_NAVIGATION_CANCELED
    return STATE_NAVIGATION_FAILED


class ChassisNavigationBridge(Node):
    """Translate scheduler commands and report Navigate action results."""

    def __init__(self) -> None:
        super().__init__("chassis_navigation_bridge")
        self._state_publisher = self.create_publisher(
            String, TOPIC_STATE_CHASSIS, 10
        )
        self._command_subscription = self.create_subscription(
            String, TOPIC_CMD_CHASSIS, self._command_callback, 10
        )
        self._navigate_action_client = ActionClient(
            self, Navigate, SEER_NAVIGATE_ACTION
        )
        self._cancel_service = self.create_service(
            Trigger, BT_CANCEL_NAVIGATION, self._cancel_callback
        )

        self._navigation_running = False
        self._current_station = ""
        self._goal_handle = None
        self._goal_sequence = 0
        self._cancel_requested = False
        self.get_logger().info(
            "Seer chassis navigation bridge ready: "
            f"{TOPIC_CMD_CHASSIS} -> {SEER_NAVIGATE_ACTION}"
        )

    def _publish_state(self, state: str) -> None:
        self._state_publisher.publish(String(data=state))
        self.get_logger().info(f"chassis state: {state}")

    def _finish_navigation(self) -> None:
        self._navigation_running = False
        self._current_station = ""
        self._goal_handle = None
        self._cancel_requested = False

    def _command_callback(self, message: String) -> None:
        try:
            command, station, _ = navigation_command_details(message.data)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            self._publish_state(STATE_NAVIGATION_FAILED)
            return

        if self._navigation_running:
            self.get_logger().warning(
                f"navigation already running; ignored {command}"
            )
            return
        if not self._navigate_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                f"{SEER_NAVIGATE_ACTION} is unavailable; start "
                "seer_navigation_node first"
            )
            self._publish_state(STATE_NAVIGATION_FAILED)
            return

        self._current_station = station
        self._navigation_running = True
        self._cancel_requested = False
        self._goal_sequence += 1
        sequence = self._goal_sequence
        future = self._navigate_action_client.send_goal_async(
            make_navigation_goal(station),
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(
            lambda completed, selected_sequence=sequence:
            self._goal_response_callback(completed, selected_sequence)
        )
        self.get_logger().info(
            f"sent Seer Navigate goal: station={station}"
        )

    def _feedback_callback(self, message: object) -> None:
        feedback = message.feedback
        self.get_logger().debug(
            "Seer navigation feedback: "
            f"task_id={feedback.task_id}, "
            f"status={feedback.task_status}, "
            f"percentage={feedback.percentage:.1f}, "
            f"distance={feedback.distance:.3f}"
        )

    def _goal_response_callback(
        self, future: object, sequence: int
    ) -> None:
        if sequence != self._goal_sequence or not self._navigation_running:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"failed to send Seer Navigate goal: {exc}"
            )
            self._publish_state(STATE_NAVIGATION_FAILED)
            self._finish_navigation()
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f"Seer Navigate goal rejected: station={self._current_station}"
            )
            self._publish_state(STATE_NAVIGATION_FAILED)
            self._finish_navigation()
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, selected_sequence=sequence:
            self._result_callback(completed, selected_sequence)
        )
        if self._cancel_requested:
            self._request_goal_cancellation()

    def _result_callback(self, future: object, sequence: int) -> None:
        if sequence != self._goal_sequence or not self._navigation_running:
            return
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            status = wrapped_result.status
        except Exception as exc:
            self.get_logger().error(
                f"Seer Navigate result failed: {exc}"
            )
            self._publish_state(STATE_NAVIGATION_FAILED)
            self._finish_navigation()
            return

        station = self._current_station
        terminal_state = navigation_terminal_state(
            station, status, result.success
        )
        if terminal_state == STATE_MAP[station]:
            self.get_logger().info(
                f"Seer navigation completed: station={station}"
            )
        elif terminal_state == STATE_NAVIGATION_CANCELED:
            self.get_logger().warning(
                f"Seer navigation canceled: station={station}"
            )
        else:
            self.get_logger().error(
                f"Seer navigation failed: station={station}, "
                f"goal_status={status}, "
                f"final_status={result.final_status}, "
                f"message={result.message}"
            )
        self._publish_state(terminal_state)
        self._finish_navigation()

    def _cancel_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._navigation_running:
            response.success = True
            response.message = "no navigation is active"
            return response
        if self._cancel_requested:
            response.success = True
            response.message = "Seer Navigate goal cancellation is pending"
            return response

        self._cancel_requested = True
        if self._goal_handle is not None:
            self._request_goal_cancellation()
        response.success = True
        response.message = "Seer Navigate goal cancellation requested"
        return response

    def _request_goal_cancellation(self) -> None:
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_response_callback)

    def _cancel_response_callback(self, future: object) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Seer Navigate cancellation failed: {exc}"
            )
            return
        if response is None or not response.goals_canceling:
            self.get_logger().error(
                "Seer Navigate goal did not accept cancellation"
            )
        else:
            self.get_logger().info(
                "Seer Navigate goal accepted cancellation"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisNavigationBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
