#!/usr/bin/env python3
"""Run a JSON-programmed dual-arm sequence through the MoveAbsJ actions.

The C++ action server must already be running. This client requires no build:

  ros2 run rokae_motion moveabsj_client PROGRAM.json
  ros2 run rokae_motion moveabsj_client PROGRAM.json --execute
  ros2 run rokae_motion moveabsj_client PROGRAM.json --arm left --execute

Without --execute the file is only validated and printed. Each program step
sends the left and right goals before waiting for either result, so both arms
start approximately together. This is not hard real-time synchronization.
"""

import argparse
import json
import math
from pathlib import Path
import sys
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
    """Raised when the JSON program or action result is invalid."""


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


def load_program(path: Path, selected_step: Optional[str]) -> List[dict]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ProgramError(f"cannot read {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ProgramError(f"invalid JSON in {path}: {exc}") from exc

    if not isinstance(data, dict):
        raise ProgramError("program root must be a JSON object")
    if data.get("replace_before_use", False):
        raise ProgramError(
            "template is locked: replace all null targets, then set "
            '"replace_before_use" to false'
        )

    raw_steps = data.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ProgramError("program must contain at least one step")
    if len(raw_steps) > 100:
        raise ProgramError("program must not contain more than 100 steps")

    steps: List[dict] = []
    names = set()
    for index, raw_step in enumerate(raw_steps):
        if not isinstance(raw_step, dict):
            raise ProgramError(f"step {index + 1} must be a JSON object")

        name = raw_step.get("name", f"step_{index + 1}")
        if not isinstance(name, str) or not name.strip():
            raise ProgramError(f"step {index + 1} has an invalid name")
        if name in names:
            raise ProgramError(f"duplicate step name: {name}")
        names.add(name)

        step = {"name": name}
        for side in ACTION_NAMES:
            if side in raw_step:
                step[side] = validate_joint_target(side, raw_step[side])
        if "left" not in step and "right" not in step:
            raise ProgramError(f"{name} must contain left and/or right target")

        wait_after_s = raw_step.get("wait_after_s", 0.0)
        if (
            isinstance(wait_after_s, bool)
            or not isinstance(wait_after_s, (int, float))
            or not math.isfinite(float(wait_after_s))
            or float(wait_after_s) < 0.0
            or float(wait_after_s) > 60.0
        ):
            raise ProgramError(
                f"{name}.wait_after_s must be between 0 and 60"
            )
        step["wait_after_s"] = float(wait_after_s)
        steps.append(step)

    if selected_step is not None:
        steps = [step for step in steps if step["name"] == selected_step]
        if not steps:
            raise ProgramError(f"step not found: {selected_step}")
    return steps


def print_program(steps: Sequence[dict]) -> None:
    print(f"Validated {len(steps)} program step(s):")
    for index, step in enumerate(steps, start=1):
        print(f"  {index}. {step['name']}")
        for side in ACTION_NAMES:
            if side in step:
                values = ", ".join(f"{value:.6f}" for value in step[side])
                print(f"     {side}: [{values}] rad")
        print(f"     wait_after_s: {step['wait_after_s']:.3f}")


def select_arms(steps: Sequence[dict], selection: str) -> List[dict]:
    if selection == "both":
        return list(steps)

    selected_steps: List[dict] = []
    for step in steps:
        if selection not in step:
            continue
        selected_steps.append(
            {
                "name": step["name"],
                selection: step[selection],
                "wait_after_s": step["wait_after_s"],
            }
        )

    if not selected_steps:
        raise ProgramError(
            f"the selected program contains no {selection} arm targets"
        )
    return selected_steps


class DualMoveAbsJClient(Node):
    def __init__(self) -> None:
        super().__init__("rokae_dual_moveabsj_client")
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


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Validate or execute a JSON dual-arm MoveAbsJ program"
        )
    )
    parser.add_argument("program", type=Path, help="JSON program file")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually send goals; without this flag only validate",
    )
    parser.add_argument(
        "--step",
        help="execute or validate only the named step",
    )
    parser.add_argument(
        "--arm",
        choices=("left", "right", "both"),
        default="both",
        help=(
            "select which arm targets to use from the JSON "
            "(default: both)"
        ),
    )
    return parser.parse_args()


def run_program(
    program: Path,
    arm: str = "both",
    selected_step: Optional[str] = None,
    execute: bool = False,
) -> int:
    """Validate and optionally execute one JSON MoveAbsJ program."""
    try:
        if arm not in ACTION_NAMES and arm != "both":
            raise ProgramError(
                f"arm must be left, right or both, got {arm!r}"
            )
        steps = load_program(program, selected_step)
        steps = select_arms(steps, arm)
        print_program(steps)
    except ProgramError as exc:
        print(f"Program error: {exc}", file=sys.stderr)
        return 2

    if not execute:
        print(
            "Dry run only: no robot command was sent. "
            "Add --execute after checking every target."
        )
        return 0

    print(
        "EXECUTION ENABLED. Confirm both targets are based on current joint "
        "feedback, the workspace is clear, and the E-stop is accessible."
    )
    confirmation = input("Type EXECUTE to start the programmed sequence: ")
    if confirmation != "EXECUTE":
        print("Cancelled; no action goal was sent.")
        return 0

    rclpy.init()
    node: Optional[DualMoveAbsJClient] = None
    try:
        node = DualMoveAbsJClient()
        for index, step in enumerate(steps, start=1):
            node.get_logger().info(
                f"starting step {index}/{len(steps)}: {step['name']}"
            )
            node.run_step(step)
            node.wait_between_steps(step["wait_after_s"])
        node.get_logger().info("dual-arm program completed")
        return 0
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                "Ctrl+C received; requesting cancellation for active goals"
            )
            node.cancel_active()
        return 130
    except ProgramError as exc:
        if node is not None:
            node.get_logger().error(str(exc))
            node.cancel_active()
        else:
            print(f"Program error: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    arguments = parse_arguments()
    return run_program(
        program=arguments.program,
        arm=arguments.arm,
        selected_step=arguments.step,
        execute=arguments.execute,
    )


if __name__ == "__main__":
    raise SystemExit(main())
