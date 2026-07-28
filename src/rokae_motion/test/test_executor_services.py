"""Tests for translating normalized behavior parameters to ROS requests."""

import pytest
import rclpy

from rokae_motion.executor_services import RokaeActionExecutor


class FakeResponse:
    success = True
    message = "ok"


class FakeFuture:
    def result(self):
        return FakeResponse()


class FakeClient:
    def __init__(self):
        self.request = None

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.request = request
        return FakeFuture()


class FakeLogger:
    def info(self, message):
        pass


class FakeExecutor:
    def __init__(self):
        self._move_l_relative_clients = {
            "left": FakeClient(),
            "right": FakeClient(),
        }

    def spin_until_all_done(self, futures, timeout_s):
        pass

    def get_logger(self):
        return FakeLogger()


def test_rokae_action_executor_initializes_ros_clients() -> None:
    rclpy.init()
    node = None
    try:
        node = RokaeActionExecutor()

        assert isinstance(node._clients, list)
        assert set(node._move_absj_action_clients) == {"left", "right"}
        assert set(node._hand_clients) == {"left", "right"}
        assert set(node._cartesian_state_clients) == {"left", "right"}
        assert set(node._move_l_clients) == {"left", "right"}
        assert set(node._move_l_relative_clients) == {"left", "right"}
        assert set(node._move_l_target_clients) == {"left", "right"}
        assert node._initialize_robots_client in node._clients
        assert node._trigger_vision_client in node._clients
        assert node._get_vision_target_client in node._clients
        assert node._set_vision_offset_client in node._clients
        assert len(node._navigation_cancel_clients) == 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_sparse_orientation_becomes_per_axis_override() -> None:
    executor = FakeExecutor()

    RokaeActionExecutor.move_l_relative(
        executor,
        {
            "offsets": {"left": [0.02, 0.0, 0.0]},
            "orientations": {"left": [None, 0.2, None]},
            "speed_mm_s": 30.0,
            "zone_mm": 0.0,
            "response_timeout_s": 70.0,
        },
    )

    request = executor._move_l_relative_clients["left"].request
    assert list(request.translation) == [0.02, 0.0, 0.0]
    assert list(request.orientation_override) == [False, True, False]
    assert list(request.orientation_rpy) == [0.0, 0.2, 0.0]
    assert executor._move_l_relative_clients["right"].request is None


class FakeInitializerExecutor:
    def __init__(self):
        self._initialize_robots_client = FakeClient()

    def spin_until_all_done(self, futures, timeout_s):
        pass

    def get_logger(self):
        return FakeLogger()


def test_robot_initialization_calls_trigger_service() -> None:
    executor = FakeInitializerExecutor()

    RokaeActionExecutor.initialize_robots(executor)

    request = executor._initialize_robots_client.request
    assert request.__class__.__name__ == "Trigger_Request"


class FakeVisionExecutor:
    def __init__(self):
        self._trigger_vision_client = FakeClient()

    def spin_until_all_done(self, futures, timeout_s):
        pass

    def get_logger(self):
        return FakeLogger()


def test_aruco_vision_parameters_become_trigger_request() -> None:
    executor = FakeVisionExecutor()

    RokaeActionExecutor.trigger_vision(
        executor,
        {
            "source": "aruco",
            "echo_topic": "/tool/pose",
            "pub_topic": "/aruco/enable",
            "key": "tool_pose",
            "trigger_value": 1,
            "labels": [],
            "point_names": ["tool"],
            "response_timeout_s": 10.0,
        },
    )

    request = executor._trigger_vision_client.request
    assert request.source == "aruco"
    assert request.echo_topic == "/tool/pose"
    assert request.pub_topic == "/aruco/enable"
    assert request.key == "tool_pose"
    assert request.trigger_value == 1
    assert request.motion_mode == 0
    assert list(request.point_names) == ["tool"]


class FakeHandExecutor:
    def __init__(self):
        self._hand_clients = {
            "left": FakeClient(),
            "right": FakeClient(),
        }

    def spin_until_all_done(self, futures, timeout_s):
        pass

    def get_logger(self):
        return FakeLogger()


def test_hand_parameters_become_dual_service_requests() -> None:
    executor = FakeHandExecutor()

    RokaeActionExecutor.control_hand(
        executor,
        {
            "requests": {
                "left": {
                    "command": "motors",
                    "values": [255, 254, 253, 252, 251, 250],
                },
                "right": {
                    "command": "speed",
                    "values": [96, 0, 0, 0, 0, 0],
                },
            },
            "response_timeout_s": 5.0,
        },
    )

    left = executor._hand_clients["left"].request
    right = executor._hand_clients["right"].request
    assert left.command == "motors"
    assert list(left.values) == [255, 254, 253, 252, 251, 250]
    assert right.command == "speed"
    assert list(right.values) == [96, 0, 0, 0, 0, 0]


class FakeNavigationPublisher:
    def __init__(self, executor):
        self.executor = executor
        self.command = None

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.command = message.data
        self.executor._chassis_state = "ARRIVE_A"
        self.executor._chassis_state_sequence += 1


class FakeNavigationExecutor:
    def __init__(self):
        self._chassis_state = ""
        self._chassis_state_sequence = 0
        self._navigation_active = False
        self._navigation_command_publisher = FakeNavigationPublisher(self)
        self.cancel_count = 0

    def get_logger(self):
        return FakeLogger()

    def _cancel_navigation(self):
        self.cancel_count += 1


def test_navigation_waits_for_expected_arrival(monkeypatch) -> None:
    executor = FakeNavigationExecutor()
    monkeypatch.setattr(rclpy, "ok", lambda: True)
    monkeypatch.setattr(rclpy, "spin_once", lambda *args, **kwargs: None)

    RokaeActionExecutor.navigate(
        executor,
        {
            "command": "LM2",
            "station": "LM2",
            "arrival_state": "ARRIVE_A",
            "connection_timeout_s": 1.0,
            "response_timeout_s": 1.0,
        },
    )

    assert executor._navigation_command_publisher.command == "LM2"
    assert not executor._navigation_active


def test_navigation_ctrl_c_requests_cancel(monkeypatch) -> None:
    executor = FakeNavigationExecutor()
    monkeypatch.setattr(rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        rclpy,
        "spin_once",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            KeyboardInterrupt()
        ),
    )

    with pytest.raises(KeyboardInterrupt):
        RokaeActionExecutor.navigate(
            executor,
            {
                "command": "LM2",
                "station": "LM2",
                "arrival_state": "ARRIVE_A",
                "connection_timeout_s": 1.0,
                "response_timeout_s": 1.0,
            },
        )

    assert executor.cancel_count == 1
    assert not executor._navigation_active
