"""Tests for translating normalized behavior parameters to ROS requests."""

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
        assert set(node._move_l_clients) == {"left", "right"}
        assert set(node._move_l_relative_clients) == {"left", "right"}
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
