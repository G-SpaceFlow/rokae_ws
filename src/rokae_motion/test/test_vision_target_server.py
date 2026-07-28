"""Tests for JSON point storage and motion-mode selection."""

import json

from box_detection_interfaces.msg import BoxGrabPoints, SmallBoxTarget
from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rokae_interfaces.srv import GetVisionTarget

import rokae_motion.vision_target_server as server_module
from rokae_motion.vision_target_server import VisionTargetServer


def test_target_server_caches_points_and_scalars() -> None:
    rclpy.init()
    node = None
    try:
        node = VisionTargetServer()
        detection = {
            "labels": ["tray"],
            "y_offset": -0.0079,
            "points": {
                "left": {"x": 0.1, "y": 0.2, "z": 0.3},
                "right": {"x": 0.4, "y": 0.5, "z": 0.6},
            },
        }
        node._cache_detection("sample", detection)

        selected = node._select_cached_points(
            "sample", ["left", "right"], 1
        )
        scalar = node._select_cached_points(
            "sample", ["y_offset"], 6
        )

        assert selected is not None
        assert selected[0].position.x == 0.1
        assert selected[1].position.x == 0.4
        assert scalar is not None
        assert scalar[0].position.x == -0.0079
        assert scalar[1].position.x == -0.0079
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_target_server_accepts_double_encoded_json() -> None:
    rclpy.init()
    node = None
    try:
        node = VisionTargetServer()
        contents = json.dumps(json.dumps({"points": {}}))

        assert node._parse_json_message(contents, "test") == {
            "points": {}
        }
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_wait_for_latest_returns_frame_after_one_second(
    monkeypatch,
) -> None:
    first = PoseStamped()
    first.pose.position.x = 0.1
    latest_message = PoseStamped()
    latest_message.pose.position.x = 0.2
    state = {"latest": first}

    monkeypatch.setattr(rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        server_module.time,
        "sleep",
        lambda _duration: state.update(latest=latest_message),
    )

    result = VisionTargetServer._wait_for_latest(
        lambda: state["latest"],
        timeout_s=5.0,
        post_detection_delay_s=1.0,
    )

    assert result is latest_message
    assert result.pose.position.x == 0.2


def test_aruco_pose_is_cached_with_orientation() -> None:
    rclpy.init()
    node = None
    try:
        node = VisionTargetServer()
        message = PoseStamped()
        message.header.frame_id = "base_link"
        message.pose.position.x = 0.485
        message.pose.position.y = 0.054
        message.pose.position.z = -0.430
        message.pose.orientation.x = 0.48
        message.pose.orientation.y = 0.51
        message.pose.orientation.z = -0.50
        message.pose.orientation.w = 0.50
        response = GetVisionTarget.Response()

        result = node._cache_aruco_result(
            response,
            "tool_pose",
            message,
            ["tool"],
            0,
        )

        assert result.success
        assert result.message == "cached"
        cached_pose = node.point_cache["tool_pose"]["tool"]
        assert cached_pose.position.x == 0.485
        assert cached_pose.position.z == -0.430
        quaternion = cached_pose.orientation
        norm = (
            quaternion.x ** 2
            + quaternion.y ** 2
            + quaternion.z ** 2
            + quaternion.w ** 2
        ) ** 0.5
        assert norm == pytest.approx(1.0)
        assert "tool" in node.point_cache["tool_pose"]
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_box_left_and_right_points_are_cached() -> None:
    rclpy.init()
    node = None
    try:
        node = VisionTargetServer()
        message = BoxGrabPoints()
        message.header.frame_id = "base_link"
        message.left_center.x = 0.521
        message.left_center.y = 0.300
        message.left_center.z = -0.618
        message.right_center.x = 0.562
        message.right_center.y = -0.075
        message.right_center.z = -0.686
        message.angle_deg = -4.98
        response = GetVisionTarget.Response()

        result = node._cache_box_result(
            response,
            "box_pose",
            message,
            ["left_center", "right_center"],
            0,
        )

        assert result.success
        assert result.message == "cached"
        points = node.point_cache["box_pose"]
        assert points["left_center"].position.y == pytest.approx(0.300)
        assert points["right_center"].position.z == pytest.approx(
            -0.686
        )
        assert node.detection_cache["box_pose"]["angle_deg"] == (
            pytest.approx(-4.98)
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_small_box_center_is_cached() -> None:
    rclpy.init()
    node = None
    try:
        node = VisionTargetServer()
        message = SmallBoxTarget()
        message.header.frame_id = "base_link"
        message.center.x = 0.467025
        message.center.y = -0.053667
        message.center.z = -0.447923
        message.angle_deg = -7.305777
        response = GetVisionTarget.Response()

        result = node._cache_small_box_result(
            response,
            "small_box_pose",
            message,
            ["center"],
            0,
        )

        assert result.success
        assert result.message == "cached"
        center = node.point_cache["small_box_pose"]["center"]
        assert center.position.x == pytest.approx(0.467025)
        assert center.position.y == pytest.approx(-0.053667)
        assert center.position.z == pytest.approx(-0.447923)
        assert (
            node.detection_cache["small_box_pose"]["angle_deg"]
            == pytest.approx(-7.305777)
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_small_box_trigger_publishes_one_then_zero(
    monkeypatch,
) -> None:
    class FakePublisher:
        def __init__(self) -> None:
            self.values = []

        def publish(self, message) -> None:
            self.values.append(message.data)

    rclpy.init()
    node = None
    try:
        node = VisionTargetServer()
        message = SmallBoxTarget()
        message.header.frame_id = "base_link"
        message.center.x = 0.467
        message.center.y = -0.054
        message.center.z = -0.448
        message.angle_deg = -7.3
        publisher = FakePublisher()
        monkeypatch.setattr(
            node,
            "_ensure_typed_interfaces",
            lambda _source, _pub_topic, _echo_topic: (
                publisher,
                lambda: message,
            ),
        )
        monkeypatch.setattr(
            node,
            "_wait_for_subscriber",
            lambda _publisher: True,
        )
        monkeypatch.setattr(
            node,
            "_wait_for_latest",
            lambda latest, _timeout, _delay: latest(),
        )
        request = GetVisionTarget.Request()
        request.source = "small_box_target"
        request.echo_topic = "/small_box/target"
        request.pub_topic = "/small_box/enable"
        request.key = "small_box_pose"
        request.trigger_value = 1
        request.point_names = ["center"]
        request.motion_mode = 0

        result = node._handle_trigger_detect(
            request, GetVisionTarget.Response()
        )

        assert result.success
        assert publisher.values == [1, 0]
        assert "center" in node.point_cache["small_box_pose"]
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
