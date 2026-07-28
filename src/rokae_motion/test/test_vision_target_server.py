"""Tests for JSON point storage and motion-mode selection."""

import json

import rclpy

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
