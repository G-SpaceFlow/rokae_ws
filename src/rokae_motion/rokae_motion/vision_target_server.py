#!/usr/bin/env python3
"""ROS 2 vision trigger, target-point cache and offset service.

This is the Rokae ROS 2 port of ``zj_robot_bt_action/bt_target_server.py``.
It stores JSON detections in memory and exposes selected points to behavior
tree actions. It does not run a detector or save camera images.
"""

import copy
import json
import math
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Pose
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rokae_interfaces.srv import GetVisionTarget
from rokae_interfaces.srv import SetVisionOffset
from std_msgs.msg import Int32
from std_msgs.msg import String


CENTER_CACHE_PREFIX = "__center__:"
VISION_POINTS_TOPIC = "/yolo_vision/front_points_base_json"
WALL_ANGLE_TOPIC = "/yolo_vision/wall_angle"
MODE5_POINTS_TOPIC = "/yolo_vision/mode5_points_json"
VISION_CONTROL_TOPIC = "/yolo_vision/control"
TARGET_LABELS_TOPIC = "/yolo_vision/target_labels"
TRIGGER_DETECT_SERVICE = "/bt_target_server/trigger_detect"
GET_TARGET_SERVICE = "/bt_target_server/get_target"
SET_OFFSET_SERVICE = "/bt_target_server/set_offset"


class VisionTargetServer(Node):
    """Cache detector JSON and return points using motion modes 1 through 6."""

    def __init__(self) -> None:
        super().__init__("bt_target_server")
        self._lock = threading.RLock()
        self._callbacks = ReentrantCallbackGroup()
        self._init_state()
        self._init_interfaces()
        self.get_logger().info(
            "vision target services ready: trigger_detect, get_target, "
            "set_offset"
        )

    def _init_state(self) -> None:
        self.pose_cache: Dict[str, Tuple[Pose, Pose]] = {}
        self.detection_cache: Dict[str, Dict[str, Any]] = {}
        self.point_cache: Dict[str, Dict[str, Pose]] = {}
        self.latest_front_detection: Optional[Dict[str, Any]] = None
        self.latest_wall_angle: Optional[Dict[str, Any]] = None
        self.latest_mode5_detection: Optional[Dict[str, Any]] = None
        self.offset_map = {"default": self._identity_offset()}
        self.active_offset_id = "default"

    def _init_interfaces(self) -> None:
        self.declare_parameter(
            "vision_points_topic", VISION_POINTS_TOPIC
        )
        self.declare_parameter("wall_angle_topic", WALL_ANGLE_TOPIC)
        self.declare_parameter(
            "mode5_points_topic", MODE5_POINTS_TOPIC
        )
        self.declare_parameter(
            "vision_control_topic", VISION_CONTROL_TOPIC
        )
        self.declare_parameter(
            "target_labels_topic", TARGET_LABELS_TOPIC
        )
        self.declare_parameter("detection_timeout_s", 5.0)
        self.declare_parameter("post_detection_delay_s", 1.0)

        points_topic = self.get_parameter(
            "vision_points_topic"
        ).value
        wall_topic = self.get_parameter("wall_angle_topic").value
        mode5_topic = self.get_parameter("mode5_points_topic").value
        control_topic = self.get_parameter(
            "vision_control_topic"
        ).value
        labels_topic = self.get_parameter(
            "target_labels_topic"
        ).value

        self.create_subscription(
            String,
            points_topic,
            self._front_points_callback,
            10,
            callback_group=self._callbacks,
        )
        self.create_subscription(
            String,
            wall_topic,
            self._wall_angle_callback,
            10,
            callback_group=self._callbacks,
        )
        self.create_subscription(
            String,
            mode5_topic,
            self._mode5_points_callback,
            10,
            callback_group=self._callbacks,
        )
        self.control_publisher = self.create_publisher(
            Int32, control_topic, 10
        )
        self.labels_publisher = self.create_publisher(
            String, labels_topic, 10
        )
        self.create_service(
            GetVisionTarget,
            TRIGGER_DETECT_SERVICE,
            self._handle_trigger_detect,
            callback_group=self._callbacks,
        )
        self.create_service(
            GetVisionTarget,
            GET_TARGET_SERVICE,
            self._handle_get_target,
            callback_group=self._callbacks,
        )
        self.create_service(
            SetVisionOffset,
            SET_OFFSET_SERVICE,
            self._handle_set_offset,
            callback_group=self._callbacks,
        )

    def _front_points_callback(self, message: String) -> None:
        detection = self._parse_json_message(
            message.data, "front-points JSON"
        )
        if detection is not None:
            with self._lock:
                self.latest_front_detection = detection

    def _wall_angle_callback(self, message: String) -> None:
        detection = self._parse_json_message(
            message.data, "wall-angle JSON"
        )
        if detection is not None:
            with self._lock:
                self.latest_wall_angle = detection

    def _mode5_points_callback(self, message: String) -> None:
        detection = self._parse_json_message(
            message.data, "mode5-points JSON"
        )
        if detection is not None:
            with self._lock:
                self.latest_mode5_detection = detection

    def _parse_json_message(
        self, contents: str, description: str
    ) -> Optional[Dict[str, Any]]:
        try:
            value = json.loads(contents)
            if isinstance(value, str):
                value = json.loads(value)
            if not isinstance(value, dict):
                raise ValueError(
                    f"expected an object, got {type(value).__name__}"
                )
            return value
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f"cannot parse {description}: {exc}"
            )
            return None

    @staticmethod
    def _normalize_motion_mode(value: Any) -> int:
        try:
            mode = int(value)
        except (TypeError, ValueError):
            mode = 1
        return 1 if mode == 0 else mode

    @staticmethod
    def _normalize_strings(values: Any) -> List[str]:
        if values is None:
            return []
        if isinstance(values, str):
            values = [values]
        return [
            str(value).strip()
            for value in values
            if str(value).strip()
        ]

    def _reset_latest(self, trigger_value: int) -> None:
        with self._lock:
            if trigger_value == 3:
                self.latest_wall_angle = None
            elif trigger_value == 5:
                self.latest_mode5_detection = None
            else:
                self.latest_front_detection = None

    def _latest_for_trigger(
        self, trigger_value: int
    ) -> Optional[Dict[str, Any]]:
        with self._lock:
            if trigger_value == 3:
                value = self.latest_wall_angle
            elif trigger_value == 5:
                value = self.latest_mode5_detection
            else:
                value = self.latest_front_detection
            return copy.deepcopy(value)

    def _handle_trigger_detect(
        self,
        request: GetVisionTarget.Request,
        response: GetVisionTarget.Response,
    ) -> GetVisionTarget.Response:
        key = request.key.strip() or "1-1-1-2"
        trigger_value = int(request.trigger_value)
        labels = self._normalize_strings(request.labels)
        point_names = self._normalize_strings(request.point_names)
        motion_mode = self._normalize_motion_mode(request.motion_mode)
        if motion_mode == 1 and len(point_names) == 1:
            motion_mode = 6 if trigger_value == 5 else 4

        self._reset_latest(trigger_value)
        self.labels_publisher.publish(
            String(data=json.dumps(labels, ensure_ascii=False))
        )
        self.control_publisher.publish(Int32(data=trigger_value))
        self.get_logger().info(
            f"vision trigger: key={key}, value={trigger_value}, "
            f"mode={motion_mode}, points={point_names}"
        )

        detection = None
        timeout_s = float(
            self.get_parameter("detection_timeout_s").value
        )
        deadline = time.monotonic() + timeout_s
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                detection = self._latest_for_trigger(trigger_value)
                if detection is not None:
                    delay_s = float(
                        self.get_parameter(
                            "post_detection_delay_s"
                        ).value
                    )
                    if delay_s > 0.0:
                        time.sleep(delay_s)
                    break
                time.sleep(0.05)
        finally:
            self.control_publisher.publish(Int32(data=0))

        if detection is None:
            response.success = False
            response.message = (
                f"no vision data received within {timeout_s:.1f} s"
            )
            return response
        if labels and not self._detection_matches_labels(
            detection, labels
        ):
            response.success = False
            response.message = (
                "vision result does not contain requested labels: "
                + ", ".join(labels)
            )
            return response

        with self._lock:
            self._cache_detection(key, detection)
        if not point_names:
            point_names = (
                ["left", "right"]
                if motion_mode == 1
                else ["center"]
            )
        return self._fill_selected_response(
            response, key, point_names, motion_mode
        )

    def _handle_get_target(
        self,
        request: GetVisionTarget.Request,
        response: GetVisionTarget.Response,
    ) -> GetVisionTarget.Response:
        key = request.key.strip()
        point_names = self._normalize_strings(request.point_names)
        motion_mode = self._normalize_motion_mode(request.motion_mode)
        if point_names:
            return self._fill_selected_response(
                response, key, point_names, motion_mode
            )

        with self._lock:
            poses = self.pose_cache.get(key)
            if poses is not None:
                response.success = True
                response.message = "ok"
                response.left_pose = copy.deepcopy(poses[0])
                response.right_pose = copy.deepcopy(poses[1])
                return response
        response.success = False
        response.message = f"no cached pose for key {key!r}"
        return response

    def _fill_selected_response(
        self,
        response: GetVisionTarget.Response,
        key: str,
        point_names: Sequence[str],
        motion_mode: int,
    ) -> GetVisionTarget.Response:
        with self._lock:
            selected = self._select_cached_points(
                key, list(point_names), motion_mode
            )
            if selected is None:
                response.success = False
                response.message = (
                    f"no selected pose for key {key!r}, "
                    f"points={list(point_names)}, mode={motion_mode}"
                )
                return response
            left_pose, right_pose, cache_key = selected
            self.pose_cache[cache_key] = (
                copy.deepcopy(left_pose),
                copy.deepcopy(right_pose),
            )

        response.success = True
        response.message = "ok"
        response.left_pose = left_pose
        response.right_pose = right_pose
        return response

    def _cache_detection(
        self, key: str, detection: Dict[str, Any]
    ) -> None:
        self.detection_cache[key] = copy.deepcopy(detection)
        point_map: Dict[str, Pose] = {}
        raw_points = detection.get("points", {})
        if not isinstance(raw_points, dict):
            raw_points = {}

        for scalar_name in (
            "yaw_rad",
            "yaw_deg",
            "x_offset",
            "y_offset",
            "z_offset",
        ):
            if scalar_name not in detection:
                continue
            try:
                point_map[scalar_name] = self._make_pose(
                    float(detection[scalar_name]), 0.0, 0.0
                )
            except (TypeError, ValueError):
                self.get_logger().warning(
                    f"invalid vision scalar {scalar_name!r} "
                    f"for key {key!r}"
                )

        if all(
            axis in detection for axis in ("base_x", "base_y", "base_z")
        ):
            try:
                point_map["base"] = self._make_pose(
                    float(detection["base_x"]),
                    float(detection["base_y"]),
                    float(detection["base_z"]),
                )
            except (TypeError, ValueError):
                self.get_logger().warning(
                    f"invalid base coordinates for key {key!r}"
                )

        for name, point in raw_points.items():
            if not isinstance(point, dict) or not all(
                axis in point for axis in ("x", "y", "z")
            ):
                self.get_logger().warning(
                    f"ignored invalid vision point {name!r} "
                    f"for key {key!r}"
                )
                continue
            try:
                point_map[str(name)] = self._make_pose(
                    float(point["x"]),
                    float(point["y"]),
                    float(point["z"]),
                )
            except (TypeError, ValueError):
                self.get_logger().warning(
                    f"ignored non-numeric vision point {name!r} "
                    f"for key {key!r}"
                )
        self.point_cache[key] = point_map
        center = point_map.get("center")
        if center is not None:
            self.pose_cache[CENTER_CACHE_PREFIX + key] = (
                copy.deepcopy(center),
                copy.deepcopy(center),
            )

    def _select_cached_points(
        self, key: str, names: List[str], motion_mode: int
    ) -> Optional[Tuple[Pose, Pose, str]]:
        point_map = self.point_cache.get(key)
        if not point_map:
            return None

        if motion_mode in (2, 3, 4, 5, 6):
            if len(names) != 1 or names[0] not in point_map:
                return None
            pose = copy.deepcopy(point_map[names[0]])
            return (
                pose,
                copy.deepcopy(pose),
                self._target_cache_key(key, names, motion_mode),
            )

        if motion_mode != 1 or len(names) < 2:
            return None
        selected_names = names[:2]
        left_raw = point_map.get(selected_names[0])
        right_raw = point_map.get(selected_names[1])
        if left_raw is None or right_raw is None:
            return None
        left_pose, right_pose = self._apply_offset(
            left_raw, right_raw, self.active_offset_id
        )
        return (
            left_pose,
            right_pose,
            self._target_cache_key(key, selected_names, motion_mode),
        )

    def _apply_offset(
        self, left_pose: Pose, right_pose: Pose, offset_id: str
    ) -> Tuple[Pose, Pose]:
        offset = self.offset_map.get(
            offset_id, self.offset_map["default"]
        )
        left = copy.deepcopy(left_pose)
        right = copy.deepcopy(right_pose)
        for pose, prefix in ((left, "left"), (right, "right")):
            pose.position.x += float(offset[f"{prefix}_dx"])
            pose.position.y += float(offset[f"{prefix}_dy"])
            pose.position.z += float(offset[f"{prefix}_dz"])
            pose.orientation.x = float(offset[f"{prefix}_ox"])
            pose.orientation.y = float(offset[f"{prefix}_oy"])
            pose.orientation.z = float(offset[f"{prefix}_oz"])
            pose.orientation.w = float(offset[f"{prefix}_ow"])
        return left, right

    def _handle_set_offset(
        self,
        request: SetVisionOffset.Request,
        response: SetVisionOffset.Response,
    ) -> SetVisionOffset.Response:
        offset_id = request.offset_id.strip() or "default"
        entry = {
            field: float(getattr(request, field))
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
            )
        }
        for prefix in ("left", "right"):
            quaternion = [
                entry[f"{prefix}_ox"],
                entry[f"{prefix}_oy"],
                entry[f"{prefix}_oz"],
                entry[f"{prefix}_ow"],
            ]
            norm = math.sqrt(sum(value * value for value in quaternion))
            if norm <= 1.0e-9:
                response.success = False
                response.message = (
                    f"{prefix} orientation quaternion must be non-zero"
                )
                return response
            for suffix, value in zip(("ox", "oy", "oz", "ow"), quaternion):
                entry[f"{prefix}_{suffix}"] = value / norm

        with self._lock:
            self.offset_map[offset_id] = entry
            self.active_offset_id = offset_id
            self.pose_cache.clear()
        response.success = True
        response.message = f"vision offset {offset_id!r} activated"
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _make_pose(x: float, y: float, z: float) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _target_cache_key(
        key: str, point_names: Sequence[str], motion_mode: int
    ) -> str:
        return (
            f"{key}|mode={int(motion_mode)}|"
            f"points={','.join(point_names)}"
        )

    @classmethod
    def _detection_matches_labels(
        cls, detection: Dict[str, Any], expected: Sequence[str]
    ) -> bool:
        found = {
            label.lower()
            for label in cls._collect_labels(detection)
        }
        # The reference implementation permits detectors without labels.
        if not found:
            return True
        return all(label.lower() in found for label in expected)

    @classmethod
    def _collect_labels(cls, value: Any) -> List[str]:
        label_keys = {
            "label",
            "labels",
            "class",
            "class_name",
            "name",
            "tag",
            "tags",
            "category",
        }
        labels: List[str] = []
        if isinstance(value, dict):
            for key, item in value.items():
                if key in label_keys:
                    labels.extend(cls._label_values(item))
                labels.extend(cls._collect_labels(item))
        elif isinstance(value, list):
            for item in value:
                labels.extend(cls._collect_labels(item))
        return labels

    @classmethod
    def _label_values(cls, value: Any) -> List[str]:
        if isinstance(value, (str, int, float)):
            return [str(value)]
        if isinstance(value, list):
            result: List[str] = []
            for item in value:
                result.extend(cls._label_values(item))
            return result
        if isinstance(value, dict):
            result = []
            for item in value.values():
                result.extend(cls._label_values(item))
            return result
        return []

    @staticmethod
    def _identity_offset() -> Dict[str, float]:
        return {
            "left_dx": 0.0,
            "left_dy": 0.0,
            "left_dz": 0.0,
            "right_dx": 0.0,
            "right_dy": 0.0,
            "right_dz": 0.0,
            "left_ox": 0.0,
            "left_oy": 0.0,
            "left_oz": 0.0,
            "left_ow": 1.0,
            "right_ox": 0.0,
            "right_oy": 0.0,
            "right_oz": 0.0,
            "right_ow": 1.0,
        }


def main(arguments=None) -> None:
    rclpy.init(args=arguments)
    node = VisionTargetServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
