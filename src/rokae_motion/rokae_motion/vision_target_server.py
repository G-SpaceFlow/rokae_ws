#!/usr/bin/env python3
"""ROS 2 detector trigger, target-pose cache and offset service.

This is the Rokae ROS 2 port of ``zj_robot_bt_action/bt_target_server.py``.
It stores JSON detections and ArUco PoseStamped results in memory and exposes
selected poses to behavior-tree actions. It does not run a detector or save
camera images.
"""

import copy
from functools import partial
import json
import math
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from box_detection_interfaces.msg import BoxGrabPoints
from geometry_msgs.msg import Pose, PoseStamped
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
ARUCO_CONTROL_TOPIC = "/aruco/enable"
ARUCO_POSE_TOPIC = "/tool/pose"
BOX_CONTROL_TOPIC = "/box/enable"
BOX_POINTS_TOPIC = "/box_grab_points"
TRIGGER_DETECT_SERVICE = "/bt_target_server/trigger_detect"
GET_TARGET_SERVICE = "/bt_target_server/get_target"
SET_OFFSET_SERVICE = "/bt_target_server/set_offset"
VISION_SOURCE_YOLO = "yolo"
VISION_SOURCE_ARUCO = "aruco"
VISION_SOURCE_BOX_GRAB_POINTS = "box_grab_points"


class VisionTargetServer(Node):
    """Cache detector results and return motion-mode target poses."""

    def __init__(self) -> None:
        super().__init__("bt_target_server")
        self._lock = threading.RLock()
        self._callbacks = ReentrantCallbackGroup()
        self._init_state()
        self._init_interfaces()
        self.get_logger().info(
            "vision target services ready: YOLO JSON, ArUco tool pose, "
            "and box left/right points; trigger_detect, get_target, "
            "set_offset"
        )

    def _init_state(self) -> None:
        self.pose_cache: Dict[str, Tuple[Pose, Pose]] = {}
        self.detection_cache: Dict[str, Dict[str, Any]] = {}
        self.point_cache: Dict[str, Dict[str, Pose]] = {}
        self.latest_front_detection: Optional[Dict[str, Any]] = None
        self.latest_wall_angle: Optional[Dict[str, Any]] = None
        self.latest_mode5_detection: Optional[Dict[str, Any]] = None
        self.latest_aruco_pose: Optional[PoseStamped] = None
        self.latest_aruco_poses: Dict[
            str, Optional[PoseStamped]
        ] = {}
        self.latest_box_points: Dict[
            str, Optional[BoxGrabPoints]
        ] = {}
        self.aruco_pose_subscriptions = {}
        self.aruco_control_publishers = {}
        self.box_point_subscriptions = {}
        self.box_control_publishers = {}
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
        self.declare_parameter(
            "aruco_control_topic", ARUCO_CONTROL_TOPIC
        )
        self.declare_parameter("aruco_pose_topic", ARUCO_POSE_TOPIC)
        self.declare_parameter("aruco_frame_id", "base_link")
        self.declare_parameter("box_frame_id", "base_link")
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
        aruco_control_topic = self.get_parameter(
            "aruco_control_topic"
        ).value
        aruco_pose_topic = self.get_parameter(
            "aruco_pose_topic"
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
        self.aruco_pose_subscription = self.create_subscription(
            PoseStamped,
            aruco_pose_topic,
            partial(
                self._aruco_pose_callback,
                topic=aruco_pose_topic,
            ),
            10,
            callback_group=self._callbacks,
        )
        self.control_publisher = self.create_publisher(
            Int32, control_topic, 10
        )
        self.aruco_control_publisher = self.create_publisher(
            Int32, aruco_control_topic, 10
        )
        self.aruco_pose_subscriptions[aruco_pose_topic] = (
            self.aruco_pose_subscription
        )
        self.aruco_control_publishers[aruco_control_topic] = (
            self.aruco_control_publisher
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

    def _aruco_pose_callback(
        self, message: PoseStamped, *, topic: str = ARUCO_POSE_TOPIC
    ) -> None:
        with self._lock:
            self.latest_aruco_pose = copy.deepcopy(message)
            self.latest_aruco_poses[topic] = copy.deepcopy(message)

    def _box_points_callback(
        self, message: BoxGrabPoints, *, topic: str
    ) -> None:
        with self._lock:
            self.latest_box_points[topic] = copy.deepcopy(message)

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

    @staticmethod
    def _normalize_source(value: Any) -> str:
        source = str(value or VISION_SOURCE_YOLO).strip().lower()
        if source not in (
            VISION_SOURCE_YOLO,
            VISION_SOURCE_ARUCO,
            VISION_SOURCE_BOX_GRAB_POINTS,
        ):
            raise ValueError(
                f"unsupported vision source {value!r}; "
                "use 'yolo', 'aruco', or 'box_grab_points'"
            )
        return source

    @staticmethod
    def _default_topics(source: str) -> Tuple[str, str]:
        if source == VISION_SOURCE_ARUCO:
            return ARUCO_CONTROL_TOPIC, ARUCO_POSE_TOPIC
        if source == VISION_SOURCE_BOX_GRAB_POINTS:
            return BOX_CONTROL_TOPIC, BOX_POINTS_TOPIC
        return VISION_CONTROL_TOPIC, VISION_POINTS_TOPIC

    def _reset_latest(
        self, source: str, trigger_value: int, echo_topic: str
    ) -> None:
        with self._lock:
            if source == VISION_SOURCE_ARUCO:
                self.latest_aruco_pose = None
                self.latest_aruco_poses[echo_topic] = None
            elif source == VISION_SOURCE_BOX_GRAB_POINTS:
                self.latest_box_points[echo_topic] = None
            elif trigger_value == 3:
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

    def _latest_aruco(
        self, topic: str = ARUCO_POSE_TOPIC
    ) -> Optional[PoseStamped]:
        with self._lock:
            return copy.deepcopy(self.latest_aruco_poses.get(topic))

    def _latest_box(
        self, topic: str = BOX_POINTS_TOPIC
    ) -> Optional[BoxGrabPoints]:
        with self._lock:
            return copy.deepcopy(self.latest_box_points.get(topic))

    def _ensure_typed_interfaces(
        self, source: str, pub_topic: str, echo_topic: str
    ):
        if source == VISION_SOURCE_ARUCO:
            if echo_topic not in self.aruco_pose_subscriptions:
                self.aruco_pose_subscriptions[echo_topic] = (
                    self.create_subscription(
                        PoseStamped,
                        echo_topic,
                        partial(
                            self._aruco_pose_callback,
                            topic=echo_topic,
                        ),
                        10,
                        callback_group=self._callbacks,
                    )
                )
            if pub_topic not in self.aruco_control_publishers:
                self.aruco_control_publishers[pub_topic] = (
                    self.create_publisher(Int32, pub_topic, 10)
                )
            return (
                self.aruco_control_publishers[pub_topic],
                partial(self._latest_aruco, echo_topic),
            )

        if echo_topic not in self.box_point_subscriptions:
            self.box_point_subscriptions[echo_topic] = (
                self.create_subscription(
                    BoxGrabPoints,
                    echo_topic,
                    partial(
                        self._box_points_callback,
                        topic=echo_topic,
                    ),
                    10,
                    callback_group=self._callbacks,
                )
            )
        if pub_topic not in self.box_control_publishers:
            self.box_control_publishers[pub_topic] = (
                self.create_publisher(Int32, pub_topic, 10)
            )
        return (
            self.box_control_publishers[pub_topic],
            partial(self._latest_box, echo_topic),
        )

    @staticmethod
    def _wait_for_subscriber(publisher, timeout_s: float = 2.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return publisher.get_subscription_count() > 0

    @staticmethod
    def _wait_for_latest(
        latest: Callable[[], Optional[Any]],
        timeout_s: float,
        post_detection_delay_s: float,
    ) -> Optional[Any]:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if latest() is not None:
                if post_detection_delay_s > 0.0:
                    time.sleep(post_detection_delay_s)
                # Read again after the settling delay so the cached value is
                # the newest frame, not the first frame that was detected.
                return latest()
            time.sleep(0.05)
        return None

    def _handle_trigger_detect(
        self,
        request: GetVisionTarget.Request,
        response: GetVisionTarget.Response,
    ) -> GetVisionTarget.Response:
        try:
            source = self._normalize_source(request.source)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            return response
        default_pub_topic, default_echo_topic = self._default_topics(source)
        pub_topic = request.pub_topic.strip() or default_pub_topic
        echo_topic = request.echo_topic.strip() or default_echo_topic
        key = request.key.strip() or "1-1-1-2"
        trigger_value = int(request.trigger_value)
        labels = self._normalize_strings(request.labels)
        point_names = self._normalize_strings(request.point_names)
        # Zero is the behavior-tree cache-only request. Non-zero values remain
        # supported here for older direct service callers.
        motion_mode = int(request.motion_mode)
        if motion_mode < 0 or motion_mode > 6:
            response.success = False
            response.message = "motion_mode must be 0 through 6"
            return response
        if (
            source == VISION_SOURCE_YOLO
            and motion_mode == 1
            and len(point_names) == 1
        ):
            motion_mode = 6 if trigger_value == 5 else 4

        if source == VISION_SOURCE_ARUCO:
            validation_error = self._validate_aruco_request(
                trigger_value, labels, point_names
            )
        elif source == VISION_SOURCE_BOX_GRAB_POINTS:
            validation_error = self._validate_box_request(
                trigger_value, labels, point_names
            )
        else:
            validation_error = ""
        if validation_error:
            response.success = False
            response.message = validation_error
            return response

        if source in (
            VISION_SOURCE_ARUCO,
            VISION_SOURCE_BOX_GRAB_POINTS,
        ):
            control_publisher, latest = self._ensure_typed_interfaces(
                source, pub_topic, echo_topic
            )
            if not self._wait_for_subscriber(control_publisher):
                detector = (
                    "aruco_tool_launch.py"
                    if source == VISION_SOURCE_ARUCO
                    else "big_box_detection_node"
                )
                response.success = False
                response.message = (
                    f"{pub_topic} has no subscriber; start {detector} "
                    "first"
                )
                return response

        self._reset_latest(source, trigger_value, echo_topic)
        if source == VISION_SOURCE_YOLO:
            self.labels_publisher.publish(
                String(data=json.dumps(labels, ensure_ascii=False))
            )
            control_publisher = self.control_publisher
            latest = partial(
                self._latest_for_trigger, trigger_value
            )
        control_publisher.publish(Int32(data=trigger_value))
        self.get_logger().info(
            f"vision trigger: source={source}, key={key}, "
            f"value={trigger_value}, "
            f"pub_topic={pub_topic}, echo_topic={echo_topic}, "
            f"points={point_names}"
        )

        timeout_s = float(
            self.get_parameter("detection_timeout_s").value
        )
        delay_s = float(
            self.get_parameter("post_detection_delay_s").value
        )
        try:
            detection = self._wait_for_latest(
                latest, timeout_s, delay_s
            )
        finally:
            control_publisher.publish(Int32(data=0))

        if detection is None:
            response.success = False
            response.message = (
                f"no {source} vision data received within "
                f"{timeout_s:.1f} s"
            )
            return response
        if source == VISION_SOURCE_ARUCO:
            return self._cache_aruco_result(
                response,
                key,
                detection,
                point_names,
                motion_mode,
            )
        if source == VISION_SOURCE_BOX_GRAB_POINTS:
            return self._cache_box_result(
                response,
                key,
                detection,
                point_names,
                motion_mode,
            )
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
        if motion_mode == 0:
            response.success = True
            response.message = "cached"
            return response
        if not point_names:
            point_names = (
                ["left", "right"]
                if motion_mode == 1
                else ["center"]
            )
        return self._fill_selected_response(
            response, key, point_names, motion_mode
        )

    @staticmethod
    def _validate_aruco_request(
        trigger_value: int,
        labels: Sequence[str],
        point_names: Sequence[str],
    ) -> str:
        if trigger_value != 1:
            return "ArUco trigger_value must be 1"
        if labels:
            return "ArUco source does not support labels"
        if len(point_names) != 1:
            return "ArUco points must contain exactly one point name"
        return ""

    @staticmethod
    def _validate_box_request(
        trigger_value: int,
        labels: Sequence[str],
        point_names: Sequence[str],
    ) -> str:
        if trigger_value != 1:
            return "box_grab_points trigger_value must be 1"
        if labels:
            return "box_grab_points source does not support labels"
        if len(point_names) != 2:
            return (
                "box_grab_points points must contain exactly two "
                "point names"
            )
        return ""

    def _cache_aruco_result(
        self,
        response: GetVisionTarget.Response,
        key: str,
        message: PoseStamped,
        point_names: Sequence[str],
        motion_mode: int,
    ) -> GetVisionTarget.Response:
        expected_frame = str(
            self.get_parameter("aruco_frame_id").value
        ).strip()
        actual_frame = message.header.frame_id.strip()
        if expected_frame and actual_frame != expected_frame:
            response.success = False
            response.message = (
                "ArUco pose frame mismatch: "
                f"expected={expected_frame!r}, received={actual_frame!r}"
            )
            return response
        try:
            pose = self._validated_pose_copy(message.pose)
        except ValueError as exc:
            response.success = False
            response.message = f"invalid ArUco pose: {exc}"
            return response

        point_name = point_names[0]
        with self._lock:
            self.detection_cache[key] = {
                "source": VISION_SOURCE_ARUCO,
                "frame_id": actual_frame,
                "point": point_name,
            }
            self.point_cache[key] = {point_name: pose}
        self.get_logger().info(
            "cached latest ArUco pose: "
            f"key={key}, point={point_name}, frame={actual_frame}, "
            f"xyz=({pose.position.x:.6f}, "
            f"{pose.position.y:.6f}, {pose.position.z:.6f})"
        )
        if motion_mode == 0:
            response.success = True
            response.message = "cached"
            return response
        return self._fill_selected_response(
            response, key, point_names, motion_mode
        )

    def _cache_box_result(
        self,
        response: GetVisionTarget.Response,
        key: str,
        message: BoxGrabPoints,
        point_names: Sequence[str],
        motion_mode: int,
    ) -> GetVisionTarget.Response:
        expected_frame = str(
            self.get_parameter("box_frame_id").value
        ).strip()
        actual_frame = message.header.frame_id.strip()
        if expected_frame and actual_frame != expected_frame:
            response.success = False
            response.message = (
                "box point frame mismatch: "
                f"expected={expected_frame!r}, received={actual_frame!r}"
            )
            return response

        values = (
            message.left_center.x,
            message.left_center.y,
            message.left_center.z,
            message.right_center.x,
            message.right_center.y,
            message.right_center.z,
            message.angle_deg,
        )
        if not all(math.isfinite(float(value)) for value in values):
            response.success = False
            response.message = "box points and angle_deg must be finite"
            return response

        left_pose = self._make_pose(*values[:3])
        right_pose = self._make_pose(*values[3:6])
        with self._lock:
            self.detection_cache[key] = {
                "source": VISION_SOURCE_BOX_GRAB_POINTS,
                "frame_id": actual_frame,
                "angle_deg": float(message.angle_deg),
                "points": list(point_names),
            }
            self.point_cache[key] = {
                point_names[0]: left_pose,
                point_names[1]: right_pose,
            }
        self.get_logger().info(
            "cached latest box grab points: "
            f"key={key}, frame={actual_frame}, "
            f"{point_names[0]}=({values[0]:.6f}, "
            f"{values[1]:.6f}, {values[2]:.6f}), "
            f"{point_names[1]}=({values[3]:.6f}, "
            f"{values[4]:.6f}, {values[5]:.6f}), "
            f"angle_deg={values[6]:.3f}"
        )
        if motion_mode == 0:
            response.success = True
            response.message = "cached"
            return response
        return self._fill_selected_response(
            response, key, point_names, motion_mode
        )

    @staticmethod
    def _validated_pose_copy(pose: Pose) -> Pose:
        result = copy.deepcopy(pose)
        values = [
            result.position.x,
            result.position.y,
            result.position.z,
            result.orientation.x,
            result.orientation.y,
            result.orientation.z,
            result.orientation.w,
        ]
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("position and orientation must be finite")
        quaternion = values[3:]
        norm = math.sqrt(
            sum(float(value) * float(value) for value in quaternion)
        )
        if norm <= 1.0e-9:
            raise ValueError("orientation quaternion must be non-zero")
        result.orientation.x /= norm
        result.orientation.y /= norm
        result.orientation.z /= norm
        result.orientation.w /= norm
        return result

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
