#!/usr/bin/env python3
"""Detect a box and publish its left/right grasp points in the robot base frame."""

import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from box_detection_interfaces.msg import BoxGrabPoints
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, Int32
from ultralytics import YOLO


DEFAULT_MODEL_PATH = os.path.join(
    get_package_share_directory("aruco_scanner"), "models", "best.pt"
)


def clamp_uv(u, v, width, height):
    return (
        int(np.clip(u, 0, width - 1)),
        int(np.clip(v, 0, height - 1)),
    )


def pixel_to_3d(u, v, depth_image, depth_encoding, intrinsics):
    """Convert an aligned depth pixel to a point in the camera optical frame."""
    height, width = depth_image.shape[:2]
    u, v = clamp_uv(u, v, width, height)
    raw_depth = float(depth_image[v, u])
    depth_m = raw_depth if depth_encoding == "32FC1" else raw_depth / 1000.0
    if not np.isfinite(depth_m) or depth_m <= 0.001:
        return None

    fx, fy, cx, cy = intrinsics
    if fx <= 0.0 or fy <= 0.0:
        return None
    return np.array([
        (u - cx) * depth_m / fx,
        (v - cy) * depth_m / fy,
        depth_m,
    ])


def transform_point(point, transform):
    if point is None:
        return None
    return (transform @ np.append(point, 1.0))[:3]


def line_intersection(p1, p2, p3, p4):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denominator) < 1e-8:
        return None
    scale = ((x1 - x3) * (y3 - y4) -
             (y1 - y3) * (x3 - x4)) / denominator
    return np.array([
        x1 + scale * (x2 - x1),
        y1 + scale * (y2 - y1),
    ], dtype=np.float32)


def offset_edge(point_0, point_1, distance):
    vector = point_1 - point_0
    length = np.linalg.norm(vector)
    if length < 1e-6:
        return point_0, point_1
    offset = np.array([vector[1], -vector[0]]) * (distance / length)
    return point_0 + offset, point_1 + offset


def offset_quad(points, offsets):
    top_left, top_right, bottom_right, bottom_left = points
    top = offset_edge(top_left, top_right, offsets[0])
    right = offset_edge(top_right, bottom_right, offsets[1])
    bottom = offset_edge(bottom_right, bottom_left, offsets[2])
    left = offset_edge(bottom_left, top_left, offsets[3])
    result = [
        line_intersection(left[1], left[0], top[0], top[1]),
        line_intersection(top[0], top[1], right[0], right[1]),
        line_intersection(right[0], right[1], bottom[0], bottom[1]),
        line_intersection(bottom[0], bottom[1], left[1], left[0]),
    ]
    return points if any(point is None for point in result) else np.array(result)


def order_quad(points):
    point_sum = points.sum(axis=1)
    point_diff = np.diff(points, axis=1).reshape(-1)
    return np.array([
        points[np.argmin(point_sum)],
        points[np.argmin(point_diff)],
        points[np.argmax(point_sum)],
        points[np.argmax(point_diff)],
    ], dtype=np.float32)


class BoxDualGrabNode(Node):
    def __init__(self):
        super().__init__("box_dual_grab_node")
        self._declare_parameters()

        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.depth_encoding = ""
        self.intrinsics = None
        self.last_color_stamp = None
        self.start_time = time.monotonic()
        self.detection_enabled = False

        model_path = self.get_parameter("model_path").value
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"YOLO model does not exist: {model_path}")
        self.get_logger().info(f"Loading box model: {model_path}")
        self.model = YOLO(model_path)

        self.transform = np.asarray(
            self.get_parameter("extrinsic_matrix").value, dtype=float
        ).reshape(4, 4)
        self.edge_offsets = np.asarray(
            self.get_parameter("edge_offsets").value, dtype=float
        )
        self.gui_enabled = bool(
            self.get_parameter("enable_gui").value
            and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        )
        if self.get_parameter("enable_gui").value and not self.gui_enabled:
            self.get_logger().warn("No graphical display; box detection GUI disabled")

        self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self.color_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value,
            self.depth_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self.camera_info_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Int32, self.get_parameter("enable_topic").value,
            self.enable_callback, 10
        )

        self.left_pub = self.create_publisher(
            PointStamped, self.get_parameter("left_point_topic").value, 10
        )
        self.right_pub = self.create_publisher(
            PointStamped, self.get_parameter("right_point_topic").value, 10
        )
        self.angle_pub = self.create_publisher(
            Float32, self.get_parameter("angle_topic").value, 10
        )
        self.combined_pub = self.create_publisher(
            BoxGrabPoints, self.get_parameter("combined_topic").value, 10
        )

        rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.process_frame)
        self.get_logger().info(
            "Box detector started; send 1 to /box/enable to start detection"
        )

    def _declare_parameters(self):
        self.declare_parameters("", [
            ("model_path", DEFAULT_MODEL_PATH),
            ("image_topic", "/camera/color/image_raw"),
            ("depth_topic", "/camera/depth/image_rect_raw"),
            ("camera_info_topic", "/camera/color/camera_info"),
            ("enable_topic", "/box/enable"),
            ("left_point_topic", "/box/grab_point/left"),
            ("right_point_topic", "/box/grab_point/right"),
            ("angle_topic", "/box/angle_deg"),
            ("combined_topic", "/box_grab_points"),
            ("frame_id", "base_link"),
            ("publish_rate", 10.0),
            ("start_delay", 2.0),
            ("confidence_threshold", 0.15),
            ("minimum_contour_area", 600.0),
            ("enable_gui", True),
            ("edge_offsets", [3.0, 3.0, 3.0, 3.0]),
            ("extrinsic_matrix", [
                -0.024283, -0.763799, 0.644997, 0.066737,
                -0.996673, 0.068711, 0.043844, 0.020982,
                -0.077807, -0.641787, -0.762926, 0.183970,
                0.0, 0.0, 0.0, 1.0,
            ]),
        ])

    def color_callback(self, message):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(message, "bgr8")
            self.last_color_stamp = message.header.stamp
        except CvBridgeError as error:
            self.get_logger().warn(f"Color conversion failed: {error}")

    def depth_callback(self, message):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="passthrough"
            )
            self.depth_encoding = message.encoding
        except CvBridgeError as error:
            self.get_logger().warn(f"Depth conversion failed: {error}")

    def camera_info_callback(self, message):
        self.intrinsics = (
            float(message.k[0]), float(message.k[4]),
            float(message.k[2]), float(message.k[5]),
        )

    def enable_callback(self, message):
        if message.data == 1 and not self.detection_enabled:
            self.detection_enabled = True
            self.start_time = time.monotonic()
            self.get_logger().info("Box detection enabled")
        elif message.data == 0 and self.detection_enabled:
            self.detection_enabled = False
            self.get_logger().info("Box detection disabled")
        elif message.data not in (0, 1):
            self.get_logger().warn(
                f"Invalid /box/enable value {message.data}; use 1 or 0"
            )

    def process_frame(self):
        if not self.detection_enabled:
            return
        if time.monotonic() - self.start_time < \
                float(self.get_parameter("start_delay").value):
            return
        if self.color_image is None or self.depth_image is None or \
                self.intrinsics is None:
            self.get_logger().warn(
                "Waiting for color, aligned depth and CameraInfo",
                throttle_duration_sec=2.0,
            )
            return

        image = self.color_image.copy()
        result = self.model.predict(
            source=image,
            imgsz=640,
            conf=float(self.get_parameter("confidence_threshold").value),
            retina_masks=True,
            verbose=False,
            device="cpu",
        )[0]
        detection = self._extract_points(result, image)
        if detection is not None:
            left_point, right_point, angle_deg, display = detection
            self.publish_detection(left_point, right_point, angle_deg)
        else:
            display = image
        self.show_image(display)

    def _extract_points(self, result, image):
        if result.masks is None or result.boxes is None:
            return None
        height, width = image.shape[:2]
        candidates = []
        for index in range(len(result.masks.data)):
            class_id = int(result.boxes.cls[index].item())
            if class_id != 0:
                continue
            confidence = float(result.boxes.conf[index].item())
            candidates.append((confidence, index))
        if not candidates:
            return None

        _, index = max(candidates)
        mask = result.masks.data[index].cpu().numpy()
        mask = cv2.resize(mask, (width, height))
        mask = (mask > 0.5).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < \
                float(self.get_parameter("minimum_contour_area").value):
            return None

        approximation = cv2.approxPolyDP(
            contour, 0.03 * cv2.arcLength(contour, True), True
        )
        if len(approximation) != 4:
            approximation = cv2.boxPoints(cv2.minAreaRect(contour))
        quad = order_quad(np.asarray(approximation).reshape(4, 2).astype(np.float32))
        quad = offset_quad(quad, self.edge_offsets)
        top_left, top_right, bottom_right, bottom_left = quad
        left_uv = clamp_uv(
            *(0.5 * (top_left + bottom_left)), width, height
        )
        right_uv = clamp_uv(
            *(0.5 * (top_right + bottom_right)), width, height
        )
        left_camera = pixel_to_3d(
            *left_uv, self.depth_image, self.depth_encoding, self.intrinsics
        )
        right_camera = pixel_to_3d(
            *right_uv, self.depth_image, self.depth_encoding, self.intrinsics
        )
        left_base = transform_point(left_camera, self.transform)
        right_base = transform_point(right_camera, self.transform)
        if left_base is None or right_base is None:
            return None

        edge = top_right - top_left
        angle_deg = float(np.degrees(np.arctan2(edge[1], edge[0])))
        display = image.copy()
        cv2.polylines(display, [quad.astype(int)], True, (0, 255, 255), 2)
        cv2.circle(display, left_uv, 6, (255, 0, 0), -1)
        cv2.circle(display, right_uv, 6, (0, 0, 255), -1)
        cv2.putText(
            display, "LEFT", (left_uv[0] + 8, left_uv[1]),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2
        )
        cv2.putText(
            display, "RIGHT", (right_uv[0] + 8, right_uv[1]),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2
        )
        return left_base, right_base, angle_deg, display

    def publish_detection(self, left, right, angle_deg):
        stamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter("frame_id").value
        for publisher, point in (
            (self.left_pub, left), (self.right_pub, right)
        ):
            message = PointStamped()
            message.header.stamp = stamp
            message.header.frame_id = frame_id
            message.point.x = float(point[0])
            message.point.y = float(point[1])
            message.point.z = float(point[2])
            publisher.publish(message)

        angle_message = Float32()
        angle_message.data = angle_deg
        self.angle_pub.publish(angle_message)

        combined = BoxGrabPoints()
        combined.header.stamp = stamp
        combined.header.frame_id = frame_id
        combined.left_center.x = float(left[0])
        combined.left_center.y = float(left[1])
        combined.left_center.z = float(left[2])
        combined.right_center.x = float(right[0])
        combined.right_center.y = float(right[1])
        combined.right_center.z = float(right[2])
        combined.angle_deg = float(angle_deg)
        self.combined_pub.publish(combined)

    def show_image(self, image):
        if not self.gui_enabled:
            return
        cv2.imshow("Box Left/Right Grab Detection", image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()

    def close(self):
        if self.gui_enabled:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BoxDualGrabNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        # ROS2 Humble may raise this conversion error when SIGINT arrives while
        # a subscription is being taken from the executor.
        if "Unable to convert call argument" not in str(error):
            raise
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
