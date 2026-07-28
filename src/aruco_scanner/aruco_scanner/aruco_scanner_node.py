#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pyrealsense2 as rs
import cv2
import numpy as np
import os
import time

# ===================== 参数区 =====================
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
COLOR_FPS = 30
FRAME_TIMEOUT = 1000
DEDUPLICATE_INTERVAL = 3.0
WINDOW_NAME = "D405 ArUco Scan"
# ==================================================

def create_aruco_detector(dictionary_id):
    """创建兼容 OpenCV 旧版和新版 API 的 ArUco 检测器。"""
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    else:
        dictionary = cv2.aruco.Dictionary_get(dictionary_id)

    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    detector = None
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    return dictionary, parameters, detector


class ArucoScannerNode(Node):
    def __init__(self):
        super().__init__("aruco_scanner_node")
        self.get_logger().info("ArUco 扫码节点启动")

        self.enable_detection = False  # 默认关闭
        self.control_sub = self.create_subscription(
            Int32,
            "/aruco/enable",
            self.control_callback,
            10
        )

        # 发布识别到的 ID
        self.pub_aruco_id = self.create_publisher(Int32, "/aruco/id", 10)
        

        # RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, COLOR_FPS)

        # ArUco
        self.aruco_dict, self.aruco_params, self.aruco_detector = \
            create_aruco_detector(cv2.aruco.DICT_6X6_250)

        # 去重缓存
        self.barcode_cache = {}

        # SSH/无桌面环境中自动禁用窗口，避免 Qt 直接中止进程。
        self.enable_gui = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if self.enable_gui:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        else:
            self.get_logger().warn("未检测到图形显示环境，已禁用图像窗口")

        # 启动相机
        try:
            self.pipeline.start(self.config)
            self.get_logger().info("D405 相机启动成功")
        except Exception as e:
            self.get_logger().error(f"相机启动失败: {str(e)}")
            raise

        # 创建定时器循环（ROS2 标准做法，不阻塞 spin）
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def control_callback(self, msg: Int32):
        if msg.data == 1:
            self.enable_detection = True
            self.get_logger().info("已开启 ArUco 识别与发布")
        elif msg.data == 0:
            self.enable_detection = False
            self.get_logger().info("已关闭 ArUco 识别与发布")

    def is_duplicate(self, marker_id):
        now = time.time()
        # 清理过期
        expired = [k for k, v in self.barcode_cache.items() if now - v > DEDUPLICATE_INTERVAL]
        for k in expired:
            del self.barcode_cache[k]

        mid = str(marker_id)
        if mid in self.barcode_cache:
            return True
        self.barcode_cache[mid] = now
        return False

    def timer_callback(self):
        if not self.enable_detection:
            try:
                frames = self.pipeline.wait_for_frames(FRAME_TIMEOUT)
                color_frame = frames.get_color_frame()
                if color_frame:
                    img = np.asanyarray(color_frame.get_data())
                    if self.show_image(img):
                        self.get_logger().info("按下 q，准备退出")
                        rclpy.shutdown()
            except RuntimeError:
                pass
            return

        try:
            frames = self.pipeline.wait_for_frames(FRAME_TIMEOUT)
        except RuntimeError:
            return

        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        img = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 检测
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            for mid in ids.flatten():
                if not self.is_duplicate(mid):
                    self.get_logger().info(f"检测到 ArUco ID: {mid}")
                    msg = Int32()
                    msg.data = int(mid)
                    self.pub_aruco_id.publish(msg)

        # 显示 + 退出 q 键
        if self.show_image(img):
            self.get_logger().info("按下 q，准备退出")
            rclpy.shutdown()

    def show_image(self, image):
        if not self.enable_gui:
            return False
        cv2.imshow(WINDOW_NAME, image)
        return cv2.waitKey(1) & 0xFF == ord("q")

    def close(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.enable_gui:
            cv2.destroyAllWindows()
        self.get_logger().info("资源已释放")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoScannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
