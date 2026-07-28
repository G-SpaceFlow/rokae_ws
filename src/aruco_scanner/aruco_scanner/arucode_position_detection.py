import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
import os
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError


def create_aruco_detector(dictionary):
    if hasattr(aruco, "DetectorParameters"):
        parameters = aruco.DetectorParameters()
    else:
        parameters = aruco.DetectorParameters_create()
    if hasattr(aruco, "ArucoDetector"):
        return aruco.ArucoDetector(dictionary, parameters)
    return None


def estimate_marker_poses(corners, marker_size, camera_matrix, dist_coeffs):
    """使用 solvePnP 解算位姿，兼容已移除 estimatePoseSingleMarkers 的 OpenCV。"""
    half = marker_size / 2.0
    object_points = np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)
    rvecs, tvecs = [], []
    for marker_corners in corners:
        image_points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            rvec = np.full((3, 1), np.nan)
            tvec = np.full((3, 1), np.nan)
        rvecs.append(rvec.reshape(1, 3))
        tvecs.append(tvec.reshape(1, 3))
    return np.asarray(rvecs), np.asarray(tvecs)


class ArucoToolPoseNode(Node):
    def __init__(self):
        super().__init__('aruco_tool_pose_node')

        # ================= ROS2 参数 =================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_rate', 10),
                ('enable_gui', True),  # 打开显示
                # 相机话题名称
                ('image_topic', '/camera/color/image_raw'),
                ('camera_info_topic', '/camera/color/camera_info'),
                # 外参矩阵：相机 -> 机器人基坐标系
                ('extrinsic_matrix', [
                   -0.094428, -0.741979,  0.66374,   0.072383,
                   -0.995499,  0.075759, -0.056937,  0.054335,
                   -0.008039, -0.666129, -0.745793,  0.130817,
                    0.0,        0.0 ,       0.0  ,      1.0
                ]),
                ('smooth_window_size', 3),
                ('tool_offset_x', 0.0),
                ('tool_offset_y', 0.0),
                ('tool_offset_z', 0.0),
                ('enable_tool_compensation', True),
                ('tool_compensation_quaternion', [0.0, 0.707, 0.0, 0.707]),
            ]
        )

        # 读取参数
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_gui = self.get_parameter('enable_gui').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        extrinsic_params = self.get_parameter('extrinsic_matrix').value
        self.tool_offset_x = self.get_parameter('tool_offset_x').value
        self.tool_offset_y = self.get_parameter('tool_offset_y').value
        self.tool_offset_z = self.get_parameter('tool_offset_z').value
        self.enable_tool_compensation = self.get_parameter('enable_tool_compensation').value
        self.tool_compensation_q = self.get_parameter('tool_compensation_quaternion').value

        # 外参矩阵：相机 -> 机器人基坐标系
        self.T_cam2base = np.array(extrinsic_params).reshape(4, 4)
        self.get_logger().info(f"\n外参矩阵 T_cam2base 已加载:\n{self.T_cam2base}")

        # ================= ArUco 配置【兼容全版本，对标标定代码写法】 =================
        self.ARUCO_DICT = aruco.DICT_4X4_50
        self.MARKER_SIZE = 0.03  # 3cm
        self.aruco_dict = aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self.aruco_detector = create_aruco_detector(self.aruco_dict)
        self.gui_available = bool(
            self.enable_gui
            and (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        )
        if self.enable_gui and not self.gui_available:
            self.get_logger().warn("未检测到图形显示环境，已禁用图像窗口")

        # ================= CV Bridge & 图像缓存 =================
        self.bridge = CvBridge()
        self.latest_color_img = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # ================= 订阅相机话题 =================
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        self.enable_detection = False  # 默认关闭
        self.waiting_for_enable_reported = False
        self.control_sub = self.create_subscription(
            Int32,
            "/aruco/enable",
            self.control_callback,
            10
        )

        # ================= 刀具补偿 =================
        self.tool_offset = np.array([self.tool_offset_x, self.tool_offset_y, self.tool_offset_z])
        self.T_aru2tool = self.build_tool_compensation_matrix()

        # ================= ROS 发布 =================
        self.pose_pub = self.create_publisher(PoseStamped, '/tool/pose', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info("报告：ArUco 刀具位姿解算节点已启动！等待相机话题...")

    def camera_info_callback(self, msg: CameraInfo):
        """接收相机内参"""
        if self.camera_info_received:
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d).reshape(-1, 1)
        self.camera_info_received = True
        self.get_logger().info("报告：相机内参从CameraInfo话题接收完成！")

    def image_callback(self, msg: Image):
        """缓存最新彩色图像"""
        try:
            # 转换为BGR图像（realsense通常输出bgr8）
            self.latest_color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"图像转换失败: {str(e)}")

    def control_callback(self, msg: Int32):
        if msg.data == 1:
            was_enabled = self.enable_detection
            self.enable_detection = True
            self.waiting_for_enable_reported = False
            if not was_enabled:
                self.get_logger().info("已开启 ArUco 识别与发布")
        elif msg.data == 0:
            was_enabled = self.enable_detection
            self.enable_detection = False
            self.waiting_for_enable_reported = False
            if was_enabled:
                self.get_logger().info("已关闭 ArUco 识别与发布")

    # ================= 工具补偿矩阵 =================
    def build_tool_compensation_matrix(self):
        T = np.eye(4)
        T[:3, 3] = self.tool_offset
        if self.enable_tool_compensation:
            qx, qy, qz, qw = self.tool_compensation_q
            T[:3, :3] = np.array([
                [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qw*qz,     2*qx*qz + 2*qw*qy],
                [2*qx*qy + 2*qw*qz,     1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qw*qx],
                [2*qx*qz - 2*qw*qy,     2*qy*qz + 2*qw*qx,     1 - 2*qx*qx - 2*qy*qy]
            ])
        return T

    # ================= 旋转矩阵 -> RPY =================
    def rot2rpy(self, R):
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        if sy < 1e-6:
            rx = np.arctan2(-R[1,2], R[1,1])
            ry = np.arctan2(-R[2,0], sy)
            rz = 0.0
        else:
            rx = np.arctan2(R[2,1], R[2,2])
            ry = np.arctan2(-R[2,0], sy)
            rz = np.arctan2(R[1,0], R[0,0])
        return np.array([rx, ry, rz])

    # ================= RPY -> 四元数 =================
    def rpy2quat(self, rpy):
        rx, ry, rz = rpy
        qx = np.sin(rx/2) * np.cos(ry/2) * np.cos(rz/2) - np.cos(rx/2) * np.sin(ry/2) * np.sin(rz/2)
        qy = np.cos(rx/2) * np.sin(ry/2) * np.cos(rz/2) + np.sin(rx/2) * np.cos(ry/2) * np.sin(rz/2)
        qz = np.cos(rx/2) * np.cos(ry/2) * np.sin(rz/2) - np.sin(rx/2) * np.sin(ry/2) * np.cos(rz/2)
        qw = np.cos(rx/2) * np.cos(ry/2) * np.cos(rz/2) + np.sin(rx/2) * np.sin(ry/2) * np.sin(rz/2)
        return np.array([qx, qy, qz, qw])

    # ================= 主循环 =================
    def timer_callback(self):
        if not self.enable_detection:
            if not self.waiting_for_enable_reported:
                self.get_logger().info(
                    "报告：正在运行，等待 enable 指令"
                )
                self.waiting_for_enable_reported = True
            return
        
        self.get_logger().info("报告：正在运行，等待 ArUco 码...", throttle_duration_sec=1)

        # 校验依赖条件
        if not self.camera_info_received:
            self.get_logger().warn("尚未收到CameraInfo，等待相机内参...", throttle_duration_sec=2)
            return
        if self.latest_color_img is None:
            self.get_logger().warn("尚未收到图像话题数据...", throttle_duration_sec=2)
            return

        try:
            color_img = self.latest_color_img.copy()
            gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

            if self.aruco_detector is not None:
                corners, ids, _ = self.aruco_detector.detectMarkers(gray)
            else:
                corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
            display_img = color_img.copy()

            if ids is not None:
                rvecs, tvecs = estimate_marker_poses(
                    corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs
                )

                for i in range(len(ids)):
                    rvec = rvecs[i]
                    tvec = tvecs[i]

                    # 1. 相机 -> ArUco 码
                    R_cam2aru, _ = cv2.Rodrigues(rvec)
                    T_cam2aru = np.eye(4)
                    T_cam2aru[:3, :3] = R_cam2aru
                    T_cam2aru[:3, 3] = tvec.reshape(3)

                    # 2. 码 -> 刀具
                    T_cam2tool = T_cam2aru @ self.T_aru2tool

                    # 3. 相机 -> 基座
                    T_base2tool = self.T_cam2base @ T_cam2tool

                    # 提取位姿
                    xyz = T_base2tool[:3, 3]
                    rpy = self.rot2rpy(T_base2tool[:3, :3])
                    quat = self.rpy2quat(rpy)

                    # 发布话题
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "base_link"
                    pose_msg.pose.position.x = float(xyz[0])
                    pose_msg.pose.position.y = float(xyz[1])
                    pose_msg.pose.position.z = float(xyz[2])
                    pose_msg.pose.orientation.x = float(quat[0])
                    pose_msg.pose.orientation.y = float(quat[1])
                    pose_msg.pose.orientation.z = float(quat[2])
                    pose_msg.pose.orientation.w = float(quat[3])
                    self.pose_pub.publish(pose_msg)

                    self.get_logger().info(f"""
                    刀具基坐标系位姿
                    XYZ: {xyz.round(4)} m
                    RPY: {np.rad2deg(rpy).round(2)} °
                    """)

                    # 绘制标记与坐标轴
                    aruco.drawDetectedMarkers(display_img, corners, ids)
                    cv2.drawFrameAxes(display_img, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

                    text1 = f"X:{xyz[0]:.3f} Y:{xyz[1]:.3f} Z:{xyz[2]:.3f}"
                    text2 = f"Rx:{np.rad2deg(rpy[0]):.1f} Ry:{np.rad2deg(rpy[1]):.1f} Rz:{np.rad2deg(rpy[2]):.1f}"
                    cv2.putText(display_img, text1, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                    cv2.putText(display_img, text2, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            # 显示图像窗口
            if self.gui_available:
                cv2.imshow("Knife Pose", display_img)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().warn(f"警告：帧处理失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoToolPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.gui_available:
            cv2.destroyAllWindows()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
