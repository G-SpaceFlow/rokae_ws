import threading
import os
import time
import yaml
import rclpy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_msgs.msg import Float32

from rclpy.node import Node
from cv_bridge import CvBridge
import pyrealsense2 as rs
from ament_index_python.packages import get_package_share_path

import numpy as np


def load_camera_config(yaml_path="/configs/camera.yaml"):
    with open(yaml_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def check_realsense_connected(target_serial):
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
            try:
                sn = dev.get_info(rs.camera_info.serial_number)
            except RuntimeError:
                continue
            if sn == target_serial:
                return True
    except RuntimeError:
        # 某些 Linux/RealSense 驱动组合在枚举设备时会抛出
        # VIDIOC_S_EXT_CTRLS。此时交给工作线程按序列号直接连接并重试。
        return None
    return False


rs_format_map = {
    "bgr8": rs.format.bgr8,
    "rgb8": rs.format.rgb8,
    "z16": rs.format.z16,
}
align = rs.align(rs.stream.color)


class MultiRealsenseNode(Node):
    def __init__(self):
        super().__init__("multi_realsense_node")
        self.bridge = CvBridge()

        # 配置文件路径
        pkg_path = get_package_share_path("realsense_multi_camera")
        cfg_path = pkg_path / "configs" / "camera.yaml"
        cfg_path = str(cfg_path)

        self.get_logger().info(f"cfg_path: {cfg_path}")
        self.camera_cfg = load_camera_config(cfg_path)["cameras"]
        
        # 自检
        self.online_cams = self._self_check()
        if not self.online_cams:
            self.get_logger().error("无可用相机")
            return
        

        # 为每个相机创建 1 个 publisher
        self.threads = []
        for name in self.online_cams:
            t = threading.Thread(target=self._camera_worker, args=(name, self.camera_cfg[name]), daemon=True)
            t.start()
            self.threads.append(t)

        self.get_logger().info("多相机节点启动完成")


    def _self_check(self):
        online = []
        self.get_logger().info("=== 相机自检 ===")
        for name, cfg in self.camera_cfg.items():
            enable = cfg["enable"]
            serial = cfg["serial"]
            if not enable:
                self.get_logger().info(f"{name}: 已禁用")
                continue
            ok = check_realsense_connected(serial)
            if ok is True:
                online.append(name)
                self.get_logger().info(f"{name}: SN {serial} 已连接")
            elif ok is None:
                online.append(name)
                self.get_logger().warn(
                    f"{name}: 设备枚举异常，交由工作线程按 SN {serial} 重试"
                )
            else:
                self.get_logger().warn(f"{name}: SN {serial} 未连接")
        return online

    def _camera_worker(self, cam_name, cam_cfg):
        # topic
        # topic_name = f"/{cam_name}/distance"
        # distance_pub = self.create_publisher(Float32, topic_name, 10)
        image_topic = f"/{cam_name}/color/image_raw"
        cam_info_topic = f"/{cam_name}/color/camera_info"

        image_pub = self.create_publisher(Image, image_topic, 10)
        info_pub = self.create_publisher(CameraInfo, cam_info_topic, 10)

        serial = cam_cfg["serial"]
        w = cam_cfg["width"]
        h = cam_cfg["height"]
        fps = cam_cfg["fps"]
        color_fmt = cam_cfg["color_format"]
        # depth_fmt = cam_cfg["depth_format"]
        # align_to_color = cam_cfg["align_to_color"]

        encoding = "bgr8" if color_fmt == "bgr8" else "rgb8"

        while rclpy.ok():
            pipeline = None
            try:
                self.get_logger().info(f"[{cam_name}] 尝试启动中...")
                pipeline = rs.pipeline()
                cfg = rs.config()
                cfg.enable_device(serial)
                cfg.enable_stream(rs.stream.color, w, h, rs_format_map[color_fmt], fps)
                # cfg.enable_stream(rs.stream.depth, w, h, rs_format_map[depth_fmt], fps)
                
                # 启动并获取流信息，用于构造内参
                profile = pipeline.start(cfg)
                color_profile = profile.get_stream(rs.stream.color)
                intr = color_profile.as_video_stream_profile().get_intrinsics()
                self.get_logger().info(f"[{cam_name}] 启动成功")

                # 构造 CameraInfo 消息（只构造一次）
                cam_info_msg = CameraInfo()
                cam_info_msg.header.frame_id = f"{cam_name}_color_optical_frame"
                cam_info_msg.width = intr.width
                cam_info_msg.height = intr.height
                cam_info_msg.distortion_model = "plumb_bob"
                cam_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # 无畸变

                # 相机内参矩阵 K
                cam_info_msg.k = [
                    intr.fx, 0.0, intr.ppx,
                    0.0, intr.fy, intr.ppy,
                    0.0, 0.0, 1.0
                ]
                # R 旋转矩阵（单位阵）
                cam_info_msg.r = [1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0]
                # P 投影矩阵
                cam_info_msg.p = [
                    intr.fx, 0.0, intr.ppx, 0.0,
                    0.0, intr.fy, intr.ppy, 0.0,
                    0.0, 0.0, 1.0, 0.0
                ]

                while rclpy.ok():
                    try:
                        frames = pipeline.wait_for_frames(timeout_ms=2000)    # 启动后要短暂等待相机
                        # aligned_frames = align.process(frames) if align_to_color else frames

                        # color_frame = aligned_frames.get_color_frame()
                        color_frame = frames.get_color_frame()
                        # depth_frame = aligned_frames.get_depth_frame()

                        # if not color_frame or not depth_frame:
                        #     continue
                        if not color_frame:
                            continue

                        # 转成 ROS Image 消息
                        img = np.asanyarray(color_frame.get_data())
                        img_msg = self.bridge.cv2_to_imgmsg(img, encoding=encoding)
                        img_msg.header.frame_id = cam_info_msg.header.frame_id
                        img_msg.header.stamp = self.get_clock().now().to_msg()

                        # 内参与图像同一时间戳
                        cam_info_msg.header.stamp = img_msg.header.stamp

                        # 发布
                        image_pub.publish(img_msg)
                        info_pub.publish(cam_info_msg)


                    except RuntimeError:
                        self.get_logger().warn(f"[{cam_name}] 断开，重连中...")
                        break

            except Exception as e:
                self.get_logger().error(f"[{cam_name}] 启动失败: {str(e)}")

            finally:
                if pipeline:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRealsenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点关闭中...")
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
