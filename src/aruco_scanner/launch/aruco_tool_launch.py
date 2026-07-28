from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_camera",
            default_value="false",
            description="是否启动本工作空间的 RealSense 驱动；已有相机发布节点时应为 false",
        ),

        # 可选：仅在相机没有被其他程序占用时启动
        Node(
            package="realsense_multi_camera",
            executable="multi_camera_node",
            name="multi_realsense_node",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("start_camera")),
        ),

        # 默认复用系统中 rs_pub.py 发布的相机话题
        Node(
            package="aruco_scanner",
            executable="arucode_position_node",
            name="aruco_tool_pose_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "publish_rate": 10,
                    "enable_gui": True,
                    "image_topic": "/camera/color/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                }
            ],
        ),

        Node(
            package="aruco_scanner",
            executable="big_box_detection_node",
            name="box_dual_grab_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "enable_gui": True,
                    "image_topic": "/camera/color/image_raw",
                    "depth_topic": "/camera/depth/image_rect_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "enable_topic": "/box/enable",
                    "left_point_topic": "/box/grab_point/left",
                    "right_point_topic": "/box/grab_point/right",
                    "angle_topic": "/box/angle_deg",
                }
            ],
        ),
    ])
