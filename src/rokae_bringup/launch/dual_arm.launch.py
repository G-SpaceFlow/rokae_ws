"""Start the dual-arm state publisher and motion servers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_parameters = str(
        Path(get_package_share_directory("rokae_bringup"))
        / "config"
        / "dual_arm.yaml"
    )

    parameters_file = LaunchConfiguration("params_file")
    start_move_server = LaunchConfiguration("start_move_server")
    start_movel_service = LaunchConfiguration("start_movel_service")
    start_state_publisher = LaunchConfiguration("start_state_publisher")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_movel_service",
                default_value="true",
                description="Start the dual-arm MoveL services",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_parameters,
                description="ROS 2 parameter YAML file",
            ),
            DeclareLaunchArgument(
                "start_move_server",
                default_value="true",
                description="Start the dual-arm MoveAbsJ action server",
            ),
            DeclareLaunchArgument(
                "start_state_publisher",
                default_value="true",
                description="Publish both arms' joints and TCP poses",
            ),
            Node(
                package="rokae_driver",
                executable="ros_movel_service",
                output="screen",
                parameters=[parameters_file],
                condition=IfCondition(start_movel_service),
            ),
            Node(
                package="rokae_driver",
                executable="ros_moveabsj_action_server",
                output="screen",
                parameters=[parameters_file],
                condition=IfCondition(start_move_server),
            ),
            Node(
                package="rokae_driver",
                executable="ros_pos_publisher",
                output="screen",
                parameters=[parameters_file],
                condition=IfCondition(start_state_publisher),
            ),
        ]
    )
