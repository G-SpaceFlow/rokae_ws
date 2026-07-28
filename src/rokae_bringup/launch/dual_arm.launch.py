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
    start_hand_service = LaunchConfiguration("start_hand_service")
    start_initializer_service = LaunchConfiguration(
        "start_initializer_service"
    )
    start_move_server = LaunchConfiguration("start_move_server")
    start_movel_service = LaunchConfiguration("start_movel_service")
    start_state_publisher = LaunchConfiguration("start_state_publisher")
    start_chassis_navigation = LaunchConfiguration(
        "start_chassis_navigation"
    )
    start_vision_target_server = LaunchConfiguration(
        "start_vision_target_server"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_hand_service",
                default_value="true",
                description="Start the dual-hand end-CAN services",
            ),
            DeclareLaunchArgument(
                "start_initializer_service",
                default_value="true",
                description="Start the explicit dual-arm power-on service",
            ),
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
            DeclareLaunchArgument(
                "start_chassis_navigation",
                default_value="false",
                description=(
                    "Start the scheduler-to-Seer chassis navigation bridge"
                ),
            ),
            DeclareLaunchArgument(
                "start_vision_target_server",
                default_value="true",
                description=(
                    "Start the vision trigger and target-point cache"
                ),
            ),
            Node(
                package="rokae_driver",
                executable="ros_robot_initializer_service",
                output="screen",
                parameters=[parameters_file],
                condition=IfCondition(start_initializer_service),
            ),
            Node(
                package="rokae_driver",
                executable="ros_hand_service",
                output="screen",
                parameters=[parameters_file],
                condition=IfCondition(start_hand_service),
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
            Node(
                package="rokae_motion",
                executable="chassis_navigation",
                output="screen",
                condition=IfCondition(start_chassis_navigation),
            ),
            Node(
                package="rokae_motion",
                executable="vision_target_server",
                output="screen",
                parameters=[parameters_file],
                condition=IfCondition(start_vision_target_server),
            ),
        ]
    )
