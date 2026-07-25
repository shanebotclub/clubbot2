from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("clubbot2")
    control_config = os.path.join(pkg_share, "config", "ros2_control.yaml")

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[control_config],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller"],
            output="screen"
        ),

        Node(
            package="clubbot2",
            executable="hardware_interface",
            output="screen"
        )
    ])

