from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory("clubbot2")

    # Load your robot parameters
    robot_params = os.path.join(pkg_share, "config", "RobotParams.yaml")

    # Load ros2_control config
    control_config = os.path.join(pkg_share, "config", "ros2_control.yaml")

    return LaunchDescription([

        # ---------------------------------------------------------
        # micro-ROS agent (serial transport)
        # ---------------------------------------------------------
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            arguments=["serial", "--dev", "/dev/ttyUSB0"],
            output="screen"
        ),

        # ---------------------------------------------------------
        # ros2_control controller manager
        # ---------------------------------------------------------
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[control_config],
            output="screen"
        ),

        # ---------------------------------------------------------
        # diff drive controller spawner
        # ---------------------------------------------------------
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller"],
            output="screen"
        ),

        # ---------------------------------------------------------
        # PID motor controller (closed-loop wheel control)
        # ---------------------------------------------------------
        Node(
            package="clubbot2",
            executable="PID_motor_controller",
            parameters=[robot_params],
            output="screen"
        )
    ])
