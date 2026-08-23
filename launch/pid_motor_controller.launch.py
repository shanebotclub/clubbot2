from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory("clubbot2")

    # Load your robot parameters
    robot_params = os.path.join(pkg_share, "config", "RobotParams.yaml")

    return LaunchDescription([

        # ---------------------------------------------------------
        # micro-ROS agent (serial transport to ESP32)
        # ---------------------------------------------------------
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            arguments=["serial", "--dev", "/dev/ttyUSB0"],
            output="screen"
        ),

        # ---------------------------------------------------------
        # PID motor controller (closed-loop wheel control via gpiozero)
        # ---------------------------------------------------------
        Node(
            package="clubbot2",
            executable="PID_motor_controller",
            parameters=[robot_params],
            output="screen"
        )
    ])