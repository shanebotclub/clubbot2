import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. Declare Launch Arguments
    # Allows passing 'robot:=aver' or specifying a custom YAML path dynamically
    robot_name_arg = DeclareLaunchArgument(
        'robot',
        default_value='aver',
        description='Robot parameter file prefix (e.g. "aver" -> RobotParams_aver.yaml or default RobotParams.yaml)'
    )

    # Path to the parameter file dynamically built from the launch argument
    # Defaults to loading 'config/RobotParams.yaml' if robot parameter matches default
    robot_params = PathJoinSubstitution([
        FindPackageShare('clubbot2'),
        'config',
        'RobotParams.yaml'
    ])

    return LaunchDescription([
        robot_name_arg,

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