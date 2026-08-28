import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. Declare the 'robot' launch argument (defaults to 'aver')
    robot_name_arg = DeclareLaunchArgument(
        'robot',
        default_value='aver',
        description='Name of robot config file to load (e.g., aver -> RobotParams_aver.yaml)'
    )

    # 2. Build parameter path dynamically: config/RobotParams_<robot>.yaml
    # If robot:=aver, loads config/RobotParams_aver.yaml
    yaml_filename = [
        'RobotParams_',
        LaunchConfiguration('robot'),
        '.yaml'
    ]
    
    robot_params = PathJoinSubstitution([
        FindPackageShare('clubbot2'),
        'config',
        yaml_filename
    ])

    return LaunchDescription([
        robot_name_arg,

        # micro-ROS agent
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            arguments=["serial", "--dev", "/dev/ttyUSB0"],
            output="screen"
        ),

        # PID motor controller node
        Node(
            package="clubbot2",
            executable="PID_motor_controller",
            name="pid_motor_controller",
            parameters=[robot_params],
            output="screen"
        )
    ])