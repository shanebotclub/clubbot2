import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Launch argument defaulting to 'aver'
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='clubbot',
        description='Robot name identifier for YAML parameter loading'
    )

    # Build filename string: "RobotParams_" + robot + ".yaml"
    param_file_name = PythonExpression(["'RobotParams_' + '", LaunchConfiguration('robot'), "' + '.yaml'"])

    robot_params = PathJoinSubstitution([
        FindPackageShare('clubbot2'),
        'config',
        param_file_name
    ])

    return LaunchDescription([
        robot_arg,

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
            parameters=[robot_params],
            output="screen"
        )
    ])