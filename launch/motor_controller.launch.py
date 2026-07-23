from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('clubbot2')
    params_file = os.path.join(pkg_share, 'config', 'RobotParams.yaml')

    return LaunchDescription([
        Node(
            package='clubbot2',
            executable='motor_controller',
            parameters=[params_file],
            output='screen'
        )
    ])
