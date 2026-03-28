from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clubbot2',
            executable='button_publisher',
            name='button_publisher',
            output='screen'
        ),
        Node(
            package='clubbot2',
            executable='led_subscriber',
            name='led_subscriber',
            output='screen'
        ),
        Node(
            package='clubbot2',
            executable='button_to_led',
            name='button_to_led',
            output='screen'
        )
    ])
