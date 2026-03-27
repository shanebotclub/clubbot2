from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='button_publisher',
            name='button_publisher',
            output='screen'
        ),
        Node(
            package='your_package',
            executable='led_subscriber',
            name='led_subscriber',
            output='screen'
        ),
        Node(
            package='your_package',
            executable='button_to_led',
            name='button_to_led',
            output='screen'
        )
    ])
