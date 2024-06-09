from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_launch',
            executable='create_path',
            output='screen'
        ),
        Node(
            package='my_launch',
            executable='controller_monitor',
            output='screen'
        )
    ])