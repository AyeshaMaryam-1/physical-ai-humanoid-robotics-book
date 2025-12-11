from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_basics',
            executable='talker',
            name='talker',
            output='screen'
        ),
        Node(
            package='ros_basics',
            executable='listener',
            name='listener',
            output='screen'
        )
    ])