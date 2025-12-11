from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )

    # Object detection node
    object_detector_node = Node(
        package='isaac_perception_examples',
        executable='object_detector_node',
        name='object_detector_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Point cloud processor node
    pointcloud_processor_node = Node(
        package='isaac_perception_examples',
        executable='pointcloud_processor',
        name='pointcloud_processor',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        object_detector_node,
        pointcloud_processor_node
    ])