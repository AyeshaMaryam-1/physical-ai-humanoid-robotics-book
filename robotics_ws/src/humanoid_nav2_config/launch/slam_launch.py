import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('humanoid_nav2_config')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # SLAM Toolbox node
    slam_toolbox = Node(
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        params_file,
        slam_toolbox
    ])