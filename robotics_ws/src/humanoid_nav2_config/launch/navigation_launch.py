import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


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
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    # Create lifecycle manager node
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'node_names': ['map_server',
                                   'planner_server',
                                   'controller_server',
                                   'bt_navigator',
                                   'amcl',
                                   'local_costmap',
                                   'global_costmap']}]
    )

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(pkg_share, 'maps', 'isaac_sim_map.yaml')},
                    {'frame_id': 'map'},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'topic_name': '/map'},
                    {'broadcast_2tf': True}],
        output='screen'
    )

    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Behavior tree navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Local costmap server
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Global costmap server
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # AMCL (for localization)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Velocity smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Pose publisher
    pose_publisher = Node(
        package='nav2_pose_smoother',
        executable='smoother_node',
        name='pose_publisher',
        parameters=[LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        params_file,
        lifecycle_manager,
        map_server,
        planner_server,
        controller_server,
        bt_navigator,
        local_costmap,
        global_costmap,
        amcl,
        velocity_smoother,
        pose_publisher
    ])