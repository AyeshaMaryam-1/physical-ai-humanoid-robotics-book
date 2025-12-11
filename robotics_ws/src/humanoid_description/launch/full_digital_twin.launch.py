from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('humanoid_description')
    default_model_path = os.path.join(pkg_share, 'urdf/simple_humanoid.urdf')
    default_world_path = os.path.join(pkg_share, 'worlds/empty_world.world')

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Absolute path to world file'
    )

    # Start Gazebo server and client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false',
        }.items()
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(default_model_path).read()
        }]
    )

    # Spawn the robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', LaunchConfiguration('model'),
            '-entity', 'simple_humanoid',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # ros2_control using FakeSystem as hardware interface
    robot_controllers_path = os.path.join(
        get_package_share_directory('humanoid_description'),
        'config',
        'simple_humanoid_controllers.yaml'
    )

    robot_controllers = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers_path],
        output='both',
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Robot specific controllers
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
    )

    # Sensor Processor node
    sensor_processor_node = Node(
        package='ai_control_agent',
        executable='sensor_processor',
        name='sensor_processor',
        output='screen'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # ROS TCP Endpoint (for Unity communication)
    ros_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='ros_tcp_endpoint',
        parameters=[
            {'tcp_port': 10000},
            {'namespace': ''},
            {'buffer_size': 65536},
            {'RosUnityTcpAddress': '0.0.0.0'}  # Listen on all interfaces
        ],
        output='screen'
    )

    # Event handler to start controllers after ros2_control_node
    delay_rviz_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_ros_tcp_endpoint_after_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ros_tcp_endpoint],
        )
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        robot_controllers,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        sensor_processor_node,
        delay_rviz_after_spawner,
        delay_ros_tcp_endpoint_after_spawner
    ])