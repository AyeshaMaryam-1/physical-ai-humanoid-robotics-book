from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('humanoid_description')

    # Define the URDF file path
    default_model_path = os.path.join(pkg_share, 'urdf/simple_humanoid.urdf')

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model')])
        }]
    )

    # ros2_control Node
    ros2_controllers_path = os.path.join(
        get_package_share_directory('ai_control_agent'),
        'config',
        'controllers.yaml'
    )

    # Create controllers.yaml configuration file
    controllers_config = """
    controller_manager:
      ros__parameters:
        update_rate: 100  # Hz

        joint_state_broadcaster:
          type: joint_state_broadcaster/JointStateBroadcaster

        joint_trajectory_controller:
          type: joint_trajectory_controller/JointTrajectoryController
    """

    # Write controllers.yaml to the config directory
    config_dir = os.path.join(get_package_share_directory('ai_control_agent'), 'config')
    os.makedirs(config_dir, exist_ok=True)
    controllers_file = os.path.join(config_dir, 'controllers.yaml')
    with open(controllers_file, 'w') as f:
        f.write(controllers_config)

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        output='both',
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Joint Trajectory Controller
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
    )

    # Simple AI Agent node
    ai_agent_node = Node(
        package='ai_control_agent',
        executable='simple_ai_agent',
        name='simple_ai_agent',
        output='screen'
    )

    # RViz2 node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('humanoid_description'), 'rviz', 'display.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Delay rviz start after joint_state_broadcaster spawner finishes
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        ai_agent_node,
        delay_rviz_after_joint_state_broadcaster_spawner
    ])