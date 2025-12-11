from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import LogInfo


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )

    llm_provider = DeclareLaunchArgument(
        'llm_provider',
        default_value='mock',
        description='LLM provider (openai, local, mock)'
    )

    # Include Isaac Sim launch file if available
    # isaac_sim_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('isaac_sim_ros_bridge'),
    #             'launch',
    #             'isaac_sim.launch.py'
    #         ])
    #     ]),
    #     condition=IfCondition(LaunchConfiguration('use_isaac_sim'))
    # )

    # Voice input system
    whisper_node = Node(
        package='vla_voice_input',
        executable='whisper_node',
        name='whisper_node',
        parameters=[
            {
                'model_size': 'base',
                'buffer_duration': 2.0,
                'sample_rate': 16000,
                'min_audio_length': 0.5,
                'silence_threshold': 0.01,
                'publish_confidence': True,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            ('audio_input', '/audio'),
            ('transcribed_text', '/vla/voice/text'),
            ('transcription_confidence', '/vla/voice/confidence')
        ],
        output='screen'
    )

    # LLM planning system
    llm_planner_node = Node(
        package='vla_llm_planner',
        executable='llm_planner_node',
        name='llm_planner_node',
        parameters=[
            {
                'llm_provider': LaunchConfiguration('llm_provider'),
                'model_name': 'gpt-3.5-turbo',
                'max_tokens': 1000,
                'temperature': 0.3,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            ('command_input', '/vla/command'),
            ('refined_command', '/vla/refined_command'),
            ('robot_plan', '/vla/plan'),
            ('plan_validation', '/vla/plan_validation')
        ],
        output='screen'
    )

    # Multi-modal fusion system
    fusion_node = Node(
        package='vla_multi_modal',
        executable='fusion_node',
        name='multi_modal_fusion_node',
        parameters=[
            {
                'vision_timeout': 5.0,
                'voice_timeout': 10.0,
                'confidence_threshold': 0.7,
                'max_context_history': 10,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
            ('vision/detections', '/vision/detections'),
            ('transcribed_text', '/vla/voice/text'),
            ('robot_plan', '/vla/plan'),
        ],
        output='screen'
    )

    # System orchestrator
    orchestrator_node = Node(
        package='vla_system_manager',
        executable='vla_orchestrator',
        name='vla_orchestrator_node',
        parameters=[
            {
                'max_execution_time': 300.0,  # 5 minutes
                'enable_human_feedback': True,
                'safety_check_interval': 1.0,
                'step_timeout': 60.0,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            ('transcribed_text', '/vla/voice/text'),
            ('command_input', '/vla/command'),
            ('refined_command', '/vla/refined_command'),
            ('robot_plan', '/vla/plan'),
            ('execution_status', '/execution/status'),
            ('human_feedback', '/human/feedback'),
            ('system_status', '/system/status')
        ],
        output='screen'
    )

    # Vision system (placeholder - would connect to actual vision pipeline)
    vision_node = Node(
        package='vision_msgs',
        executable='vision_processor',  # This would be a placeholder for actual vision processing
        name='vision_processor_node',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('detections', '/vision/detections')
        ],
        output='screen'
    )

    # Navigation system (connect to Nav2)
    navigation_node = Node(
        package='nav2_bringup',
        executable='nav2_launch',  # Placeholder for Nav2 integration
        name='navigation_system',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        output='screen'
    )

    # Manipulation system (placeholder)
    manipulation_node = Node(
        package='manipulation_msgs',
        executable='manipulation_controller',  # Placeholder for manipulation system
        name='manipulation_system',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        output='screen'
    )

    # Log system startup
    system_start_log = LogInfo(
        msg=['VLA System is starting up...']
    )

    # Register event handler for system startup
    startup_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=orchestrator_node,
            on_exit=[LogInfo(msg=['VLA System has shut down.'])]
        )
    )

    return LaunchDescription([
        use_sim_time,
        llm_provider,
        system_start_log,
        whisper_node,
        llm_planner_node,
        fusion_node,
        orchestrator_node,
        # Note: Vision, Navigation, and Manipulation nodes are placeholders
        # They would need to be implemented based on specific robot hardware
        startup_handler
    ])