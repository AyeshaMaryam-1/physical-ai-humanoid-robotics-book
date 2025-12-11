from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import LogInfo, TimerAction
import os


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

    robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='humanoid_robot',
        description='Namespace for the robot'
    )

    # Create groups for different system components
    voice_system_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('robot_namespace')),

            # Whisper voice input node
            Node(
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
                    ('audio_input', 'audio'),
                    ('transcribed_text', 'vla/voice/text'),
                    ('transcription_confidence', 'vla/voice/confidence')
                ],
                output='screen'
            ),
        ]
    )

    llm_planning_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('robot_namespace')),

            # LLM planning node
            Node(
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
                    ('command_input', 'vla/command'),
                    ('refined_command', 'vla/refined_command'),
                    ('robot_plan', 'vla/plan'),
                    ('plan_validation', 'vla/plan_validation')
                ],
                output='screen'
            ),
        ]
    )

    multi_modal_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('robot_namespace')),

            # Multi-modal fusion node
            Node(
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
                    ('camera/image_raw', 'camera/image_raw'),
                    ('vision/detections', 'vision/detections'),
                    ('transcribed_text', 'vla/voice/text'),
                    ('robot_plan', 'vla/plan'),
                ],
                output='screen'
            ),
        ]
    )

    system_orchestrator_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('robot_namespace')),

            # System orchestrator node
            Node(
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
                    ('transcribed_text', 'vla/voice/text'),
                    ('command_input', 'vla/command'),
                    ('refined_command', 'vla/refined_command'),
                    ('robot_plan', 'vla/plan'),
                    ('execution_status', 'execution/status'),
                    ('human_feedback', 'human/feedback'),
                    ('system_status', 'system/status')
                ],
                output='screen'
            ),
        ]
    )

    # System startup log
    system_start_log = LogInfo(
        msg=['Starting complete VLA (Vision-Language-Action) System...']
    )

    system_ready_log = TimerAction(
        period=5.0,  # Wait 5 seconds before logging
        actions=[
            LogInfo(msg=['VLA System is ready for autonomous operation.'])
        ]
    )

    return LaunchDescription([
        use_sim_time,
        llm_provider,
        robot_namespace,
        system_start_log,
        voice_system_group,
        llm_planning_group,
        multi_modal_group,
        system_orchestrator_group,
        system_ready_log
    ])