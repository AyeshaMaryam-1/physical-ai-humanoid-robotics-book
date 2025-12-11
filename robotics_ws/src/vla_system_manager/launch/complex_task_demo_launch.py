from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )

    robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='humanoid_robot',
        description='Namespace for the robot'
    )

    # Complex task demonstration node
    complex_task_demo = Node(
        package='vla_system_manager',
        executable='complex_task_demo',
        name='complex_task_demo',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            ('vla/voice/text', 'vla/voice/text'),
            ('human/feedback', 'human/feedback'),
            ('vla/plan', 'vla/plan'),
            ('system/status', 'system/status'),
            ('execution/status', 'execution/status')
        ],
        output='screen'
    )

    # Include the complete VLA system in a group with namespace
    vla_system_group = GroupAction(
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

            # LLM planning node
            Node(
                package='vla_llm_planner',
                executable='llm_planner_node',
                name='llm_planner_node',
                parameters=[
                    {
                        'llm_provider': 'mock',  # Use mock for demonstration
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

    return LaunchDescription([
        use_sim_time,
        robot_namespace,
        vla_system_group,
        complex_task_demo
    ])