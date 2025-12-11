from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch all the nodes needed for the simple task demonstration
    return LaunchDescription([
        # Multi-modal fusion node
        Node(
            package='vla_multi_modal',
            executable='fusion_node',
            name='multi_modal_fusion_node',
            parameters=[
                {'vision_timeout': 5.0},
                {'voice_timeout': 10.0},
                {'confidence_threshold': 0.7},
                {'max_context_history': 10}
            ],
            remappings=[
                ('camera/image_raw', '/camera/image_raw'),
                ('vision/detections', '/vision/detections'),
                ('transcribed_text', '/vla/voice/text'),
                ('robot_plan', '/vla/plan'),
            ],
            output='screen'
        ),

        # LLM planner node
        Node(
            package='vla_llm_planner',
            executable='llm_planner_node',
            name='llm_planner_node',
            parameters=[
                {'llm_provider': 'mock'},  # Use mock for demonstration
                {'model_name': 'gpt-3.5-turbo'},
                {'max_tokens': 1000},
                {'temperature': 0.3}
            ],
            remappings=[
                ('command_input', '/vla/command'),
                ('refined_command', '/vla/refined_command'),
                ('robot_plan', '/vla/plan'),
            ],
            output='screen'
        ),

        # Simple task demonstration node
        Node(
            package='vla_multi_modal',
            executable='simple_task_demo',
            name='simple_task_demo',
            output='screen'
        )
    ])