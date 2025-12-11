from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    llm_provider = DeclareLaunchArgument(
        'llm_provider',
        default_value='mock',
        description='LLM provider (openai, local, mock)'
    )

    model_name = DeclareLaunchArgument(
        'model_name',
        default_value='gpt-3.5-turbo',
        description='LLM model name to use'
    )

    openai_api_key = DeclareLaunchArgument(
        'openai_api_key',
        default_value='',
        description='OpenAI API key (if using OpenAI provider)'
    )

    max_tokens = DeclareLaunchArgument(
        'max_tokens',
        default_value='1000',
        description='Maximum tokens for LLM response'
    )

    temperature = DeclareLaunchArgument(
        'temperature',
        default_value='0.3',
        description='Temperature for LLM response creativity'
    )

    # Get the path to the config file
    config_file_path = os.path.join(
        get_package_share_directory('vla_llm_planner'),
        'config',
        'robot_capabilities.json'
    )

    # Create the LLM planner node
    llm_planner_node = Node(
        package='vla_llm_planner',
        executable='llm_planner_node',
        name='llm_planner_node',
        parameters=[
            {
                'llm_provider': LaunchConfiguration('llm_provider'),
                'model_name': LaunchConfiguration('model_name'),
                'max_tokens': LaunchConfiguration('max_tokens'),
                'temperature': LaunchConfiguration('temperature'),
                'robot_capabilities_file': config_file_path,
                'plan_cache_enabled': True
            }
        ],
        remappings=[
            ('command_input', '/vla/command'),
            ('robot_plan', '/vla/plan'),
            ('plan_validation', '/vla/plan_validation')
        ],
        output='screen'
    )

    return LaunchDescription([
        llm_provider,
        model_name,
        openai_api_key,
        max_tokens,
        temperature,
        llm_planner_node
    ])