from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    model_size = DeclareLaunchArgument(
        'model_size',
        default_value='base',
        description='Whisper model size (tiny, base, small, medium, large)'
    )

    buffer_duration = DeclareLaunchArgument(
        'buffer_duration',
        default_value='2.0',
        description='Duration of audio buffer in seconds'
    )

    sample_rate = DeclareLaunchArgument(
        'sample_rate',
        default_value='16000',
        description='Audio sample rate in Hz'
    )

    # Create the Whisper node
    whisper_node = Node(
        package='vla_voice_input',
        executable='whisper_node',
        name='whisper_node',
        parameters=[
            {
                'model_size': LaunchConfiguration('model_size'),
                'buffer_duration': LaunchConfiguration('buffer_duration'),
                'sample_rate': LaunchConfiguration('sample_rate'),
                'min_audio_length': 0.5,
                'silence_threshold': 0.01,
                'publish_confidence': True
            }
        ],
        remappings=[
            ('audio_input', '/audio'),
            ('transcribed_text', '/vla/voice/text'),
            ('transcription_confidence', '/vla/voice/confidence')
        ],
        output='screen'
    )

    return LaunchDescription([
        model_size,
        buffer_duration,
        sample_rate,
        whisper_node
    ])