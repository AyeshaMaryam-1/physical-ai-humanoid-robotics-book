#!/usr/bin/env python3

"""
Whisper Transcription Node for VLA System

This node subscribes to audio data and publishes transcribed text using OpenAI's Whisper model.
"""

import rclpy
from rclpy.node import Node
import whisper
import numpy as np
import torch
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String, Float32
import io
import wave
from collections import deque
import time


class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')

        # Load Whisper model (using smaller model for real-time performance)
        self.get_logger().info("Loading Whisper model...")
        model_size = self.declare_parameter('model_size', 'base').value
        self.model = whisper.load_model(model_size)

        # Audio buffer for accumulating audio chunks
        self.audio_buffer = np.array([], dtype=np.float32)

        # Parameters
        self.buffer_duration = self.declare_parameter('buffer_duration', 2.0).value  # seconds
        self.sample_rate = self.declare_parameter('sample_rate', 16000).value
        self.min_audio_length = self.declare_parameter('min_audio_length', 0.5).value  # seconds
        self.silence_threshold = self.declare_parameter('silence_threshold', 0.01).value  # amplitude threshold
        self.publish_confidence = self.declare_parameter('publish_confidence', True).value

        # Publishers and subscribers
        self.audio_sub = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        self.text_pub = self.create_publisher(
            String,
            'transcribed_text',
            10
        )

        # Optional confidence score publisher
        if self.publish_confidence:
            self.confidence_pub = self.create_publisher(
                Float32,
                'transcription_confidence',
                10
            )

        # Statistics
        self.transcription_count = 0
        self.last_transcription_time = time.time()

        self.get_logger().info(f"Whisper node initialized with {model_size} model")

    def audio_callback(self, msg):
        """Process incoming audio data"""
        # Convert audio data to numpy array (assuming int16 format)
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

        # Add to buffer
        self.audio_buffer = np.concatenate([self.audio_buffer, audio_data])

        # Check if buffer has enough data for transcription
        required_samples = int(self.buffer_duration * self.sample_rate)
        if len(self.audio_buffer) >= required_samples:
            self.transcribe_audio()

    def is_silence(self, audio_segment):
        """Check if audio segment is mostly silence"""
        if len(audio_segment) == 0:
            return True

        # Calculate RMS (Root Mean Square) to determine if it's silence
        rms = np.sqrt(np.mean(audio_segment ** 2))
        return rms < self.silence_threshold

    def transcribe_audio(self):
        """Transcribe buffered audio using Whisper"""
        if len(self.audio_buffer) < self.min_audio_length * self.sample_rate:
            # Not enough audio data, return early
            return

        # Check if the audio is mostly silence
        if self.is_silence(self.audio_buffer):
            # Clear buffer and return early to avoid processing silence
            self.audio_buffer = np.array([], dtype=np.float32)
            return

        # Ensure audio is in the right format
        audio = self.audio_buffer.copy()

        # Clear buffer for next round
        self.audio_buffer = np.array([], dtype=np.float32)

        try:
            # Transcribe using Whisper
            result = self.model.transcribe(audio, fp16=False)  # fp16 might cause issues on some systems
            text = result["text"].strip()

            if text:  # Only publish if there's text
                # Publish transcribed text
                text_msg = String()
                text_msg.data = text
                self.text_pub.publish(text_msg)

                # Calculate and publish confidence if requested
                if self.publish_confidence and 'segments' in result:
                    # Calculate average confidence from segments
                    if result['segments']:
                        avg_confidence = sum([seg.get('avg_logprob', 0.0) for seg in result['segments']]) / len(result['segments'])
                        confidence_msg = Float32()
                        confidence_msg.data = float(avg_confidence)
                        self.confidence_pub.publish(confidence_msg)

                self.get_logger().info(f"Transcribed: {text}")
                self.transcription_count += 1

                # Log performance statistics periodically
                current_time = time.time()
                if current_time - self.last_transcription_time > 30:  # Every 30 seconds
                    self.get_logger().info(f"Transcription rate: {self.transcription_count / (current_time - self.last_transcription_time):.2f} transcriptions/minute")
                    self.transcription_count = 0
                    self.last_transcription_time = current_time

        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WhisperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()