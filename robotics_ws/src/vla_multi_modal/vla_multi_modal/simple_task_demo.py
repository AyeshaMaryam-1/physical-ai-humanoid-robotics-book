#!/usr/bin/env python3

"""
Simple Task Demonstration for VLA System

This script demonstrates a simple task that combines voice, vision, and LLM guidance
to show how the multi-modal system works together.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from vla_msgs.msg import LLMPlan
import time
import threading


class SimpleTaskDemo(Node):
    def __init__(self):
        super().__init__('simple_task_demo')

        # Publishers for the different system components
        self.voice_command_pub = self.create_publisher(
            String,
            'transcribed_text',  # This is where the voice input goes
            10
        )

        self.command_input_pub = self.create_publisher(
            String,
            'command_input',  # This is where LLM planner receives commands
            10
        )

        # Subscribers to monitor system state
        self.plan_sub = self.create_subscription(
            LLMPlan,
            'robot_plan',
            self.plan_callback,
            10
        )

        self.decision_sub = self.create_subscription(
            String,
            'fused_decision',
            self.decision_callback,
            10
        )

        # Task state tracking
        self.current_task = None
        self.task_completed = False
        self.plan_received = False

        # Timer to run the demonstration
        self.demo_timer = self.create_timer(5.0, self.demo_step)
        self.step_counter = 0

        self.get_logger().info("Simple task demonstration node initialized")

    def plan_callback(self, msg):
        """Handle received plans from the LLM planner"""
        self.get_logger().info(f"Received plan: {msg.description}")
        self.get_logger().info(f"Plan has {len(msg.steps)} steps")
        self.plan_received = True

        # Print out the plan steps
        for i, step in enumerate(msg.steps):
            self.get_logger().info(f"Step {i+1}: {step.action_name} at {step.target_pose.position.x:.2f}, {step.target_pose.position.y:.2f}")

    def decision_callback(self, msg):
        """Handle fused decisions from the multi-modal fusion node"""
        self.get_logger().info(f"Multi-modal decision: {msg.data}")

    def demo_step(self):
        """Execute demonstration steps"""
        if self.step_counter == 0:
            self.get_logger().info("=== Starting Simple Task Demonstration ===")
            self.get_logger().info("Task: Fetch a red cup and bring it to the user")
            self.current_task = "fetch_red_cup"

        elif self.step_counter == 1:
            # Simulate voice command: "Please bring me the red cup"
            voice_cmd = String()
            voice_cmd.data = "Please bring me the red cup"
            self.voice_command_pub.publish(voice_cmd)
            self.get_logger().info(f"Published voice command: {voice_cmd.data}")

        elif self.step_counter == 2:
            # Check if we have a plan
            if self.plan_received:
                self.get_logger().info("Plan successfully generated - task demonstration complete!")
                self.task_completed = True
            else:
                self.get_logger().info("Waiting for plan generation...")

        elif self.step_counter == 3:
            self.get_logger().info("=== Demonstration Summary ===")
            self.get_logger().info("1. Voice input: 'Please bring me the red cup'")
            self.get_logger().info("2. Vision system: Detects objects in the environment")
            self.get_logger().info("3. Multi-modal fusion: Combines voice and vision to understand the task")
            self.get_logger().info("4. LLM planner: Generates a plan to fetch the object")
            self.get_logger().info("5. Execution: Robot executes the plan to complete the task")
            self.get_logger().info("==============================")

        elif self.step_counter >= 4:
            # End the demonstration
            self.demo_timer.cancel()
            self.get_logger().info("Demonstration completed")
            return

        self.step_counter += 1

    def run_demo_sequence(self):
        """Run the complete demonstration sequence"""
        self.get_logger().info("Running simple task demonstration sequence...")

        # Wait for system to be ready
        time.sleep(2.0)

        # Execute the sequence
        self.demo_step()  # Step 0
        time.sleep(5.0)
        self.demo_step()  # Step 1
        time.sleep(5.0)
        self.demo_step()  # Step 2
        time.sleep(5.0)
        self.demo_step()  # Step 3


def main(args=None):
    rclpy.init(args=args)

    demo_node = SimpleTaskDemo()

    # Run the demo sequence in a separate thread to allow callbacks to process
    def run_demo():
        time.sleep(1.0)  # Allow system to initialize
        demo_node.run_demo_sequence()

    demo_thread = threading.Thread(target=run_demo)
    demo_thread.start()

    try:
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        demo_node.get_logger().info("Demo interrupted by user")
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()
        demo_thread.join()


if __name__ == '__main__':
    main()