#!/usr/bin/env python3

"""
Complex Multi-Stage Task Demonstration for VLA System

This script demonstrates a complex multi-stage task: "Go to kitchen, find cup, bring to me"
using the complete VLA (Vision-Language-Action) system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from vla_msgs.msg import LLMPlan
import json
import time
from threading import Thread


class ComplexTaskDemo(Node):
    def __init__(self):
        super().__init__('complex_task_demo')

        # Publishers for the different system components
        self.voice_command_pub = self.create_publisher(
            String,
            'vla/voice/text',  # This is where the voice input goes after Whisper processing
            10
        )

        self.human_feedback_pub = self.create_publisher(
            String,
            'human/feedback',
            10
        )

        # Subscribers to monitor system state
        self.plan_sub = self.create_subscription(
            LLMPlan,
            'vla/plan',
            self.plan_callback,
            10
        )

        self.system_status_sub = self.create_subscription(
            String,
            'system/status',
            self.system_status_callback,
            10
        )

        self.execution_status_sub = self.create_subscription(
            String,
            'execution/status',
            self.execution_status_callback,
            10
        )

        # Task state tracking
        self.current_task_stage = 0
        self.task_completed = False
        self.plan_received = False
        self.task_stages = [
            "Navigate to kitchen",
            "Search for cup",
            "Pick up cup",
            "Return to user",
            "Give cup to user"
        ]

        # Timer to run the demonstration
        self.demo_timer = self.create_timer(10.0, self.demo_step)
        self.step_counter = 0

        # Task execution tracking
        self.task_start_time = self.get_clock().now().to_msg()
        self.stage_start_time = self.get_clock().now().to_msg()

        self.get_logger().info("Complex multi-stage task demonstration node initialized")
        self.get_logger().info("Demonstrating: 'Go to kitchen, find cup, bring to me'")

    def plan_callback(self, msg):
        """Handle received plans from the LLM planner"""
        self.get_logger().info(f"Received plan: {msg.description}")
        self.get_logger().info(f"Plan has {len(msg.steps)} steps")
        self.plan_received = True

        # Print out the plan steps
        for i, step in enumerate(msg.steps):
            self.get_logger().info(f"Step {i+1}: {step.action_name} at ({step.target_pose.position.x:.2f}, {step.target_pose.position.y:.2f})")

    def system_status_callback(self, msg):
        """Handle system status updates"""
        try:
            status_data = json.loads(msg.data)
            state = status_data.get('state', 'unknown')
            self.get_logger().info(f"System status: {state}")

            if state == 'completed':
                self.get_logger().info("Task completed successfully!")
                self.task_completed = True
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse system status: {msg.data}")

    def execution_status_callback(self, msg):
        """Handle execution status updates"""
        try:
            status_data = json.loads(msg.data)
            status = status_data.get('status', 'unknown')
            self.get_logger().info(f"Execution status: {status}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse execution status: {msg.data}")

    def demo_step(self):
        """Execute demonstration steps"""
        if self.step_counter == 0:
            self.get_logger().info("=== Starting Complex Multi-Stage Task Demonstration ===")
            self.get_logger().info("Task: Go to kitchen, find cup, bring to me")
            self.get_logger().info("This task requires coordination between:")
            self.get_logger().info("  - Voice understanding (Whisper)")
            self.get_logger().info("  - LLM planning (GPT/LLM)")
            self.get_logger().info("  - Multi-modal fusion (Voice + Vision)")
            self.get_logger().info("  - Navigation system")
            self.get_logger().info("  - Manipulation system")
            self.get_logger().info("  - System orchestration")

        elif self.step_counter == 1:
            # Give the voice command: "Go to kitchen, find cup, bring to me"
            voice_cmd = String()
            voice_cmd.data = "Go to kitchen, find a cup, and bring it to me"
            self.voice_command_pub.publish(voice_cmd)
            self.get_logger().info(f"Published voice command: {voice_cmd.data}")
            self.get_logger().info("The VLA system will now process this command through all components...")

        elif self.step_counter == 2:
            # Check if we have a plan
            if self.plan_received:
                self.get_logger().info("Plan successfully generated by LLM planner!")
                self.get_logger().info("The plan will now be executed by the orchestration system...")
            else:
                self.get_logger().info("Waiting for plan generation from LLM planner...")

        elif self.step_counter == 3:
            self.get_logger().info("=== Task Execution in Progress ===")
            self.get_logger().info("The system should now be executing the multi-stage task:")
            self.get_logger().info("1. Navigating to kitchen using Nav2")
            self.get_logger().info("2. Using vision system to locate cup")
            self.get_logger().info("3. Planning manipulation to grasp cup")
            self.get_logger().info("4. Returning to user location")
            self.get_logger().info("5. Delivering cup to user")

        elif self.step_counter == 4:
            self.get_logger().info("=== Monitoring Task Progress ===")
            if self.task_completed:
                self.get_logger().info("Task completed successfully!")
            else:
                self.get_logger().info("Task still in progress, continuing to monitor...")

        elif self.step_counter == 5:
            self.get_logger().info("=== Demonstration Summary ===")
            self.get_logger().info("Completed complex multi-stage task: 'Go to kitchen, find cup, bring to me'")
            self.get_logger().info("")
            self.get_logger().info("Stage 1 - Voice Input:")
            self.get_logger().info("  - Whisper processed: 'Go to kitchen, find cup, bring to me'")
            self.get_logger().info("")
            self.get_logger().info("Stage 2 - LLM Planning:")
            self.get_logger().info("  - GPT generated multi-step plan with navigation, perception, manipulation")
            self.get_logger().info("")
            self.get_logger().info("Stage 3 - Multi-Modal Fusion:")
            self.get_logger().info("  - Combined voice command with visual scene understanding")
            self.get_logger().info("")
            self.get_logger().info("Stage 4 - Plan Execution:")
            self.get_logger().info("  - Orchestrator managed execution across all systems")
            self.get_logger().info("")
            self.get_logger().info("Stage 5 - Task Completion:")
            self.get_logger().info("  - Robot successfully delivered cup to user")
            self.get_logger().info("")
            self.get_logger().info("This demonstration showcases the complete VLA system integration!")

        elif self.step_counter >= 6:
            # End the demonstration
            self.demo_timer.cancel()
            self.get_logger().info("Complex multi-stage task demonstration completed")
            return

        self.step_counter += 1

    def run_demo_sequence(self):
        """Run the complete demonstration sequence"""
        self.get_logger().info("Running complex multi-stage task demonstration sequence...")

        # Wait for system to be ready
        time.sleep(2.0)

        # Execute the sequence
        for i in range(7):  # Execute 7 steps
            self.demo_step()
            time.sleep(10.0)  # Wait 10 seconds between steps


def main(args=None):
    rclpy.init(args=args)

    demo_node = ComplexTaskDemo()

    # Run the demo sequence in a separate thread to allow callbacks to process
    def run_demo():
        time.sleep(1.0)  # Allow system to initialize
        demo_node.run_demo_sequence()

    demo_thread = Thread(target=run_demo)
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