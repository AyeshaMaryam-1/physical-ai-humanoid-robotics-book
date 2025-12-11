#!/usr/bin/env python3

"""
VLA System Orchestrator Node

This node manages the overall VLA system by orchestrating all components
for autonomous humanoid robot operation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from vla_msgs.msg import LLMPlan
from geometry_msgs.msg import Pose
import json
import time
from threading import Lock
import traceback


class VLAOrchestratorNode(Node):
    def __init__(self):
        super().__init__('vla_orchestrator_node')

        # System state management
        self.current_state = 'idle'  # idle, listening, planning, executing, error
        self.current_plan = None
        self.system_lock = Lock()

        # Parameters
        self.max_execution_time = self.declare_parameter('max_execution_time', 300.0).value  # 5 minutes
        self.enable_human_feedback = self.declare_parameter('enable_human_feedback', True).value
        self.safety_check_interval = self.declare_parameter('safety_check_interval', 1.0).value
        self.step_timeout = self.declare_parameter('step_timeout', 60.0).value  # 1 minute per step

        # Publishers and subscribers
        self.voice_command_pub = self.create_publisher(
            String,
            'transcribed_text',
            10
        )

        self.command_input_pub = self.create_publisher(
            String,
            'command_input',
            10
        )

        self.refined_command_pub = self.create_publisher(
            String,
            'refined_command',
            10
        )

        self.plan_sub = self.create_subscription(
            LLMPlan,
            'robot_plan',
            self.plan_callback,
            10
        )

        self.execution_status_sub = self.create_subscription(
            String,
            'execution_status',
            self.execution_status_callback,
            10
        )

        self.human_feedback_sub = self.create_subscription(
            String,
            'human_feedback',
            self.human_feedback_callback,
            10
        )

        # System control publishers
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            'emergency_stop',
            10
        )

        # Status publisher
        self.system_status_pub = self.create_publisher(
            String,
            'system_status',
            10
        )

        # Timer for system monitoring
        self.system_monitor_timer = self.create_timer(0.1, self.system_monitor_callback)
        self.safety_timer = self.create_timer(self.safety_check_interval, self.safety_check_callback)

        # Execution tracking
        self.plan_start_time = None
        self.current_step_index = 0
        self.step_start_time = None

        # Task history
        self.task_history = []

        self.get_logger().info("VLA Orchestrator node initialized")

    def plan_callback(self, msg):
        """Handle received plans from the LLM planner"""
        with self.system_lock:
            self.current_plan = msg
            self.current_state = 'executing'
            self.plan_start_time = self.get_clock().now().to_msg()
            self.current_step_index = 0
            self.step_start_time = self.get_clock().now().to_msg()

            self.get_logger().info(f"Received new plan: {msg.description}")
            self.get_logger().info(f"Plan has {len(msg.steps)} steps")

            # Publish system status
            status_msg = String()
            status_msg.data = json.dumps({
                'state': self.current_state,
                'plan_description': msg.description,
                'total_steps': len(msg.steps),
                'timestamp': self.get_clock().now().to_msg().sec
            })
            self.system_status_pub.publish(status_msg)

            # Execute first step
            self.execute_next_step()

    def execution_status_callback(self, msg):
        """Handle execution status updates"""
        try:
            status_data = json.loads(msg.data) if msg.data.startswith('{') else {'status': msg.data}
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse execution status: {msg.data}")
            return

        with self.system_lock:
            if self.current_plan and self.current_step_index < len(self.current_plan.steps):
                current_step = self.current_plan.steps[self.current_step_index]

                if status_data.get('status') == 'completed':
                    self.get_logger().info(f"Completed step: {current_step.action_name}")

                    # Record step completion in history
                    self.task_history.append({
                        'step': self.current_step_index,
                        'action': current_step.action_name,
                        'status': 'completed',
                        'timestamp': self.get_clock().now().to_msg().sec
                    })

                    self.current_step_index += 1
                    self.step_start_time = self.get_clock().now().to_msg()

                    if self.current_step_index >= len(self.current_plan.steps):
                        # All steps completed
                        self.get_logger().info("Plan execution completed successfully!")
                        self.task_history.append({
                            'task': 'complete',
                            'status': 'success',
                            'timestamp': self.get_clock().now().to_msg().sec
                        })
                        self.current_state = 'idle'
                        self.current_plan = None

                        # Publish completion status
                        status_msg = String()
                        status_msg.data = json.dumps({
                            'state': 'completed',
                            'result': 'success',
                            'timestamp': self.get_clock().now().to_msg().sec
                        })
                        self.system_status_pub.publish(status_msg)
                    else:
                        # Move to next step
                        self.execute_next_step()
                elif status_data.get('status') == 'failed':
                    self.get_logger().error(f"Step failed: {current_step.action_name}")
                    self.handle_plan_failure(status_data.get('error', 'Unknown error'))
                elif status_data.get('status') == 'in_progress':
                    # Update status but continue execution
                    pass

    def human_feedback_callback(self, msg):
        """Handle human feedback and corrections"""
        if not self.enable_human_feedback:
            return

        feedback = msg.data.lower()
        self.get_logger().info(f"Received human feedback: {feedback}")

        with self.system_lock:
            if 'stop' in feedback or 'cancel' in feedback or 'abort' in feedback:
                self.emergency_stop()
            elif 'repeat' in feedback or 'again' in feedback:
                self.repeat_current_step()
            elif 'different' in feedback or 'change' in feedback or 'modify' in feedback:
                self.request_plan_revision()
            elif 'status' in feedback or 'progress' in feedback:
                self.report_status()

    def execute_next_step(self):
        """Execute the next step in the current plan"""
        if not self.current_plan or self.current_step_index >= len(self.current_plan.steps):
            return

        step = self.current_plan.steps[self.current_step_index]
        self.get_logger().info(f"Executing step {self.current_step_index + 1}: {step.action_name}")

        # Publish command to execute the step
        step_cmd = String()
        step_cmd.data = json.dumps({
            'action_type': step.action_type,
            'action_name': step.action_name,
            'action_params': step.action_params,
            'target_pose': {
                'x': step.target_pose.position.x,
                'y': step.target_pose.position.y,
                'z': step.target_pose.position.z,
                'qx': step.target_pose.orientation.x,
                'qy': step.target_pose.orientation.y,
                'qz': step.target_pose.orientation.z,
                'qw': step.target_pose.orientation.w
            },
            'expected_duration': step.expected_duration,
            'step_index': self.current_step_index
        })

        # Publish to appropriate execution system based on action type
        if step.action_type == 'navigation':
            self.publish_to_navigation_system(step_cmd)
        elif step.action_type == 'manipulation':
            self.publish_to_manipulation_system(step_cmd)
        elif step.action_type == 'perception':
            self.publish_to_perception_system(step_cmd)
        elif step.action_type == 'communication':
            self.publish_to_communication_system(step_cmd)
        else:
            self.publish_to_generic_system(step_cmd)

    def publish_to_navigation_system(self, cmd):
        """Publish navigation commands"""
        # Publish to navigation system
        nav_pub = self.create_publisher(String, 'navigation_command', 10)
        nav_pub.publish(cmd)

    def publish_to_manipulation_system(self, cmd):
        """Publish manipulation commands"""
        # Publish to manipulation system
        manip_pub = self.create_publisher(String, 'manipulation_command', 10)
        manip_pub.publish(cmd)

    def publish_to_perception_system(self, cmd):
        """Publish perception commands"""
        # Publish to perception system
        percep_pub = self.create_publisher(String, 'perception_command', 10)
        percep_pub.publish(cmd)

    def publish_to_communication_system(self, cmd):
        """Publish communication commands"""
        # Publish to communication system
        comm_pub = self.create_publisher(String, 'communication_command', 10)
        comm_pub.publish(cmd)

    def publish_to_generic_system(self, cmd):
        """Publish to generic action system"""
        # Publish to generic action system
        action_pub = self.create_publisher(String, 'action_command', 10)
        action_pub.publish(cmd)

    def handle_plan_failure(self, error_msg="Unknown error"):
        """Handle plan execution failure"""
        self.get_logger().error(f"Plan execution failed: {error_msg}")

        # Record failure in history
        if self.current_plan and self.current_step_index < len(self.current_plan.steps):
            current_step = self.current_plan.steps[self.current_step_index]
            self.task_history.append({
                'step': self.current_step_index,
                'action': current_step.action_name,
                'status': 'failed',
                'error': error_msg,
                'timestamp': self.get_clock().now().to_msg().sec
            })

        # Request plan revision from LLM with failure context
        if self.current_plan:
            failure_context = f"Plan failed at step {self.current_step_index} with error: {error_msg}. Plan: {self.current_plan.description}"
            failure_cmd = String()
            failure_cmd.data = f"Revise plan based on failure: {failure_context}"
            self.command_input_pub.publish(failure_cmd)

    def request_plan_revision(self):
        """Request plan revision based on human feedback"""
        self.get_logger().info("Human requested plan revision")

        if self.current_plan:
            revision_cmd = String()
            revision_cmd.data = f"Revise current plan based on human feedback: {self.current_plan.description}"
            self.command_input_pub.publish(revision_cmd)

    def repeat_current_step(self):
        """Repeat the current step"""
        if self.current_plan and self.current_step_index > 0:
            # Go back to previous step to repeat
            self.current_step_index -= 1
            self.execute_next_step()

    def report_status(self):
        """Report current system status"""
        status = {
            'state': self.current_state,
            'current_plan': self.current_plan.description if self.current_plan else 'None',
            'current_step': self.current_step_index,
            'total_steps': len(self.current_plan.steps) if self.current_plan else 0,
            'timestamp': self.get_clock().now().to_msg().sec
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.system_status_pub.publish(status_msg)

    def emergency_stop(self):
        """Emergency stop all systems"""
        self.get_logger().warn("Emergency stop activated!")
        self.current_state = 'error'

        # Send emergency stop to all systems
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Record emergency stop in history
        self.task_history.append({
            'event': 'emergency_stop',
            'timestamp': self.get_clock().now().to_msg().sec
        })

        # Clear current plan
        self.current_plan = None

        # Publish error status
        status_msg = String()
        status_msg.data = json.dumps({
            'state': 'error',
            'result': 'emergency_stop',
            'timestamp': self.get_clock().now().to_msg().sec
        })
        self.system_status_pub.publish(status_msg)

    def system_monitor_callback(self):
        """Monitor system state and progress"""
        with self.system_lock:
            if self.current_state == 'executing' and self.plan_start_time:
                current_time = self.get_clock().now().to_msg()
                elapsed_time = (current_time.sec - self.plan_start_time.sec) + \
                              (current_time.nanosec - self.plan_start_time.nanosec) / 1e9

                if elapsed_time > self.max_execution_time:
                    self.get_logger().warn("Plan execution timeout!")
                    self.emergency_stop()

            # Check for step timeout
            if self.current_state == 'executing' and self.step_start_time and self.current_plan:
                current_time = self.get_clock().now().to_msg()
                step_elapsed = (current_time.sec - self.step_start_time.sec) + \
                              (current_time.nanosec - self.step_start_time.nanosec) / 1e9

                if step_elapsed > self.step_timeout:
                    self.get_logger().warn(f"Step {self.current_step_index} timeout!")
                    self.handle_plan_failure(f"Step timeout after {self.step_timeout} seconds")

    def safety_check_callback(self):
        """Perform safety checks"""
        # This would integrate with safety systems to check:
        # - Collision avoidance
        # - Robot health
        # - Environment safety
        # - Human safety
        # For now, just log that safety check is running
        pass

    def process_voice_command(self, command):
        """Process a voice command through the VLA system"""
        with self.system_lock:
            if self.current_state in ['planning', 'executing']:
                self.get_logger().warn("System busy, cannot accept new command")
                return False

            self.current_state = 'listening'

            # Publish to voice input system
            voice_msg = String()
            voice_msg.data = command
            self.voice_command_pub.publish(voice_msg)

            return True

def main(args=None):
    rclpy.init(args=args)
    node = VLAOrchestratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("VLA Orchestrator node shutting down...")
        node.get_logger().info(f"Task history: {len(node.task_history)} events recorded")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()