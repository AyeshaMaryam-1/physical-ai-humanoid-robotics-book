#!/usr/bin/env python3

"""
LLM Planning Node for VLA System

This node subscribes to text commands and publishes structured robot plans using LLMs.
"""

import rclpy
from rclpy.node import Node
import json
import time
import re
import os
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory

# Import the custom message types (these will be generated later)
try:
    from vla_msgs.msg import LLMPlan, PlanStep
except ImportError:
    # For documentation purposes, define placeholder classes
    class PlanStep:
        def __init__(self):
            self.action_type = ""
            self.action_name = ""
            self.action_params = []
            self.target_pose = Pose()
            self.expected_duration = 0.0
            self.preconditions = []
            self.effects = []
            self.success_probability = 0.0

    class LLMPlan:
        def __init__(self):
            self.plan_id = ""
            self.description = ""
            self.timestamp = Time()
            self.confidence = 0.0
            self.steps = []

# Import OpenAI if available
try:
    import openai
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False
    print("OpenAI library not available. Using mock responses.")


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Initialize LLM client (multiple options supported)
        self.llm_provider = self.declare_parameter('llm_provider', 'mock').value
        self.setup_llm_client()

        # Parameters
        self.model_name = self.declare_parameter('model_name', 'gpt-3.5-turbo').value
        self.max_tokens = self.declare_parameter('max_tokens', 1000).value
        self.temperature = self.declare_parameter('temperature', 0.3).value
        self.plan_cache_enabled = self.declare_parameter('plan_cache_enabled', True).value

        # Try to get robot capabilities file
        try:
            self.robot_capabilities_file = self.declare_parameter(
                'robot_capabilities_file',
                os.path.join(get_package_share_directory('vla_llm_planner'), 'config', 'robot_capabilities.json')
            ).value
        except:
            # Fallback to relative path
            self.robot_capabilities_file = os.path.join('config', 'robot_capabilities.json')

        # Load robot capabilities
        self.robot_capabilities = self.load_robot_capabilities()

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String,
            'command_input',
            self.command_callback,
            10
        )

        # Subscribe to refined commands from multi-modal fusion
        self.refined_command_sub = self.create_subscription(
            String,
            'refined_command',
            self.refined_command_callback,
            10
        )

        self.plan_pub = self.create_publisher(
            LLMPlan,
            'robot_plan',
            10
        )

        # Optional plan validation feedback
        self.validation_sub = self.create_subscription(
            String,
            'plan_validation',
            self.validation_callback,
            10
        )

        # Plan cache for frequently requested plans
        self.plan_cache = {}
        self.cache_hits = 0
        self.total_requests = 0

        self.get_logger().info(f"LLM Planner node initialized with {self.llm_provider} provider")

    def setup_llm_client(self):
        """Setup LLM client based on provider"""
        if self.llm_provider == 'openai':
            if OPENAI_AVAILABLE:
                api_key = self.declare_parameter('openai_api_key', '').value
                if api_key:
                    openai.api_key = api_key
                    self.get_logger().info("OpenAI client configured")
                else:
                    self.get_logger().warn("No OpenAI API key provided - using mock responses")
                    self.llm_provider = 'mock'
            else:
                self.get_logger().warn("OpenAI library not available - using mock responses")
                self.llm_provider = 'mock'
        elif self.llm_provider == 'local':
            try:
                from transformers import pipeline
                self.local_pipeline = pipeline("text-generation", model="microsoft/DialoGPT-medium")
                self.get_logger().info("Local LLM pipeline configured")
            except ImportError:
                self.get_logger().warn("Transformers library not available - using mock responses")
                self.llm_provider = 'mock'
        # Add other providers as needed

    def load_robot_capabilities(self):
        """Load robot capabilities from configuration file"""
        try:
            with open(self.robot_capabilities_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            self.get_logger().warn(f"Robot capabilities file not found: {self.robot_capabilities_file}")
            # Return default capabilities
            return {
                "navigation": {
                    "max_speed": 1.0,
                    "min_turn_radius": 0.5,
                    "supported_terrains": ["indoor", "flat", "carpet", "tile"]
                },
                "manipulation": {
                    "max_payload": 5.0,
                    "reach": 1.2,
                    "gripper_types": ["parallel", "suction"]
                },
                "perception": {
                    "camera_range": 10.0,
                    "object_detection": ["person", "cup", "bottle", "chair", "table"],
                    "language_understanding": True
                }
            }

    def command_callback(self, msg):
        """Process incoming text commands and generate plans"""
        command = msg.data
        self.total_requests += 1
        self.get_logger().info(f"Received command: {command}")

        # Check plan cache first if enabled
        if self.plan_cache_enabled:
            cached_plan = self.plan_cache.get(command.lower().strip())
            if cached_plan:
                self.cache_hits += 1
                hit_rate = self.cache_hits / self.total_requests
                self.get_logger().info(f"Cache hit! Using cached plan. Hit rate: {hit_rate:.2%}")
                self.plan_pub.publish(cached_plan)
                return

        # Generate plan using LLM
        start_time = time.time()
        plan = self.generate_plan(command)
        generation_time = time.time() - start_time

        if plan:
            self.get_logger().info(f"Generated plan in {generation_time:.2f}s with {len(plan.steps)} steps")

            # Cache the plan if caching is enabled
            if self.plan_cache_enabled:
                self.plan_cache[command.lower().strip()] = plan

            self.plan_pub.publish(plan)
            self.get_logger().info("Published plan to execution system")
        else:
            self.get_logger().error("Failed to generate plan for command")

    def refined_command_callback(self, msg):
        """Process refined commands from multi-modal fusion system"""
        refined_command = msg.data
        self.total_requests += 1
        self.get_logger().info(f"Received refined command from multi-modal fusion: {refined_command}")

        # Generate plan using LLM with the refined command
        start_time = time.time()
        plan = self.generate_plan(refined_command)
        generation_time = time.time() - start_time

        if plan:
            self.get_logger().info(f"Generated plan from refined command in {generation_time:.2f}s with {len(plan.steps)} steps")

            # Add metadata to indicate this plan came from multi-modal fusion
            plan.description = f"Multi-modal fusion refined: {plan.description}"

            self.plan_pub.publish(plan)
            self.get_logger().info("Published refined plan to execution system")
        else:
            self.get_logger().error("Failed to generate plan for refined command")

    def generate_plan(self, command):
        """Generate robot plan using LLM"""
        # Create prompt for the LLM with robot capabilities
        prompt = self.create_prompt(command)

        try:
            if self.llm_provider == 'openai' and openai.api_key:
                # Use OpenAI API
                response = openai.ChatCompletion.create(
                    model=self.model_name,
                    messages=[
                        {"role": "system", "content": self.get_system_prompt()},
                        {"role": "user", "content": prompt}
                    ],
                    max_tokens=self.max_tokens,
                    temperature=self.temperature
                )

                llm_response = response.choices[0].message['content']
            elif self.llm_provider == 'local':
                # Use local model (simplified example)
                llm_response = self.local_generate(prompt)
            else:
                # Mock response for demonstration
                llm_response = self.mock_plan_response(command)

            # Parse LLM response into structured plan
            return self.parse_plan_response(llm_response)

        except Exception as e:
            self.get_logger().error(f"LLM query failed: {e}")
            # Try mock response as fallback
            try:
                mock_response = self.mock_plan_response(command)
                return self.parse_plan_response(mock_response)
            except:
                return None

    def create_prompt(self, command):
        """Create structured prompt for LLM with robot capabilities"""
        return f"""
        Given the following command: "{command}"

        Generate a step-by-step plan for a humanoid robot with these capabilities:
        {json.dumps(self.robot_capabilities, indent=2)}

        The robot can perform these high-level actions:
        - Navigation: move to specific locations, avoid obstacles
        - Manipulation: grasp objects, open doors, press buttons
        - Perception: detect objects, recognize people, assess environment
        - Communication: speak, gesture, acknowledge commands

        Respond in JSON format with the following structure:
        {{
            "description": "Brief description of the plan",
            "steps": [
                {{
                    "action_type": "navigation|manipulation|perception|communication",
                    "action_name": "move_to|pick_object|detect_person|speak|etc",
                    "action_params": ["param1", "param2"],
                    "target_pose": {{"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}},
                    "expected_duration": 10.0,
                    "preconditions": ["robot_is_idle", "object_in_reach"],
                    "effects": ["robot_at_location", "object_grasped"],
                    "success_probability": 0.9
                }}
            ]
        }}

        Ensure all actions are feasible given the robot's capabilities.
        Include safety considerations in the plan.
        """

    def get_system_prompt(self):
        """Get system prompt for LLM"""
        return f"""
        You are a helpful assistant that generates robot action plans.
        The robot is a humanoid with the capabilities defined above.
        Always respond in the exact JSON format requested.
        Ensure all actions are feasible for the robot given its physical and sensory limitations.
        Include only actions that the robot is capable of performing based on the provided capabilities.
        Consider safety constraints in all plans.
        """

    def parse_plan_response(self, response):
        """Parse LLM response into structured plan"""
        try:
            # Extract JSON from response if needed
            json_match = re.search(r'\{.*\}', response, re.DOTALL)
            if json_match:
                plan_data = json.loads(json_match.group())
            else:
                plan_data = json.loads(response)

            # Create LLMPlan message
            plan = LLMPlan()
            plan.plan_id = f"plan_{self.get_clock().now().nanoseconds}"
            plan.description = plan_data.get("description", "Generated plan")
            plan.timestamp = self.get_clock().now().to_msg()
            plan.confidence = 0.8  # Default confidence, could be derived from LLM response

            # Create plan steps
            for step_data in plan_data.get("steps", []):
                step = PlanStep()
                step.action_type = step_data.get("action_type", "unknown")
                step.action_name = step_data.get("action_name", "unknown")
                step.action_params = step_data.get("action_params", [])
                step.expected_duration = float(step_data.get("expected_duration", 10.0))
                step.preconditions = step_data.get("preconditions", [])
                step.effects = step_data.get("effects", [])
                step.success_probability = float(step_data.get("success_probability", 0.8))

                # Parse target pose if provided
                target_pose_data = step_data.get("target_pose", {})
                if target_pose_data:
                    step.target_pose.position.x = target_pose_data.get("x", 0.0)
                    step.target_pose.position.y = target_pose_data.get("y", 0.0)
                    step.target_pose.position.z = target_pose_data.get("z", 0.0)
                    step.target_pose.orientation.x = target_pose_data.get("qx", 0.0)
                    step.target_pose.orientation.y = target_pose_data.get("qy", 0.0)
                    step.target_pose.orientation.z = target_pose_data.get("qz", 0.0)
                    step.target_pose.orientation.w = target_pose_data.get("qw", 1.0)

                plan.steps.append(step)

            return plan

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON response: {e}")
            self.get_logger().debug(f"Response was: {response}")
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to parse plan response: {e}")
            return None

    def local_generate(self, prompt):
        """Generate response using local model (placeholder implementation)"""
        # This is a simplified placeholder - in practice you'd use transformers or similar
        return f'{{"description": "Local plan for: {prompt[:50]}...", "steps": [{{"action_type": "navigation", "action_name": "move_to", "action_params": ["location"], "target_pose": {{"x": 1.0, "y": 1.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}}, "expected_duration": 30.0, "preconditions": ["robot_is_idle"], "effects": ["robot_at_location"], "success_probability": 0.85}}]}}'

    def mock_plan_response(self, command):
        """Generate mock plan for demonstration"""
        return f"""
        {{
            "description": "Mock plan for: {command}",
            "steps": [
                {{
                    "action_type": "navigation",
                    "action_name": "move_to",
                    "action_params": ["kitchen"],
                    "target_pose": {{"x": 2.0, "y": 1.5, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}},
                    "expected_duration": 30.0,
                    "preconditions": ["robot_is_idle"],
                    "effects": ["robot_at_kitchen"],
                    "success_probability": 0.9
                }}
            ]
        }}
        """

    def validation_callback(self, msg):
        """Handle plan validation feedback"""
        feedback = msg.data
        self.get_logger().info(f"Received plan validation feedback: {feedback}")
        # Could be used to improve future planning or adjust confidence

def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("LLM Planner node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()