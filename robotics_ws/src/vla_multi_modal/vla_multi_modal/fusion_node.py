#!/usr/bin/env python3

"""
Multi-Modal Fusion Node for VLA System

This node fuses information from voice, vision, and LLM reasoning to create intelligent robot behaviors.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
from vla_msgs.msg import LLMPlan, PlanStep
import cv2
from cv_bridge import CvBridge
import numpy as np
import json
from collections import deque
import time
import re


class MultiModalFusionNode(Node):
    def __init__(self):
        super().__init__('multi_modal_fusion_node')

        # Initialize components
        self.bridge = CvBridge()

        # Parameters
        self.vision_timeout = self.declare_parameter('vision_timeout', 5.0).value
        self.voice_timeout = self.declare_parameter('voice_timeout', 10.0).value
        self.fusion_strategy = self.declare_parameter('fusion_strategy', 'intermediate').value
        self.confidence_threshold = self.declare_parameter('confidence_threshold', 0.7).value
        self.max_context_history = self.declare_parameter('max_context_history', 10).value

        # Data storage for multi-modal context
        self.latest_image = None
        self.latest_detections = None
        self.latest_voice_command = None
        self.latest_plan = None
        self.context_history = deque(maxlen=self.max_context_history)

        # Time stamps for data freshness
        self.image_timestamp = None
        self.detections_timestamp = None
        self.voice_timestamp = None
        self.plan_timestamp = None

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.detections_sub = self.create_subscription(
            Detection2DArray,
            'vision/detections',
            self.detections_callback,
            10
        )

        self.voice_sub = self.create_subscription(
            String,
            'transcribed_text',
            self.voice_callback,
            10
        )

        self.plan_sub = self.create_subscription(
            LLMPlan,
            'robot_plan',
            self.plan_callback,
            10
        )

        # Publishers for fused decisions
        self.decision_pub = self.create_publisher(
            String,
            'fused_decision',
            10
        )

        # Publisher for task context
        self.task_context_pub = self.create_publisher(
            String,
            'task_context',
            10
        )

        # Publisher for refined commands to LLM
        self.refined_command_pub = self.create_publisher(
            String,
            'refined_command',
            10
        )

        # Timer for fusion processing
        self.fusion_timer = self.create_timer(0.1, self.fusion_callback)

        self.get_logger().info("Multi-modal fusion node initialized")

    def image_callback(self, msg):
        """Handle incoming camera images"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def detections_callback(self, msg):
        """Handle incoming object detections"""
        self.latest_detections = msg
        self.detections_timestamp = msg.header.stamp

    def voice_callback(self, msg):
        """Handle incoming voice commands"""
        self.latest_voice_command = msg.data
        self.voice_timestamp = self.get_clock().now().to_msg()

    def plan_callback(self, msg):
        """Handle incoming LLM plans"""
        self.latest_plan = msg
        self.plan_timestamp = msg.timestamp

    def fusion_callback(self):
        """Main fusion logic - called periodically"""
        # Check if we have recent data from multiple modalities
        current_time = self.get_clock().now().to_msg()

        # Check for recent voice command
        if (self.latest_voice_command and
            self.time_diff(current_time, self.voice_timestamp) < self.voice_timeout):

            # Create task context combining all available information
            task_context = self.create_task_context()

            # Store in history
            self.context_history.append(task_context)

            # Publish task context for decision making
            context_msg = String()
            context_msg.data = json.dumps(task_context)
            self.task_context_pub.publish(context_msg)

            # Make fused decision based on all modalities
            decision = self.make_fused_decision(task_context)
            if decision:
                decision_msg = String()
                decision_msg.data = decision
                self.decision_pub.publish(decision_msg)

                # If decision requires refined planning, send to LLM
                if self.should_refine_plan(decision, task_context):
                    refined_command = self.create_refined_command(decision, task_context)
                    refined_cmd_msg = String()
                    refined_cmd_msg.data = refined_command
                    self.refined_command_pub.publish(refined_cmd_msg)

    def create_task_context(self):
        """Create comprehensive task context from all modalities"""
        context = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'voice_command': self.latest_voice_command,
            'vision_data': self.extract_vision_context(),
            'robot_state': self.get_robot_state(),
            'environment_context': self.extract_environment_context(),
            'previous_context': list(self.context_history)[-3:] if self.context_history else []  # Last 3 contexts
        }
        return context

    def extract_vision_context(self):
        """Extract relevant information from vision data"""
        if not self.latest_detections:
            return {}

        vision_context = {
            'objects': [],
            'spatial_relations': [],
            'scene_description': 'No detections available',
            'confidence_summary': 0.0
        }

        # Extract object information
        total_confidence = 0.0
        for detection in self.latest_detections.detections:
            if detection.results:  # Check if results exist
                hypothesis = detection.results[0].hypothesis
                obj_info = {
                    'label': hypothesis.class_id,
                    'confidence': hypothesis.score,
                    'bbox': {
                        'x': detection.bbox.center.x,
                        'y': detection.bbox.center.y,
                        'width': detection.bbox.size_x,
                        'height': detection.bbox.size_y
                    },
                    'center_point': {
                        'x': detection.bbox.center.x,
                        'y': detection.bbox.center.y
                    }
                }
                vision_context['objects'].append(obj_info)
                total_confidence += hypothesis.score

        # Calculate average confidence
        if vision_context['objects']:
            vision_context['confidence_summary'] = total_confidence / len(vision_context['objects'])

        # Analyze spatial relationships between objects
        if len(vision_context['objects']) > 1:
            vision_context['spatial_relations'] = self.compute_spatial_relations(
                vision_context['objects']
            )

        return vision_context

    def compute_spatial_relations(self, objects):
        """Compute spatial relationships between detected objects"""
        relations = []
        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects):
                if i != j:
                    # Calculate relative positions
                    dx = obj2['center_point']['x'] - obj1['center_point']['x']
                    dy = obj2['center_point']['y'] - obj1['center_point']['y']

                    # Determine spatial relationship
                    distance = np.sqrt(dx*dx + dy*dy)
                    angle = np.arctan2(dy, dx) * 180 / np.pi  # Convert to degrees

                    relation = {
                        'from': obj1['label'],
                        'to': obj2['label'],
                        'distance': distance,
                        'angle_degrees': angle,
                        'relative_position': self.determine_relative_position(dx, dy)
                    }
                    relations.append(relation)
        return relations

    def determine_relative_position(self, dx, dy):
        """Determine relative position based on dx, dy"""
        # Define sectors (8 directions)
        angle = np.arctan2(dy, dx)
        angle_deg = angle * 180 / np.pi

        if -22.5 <= angle_deg < 22.5:
            return "right"
        elif 22.5 <= angle_deg < 67.5:
            return "bottom-right"
        elif 67.5 <= angle_deg < 112.5:
            return "bottom"
        elif 112.5 <= angle_deg < 157.5:
            return "bottom-left"
        elif 157.5 <= angle_deg or angle_deg < -157.5:
            return "left"
        elif -157.5 <= angle_deg < -112.5:
            return "top-left"
        elif -112.5 <= angle_deg < -67.5:
            return "top"
        else:  # -67.5 <= angle_deg < -22.5
            return "top-right"

    def get_robot_state(self):
        """Get current robot state (position, battery, etc.)"""
        # This would typically come from robot state publisher
        # For simulation purposes, return mock data
        return {
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'battery_level': 0.8,
            'current_task': 'idle',
            'capabilities': ['navigation', 'manipulation', 'communication'],
            'manipulation_status': 'available',
            'navigation_status': 'ready'
        }

    def extract_environment_context(self):
        """Extract environment context from available data"""
        # This would integrate with mapping and localization systems
        return {
            'room_type': 'unknown',  # Would be determined from scene analysis
            'lighting_condition': 'normal',  # Would be analyzed from image
            'obstacles': [],  # Would come from navigation system
            'navigable_areas': [],  # Would come from mapping system
            'landmarks': []  # Would come from SLAM system
        }

    def make_fused_decision(self, context):
        """Make decision by fusing information from all modalities"""
        voice_cmd = context.get('voice_command', '').lower()
        vision_data = context.get('vision_data', {})
        robot_state = context.get('robot_state', {})

        # Analyze confidence levels
        vision_confidence = vision_data.get('confidence_summary', 0.0)
        voice_confidence = 1.0  # Assume voice input is reliable

        # If vision confidence is too low, request clarification or exploration
        if vision_confidence < self.confidence_threshold and 'where' not in voice_cmd and 'what' not in voice_cmd:
            return f"request_visual_clarification: {voice_cmd}"

        # Example decision logic based on multi-modal input
        decision = ""

        if 'bring me' in voice_cmd or 'get me' in voice_cmd or 'fetch' in voice_cmd:
            # Object retrieval task
            target_object = self.extract_target_object(voice_cmd)
            if target_object:
                # Check if object is visible
                visible_objects = [obj['label'] for obj in vision_data.get('objects', [])]
                if target_object in visible_objects:
                    # Find the specific object instance
                    target_obj_data = None
                    for obj in vision_data.get('objects', []):
                        if obj['label'] == target_object:
                            target_obj_data = obj
                            break

                    if target_obj_data:
                        decision = f"object_retrieval_confirmed: {target_object}, position: ({target_obj_data['center_point']['x']}, {target_obj_data['center_point']['y']})"
                    else:
                        decision = f"object_found: {target_object}"
                else:
                    decision = f"searching_for: {target_object}"
            else:
                decision = "object_not_specified"

        elif 'go to' in voice_cmd or 'move to' in voice_cmd or 'navigate to' in voice_cmd:
            # Navigation task
            target_location = self.extract_target_location(voice_cmd)
            if target_location:
                decision = f"navigate_to: {target_location}"
            else:
                decision = "location_not_specified"

        elif 'what do you see' in voice_cmd or 'describe scene' in voice_cmd:
            # Scene description task
            objects = [obj['label'] for obj in vision_data.get('objects', [])]
            if objects:
                decision = f"scene_description: I see {', '.join(objects)}"
            else:
                decision = "no_objects_detected"

        elif 'where is' in voice_cmd or 'find' in voice_cmd:
            # Object localization task
            target_object = self.extract_target_object(voice_cmd)
            if target_object:
                visible_objects = [obj for obj in vision_data.get('objects', []) if obj['label'] == target_object]
                if visible_objects:
                    obj_info = visible_objects[0]
                    decision = f"object_location: The {target_object} is at coordinates ({obj_info['center_point']['x']}, {obj_info['center_point']['y']}) with confidence {obj_info['confidence']:.2f}"
                else:
                    decision = f"object_not_found: {target_object}"
            else:
                decision = "object_not_specified"

        else:
            # Default case - pass to LLM planner with context
            decision = f"llm_planning_needed_with_context: {voice_cmd}"

        return decision

    def extract_target_object(self, command):
        """Extract target object from voice command using keyword matching"""
        # Extended list of common objects
        common_objects = [
            'cup', 'bottle', 'water bottle', 'coffee cup', 'mug', 'glass',
            'book', 'phone', 'mobile', 'cellphone', 'keys', 'wallet',
            'pen', 'pencil', 'paper', 'notebook', 'laptop', 'tablet',
            'chair', 'table', 'sofa', 'couch', 'bed', 'desk',
            'apple', 'banana', 'orange', 'fruit', 'snack', 'food',
            'ball', 'toy', 'remote', 'tv remote', 'bowl', 'plate',
            'box', 'bag', 'backpack', 'purse', 'hat', 'shoe'
        ]

        command_lower = command.lower()
        for obj in common_objects:
            if obj in command_lower:
                return obj

        # If no exact match, try partial matching
        for obj in common_objects:
            if any(word in command_lower for word in obj.split()):
                return obj

        return None

    def extract_target_location(self, command):
        """Extract target location from voice command"""
        # Extended list of common locations
        common_locations = [
            'kitchen', 'living room', 'bedroom', 'office', 'bathroom',
            'dining room', 'hallway', 'garage', 'garden', 'patio',
            'entrance', 'dining area', 'work area', 'study', 'library',
            'dormitory', 'classroom', 'laboratory', 'workshop'
        ]

        command_lower = command.lower()
        for loc in common_locations:
            if loc in command_lower:
                return loc

        # If no exact match, try partial matching
        for loc in common_locations:
            if any(word in command_lower for word in loc.split()):
                return loc

        return None

    def should_refine_plan(self, decision, context):
        """Determine if the decision requires refined planning"""
        return ('object_retrieval_confirmed' in decision or
                'navigate_to' in decision or
                'llm_planning_needed_with_context' in decision)

    def create_refined_command(self, decision, context):
        """Create a refined command for the LLM planner with multi-modal context"""
        original_command = context.get('voice_command', '')
        vision_data = context.get('vision_data', {})

        if 'object_retrieval_confirmed' in decision:
            # Extract object and position
            obj_match = re.search(r'object_retrieval_confirmed: ([^,]+)', decision)
            pos_match = re.search(r'position: \(([^)]+)\)', decision)

            if obj_match and pos_match:
                target_obj = obj_match.group(1)
                pos_str = pos_match.group(1)
                pos_parts = pos_str.split(', ')
                if len(pos_parts) >= 2:
                    x, y = float(pos_parts[0]), float(pos_parts[1])
                    return f"Go to position ({x}, {y}) and pick up the {target_obj}, then bring it to me"

        elif 'navigate_to' in decision:
            # Extract target location
            loc_match = re.search(r'navigate_to: (.+)', decision)
            if loc_match:
                target_loc = loc_match.group(1)
                return f"Navigate to the {target_loc} and wait for further instructions"

        # Default: use original command with context
        return f"{original_command}. The current context is: {json.dumps(vision_data, indent=2)[:500]}..."

    def time_diff(self, time1, time2):
        """Calculate time difference in seconds"""
        if not time2:
            return float('inf')
        return abs(time1.sec - time2.sec + (time1.nanosec - time2.nanosec) / 1e9)

def main(args=None):
    rclpy.init(args=args)
    node = MultiModalFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Multi-modal fusion node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()