import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import random
import math


class SimpleAIAgentNode(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

        # Publisher for joint trajectory commands
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Subscriber for obstacle alerts
        self.obstacle_subscriber = self.create_subscription(
            String,
            'obstacle_alert',
            self.obstacle_callback,
            10
        )

        # Timer for AI decision making
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.ai_decision_callback)

        # Robot state tracking
        self.current_joint_positions = {}
        self.obstacle_detected = False

        # Define joint names for the simple humanoid (from URDF)
        self.joint_names = [
            'left_shoulder', 'left_elbow', 'right_shoulder', 'right_elbow',
            'left_hip', 'left_knee', 'right_hip', 'right_knee'
        ]

        self.get_logger().info('Simple AI Agent Node initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions from joint state messages"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joint_positions[name] = msg.position[i]

    def obstacle_callback(self, msg):
        """Handle obstacle detection messages"""
        self.obstacle_detected = True
        self.get_logger().warn(f'Obstacle detected: {msg.data}')

        # Implement obstacle avoidance behavior
        self.execute_avoidance_behavior()

    def ai_decision_callback(self):
        """Main AI decision-making loop"""
        if self.obstacle_detected:
            self.get_logger().info('Avoiding obstacle')
            return

        # Simple AI behavior: move joints to random positions
        self.execute_random_movement()

    def execute_random_movement(self):
        """Execute random joint movements"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # Generate random positions for each joint within limits
        for joint_name in self.joint_names:
            # Generate random position between -1.0 and 1.0 radians
            random_position = random.uniform(-1.0, 1.0)
            point.positions.append(random_position)

            # Set velocity and acceleration to 0 for simplicity
            point.velocities.append(0.0)
            point.accelerations.append(0.0)

        # Set the time from start (2 seconds)
        point.time_from_start = Duration(sec=2, nanosec=0)
        trajectory_msg.points = [point]

        self.joint_cmd_publisher.publish(trajectory_msg)
        self.get_logger().info(f'AI commanded joint positions: {[round(p, 2) for p in point.positions]}')

    def execute_avoidance_behavior(self):
        """Execute obstacle avoidance behavior"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # For obstacle avoidance, move to a "safe" position
        # In a real implementation, this would be more sophisticated
        for joint_name in self.joint_names:
            # Move to neutral position (0.0) for all joints
            point.positions.append(0.0)
            point.velocities.append(0.0)
            point.accelerations.append(0.0)

        # Set the time from start (1 second for quick response)
        point.time_from_start = Duration(sec=1, nanosec=0)
        trajectory_msg.points = [point]

        self.joint_cmd_publisher.publish(trajectory_msg)
        self.get_logger().info('AI executed obstacle avoidance behavior')

        # Reset obstacle flag after a short delay
        self.create_timer(1.0, self.reset_obstacle_flag)

    def reset_obstacle_flag(self):
        """Reset the obstacle detection flag"""
        self.obstacle_detected = False
        self.get_logger().info('Obstacle flag reset')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAIAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()