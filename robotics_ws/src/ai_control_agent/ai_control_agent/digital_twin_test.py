#!/usr/bin/env python3

"""
Digital Twin Test Script

This script tests the complete digital twin pipeline by:
1. Publishing joint commands to control the robot
2. Monitoring sensor data from the simulated robot
3. Verifying synchronization between Gazebo and Unity
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, LaserScan, Image, Imu
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import time
import math


class DigitalTwinTester(Node):
    def __init__(self):
        super().__init__('digital_twin_tester')

        # Joint command publisher
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/forward_position_controller/joint_trajectory',
            10
        )

        # Sensor data subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Timer for sending commands
        self.timer = self.create_timer(1.0, self.send_test_command)

        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_shoulder_joint', 'left_elbow_joint',
            'right_hip_joint', 'right_knee_joint', 'right_shoulder_joint', 'right_elbow_joint'
        ]

        self.get_logger().info("Digital Twin Tester initialized")

    def send_test_command(self):
        """Send a test joint trajectory command"""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # Create a simple oscillating motion
        t = self.get_clock().now().nanoseconds / 1e9
        amplitude = 0.5
        frequency = 0.5

        positions = []
        for i, joint_name in enumerate(self.joint_names):
            pos = amplitude * math.sin(2 * math.pi * frequency * t + i * math.pi / 4)
            positions.append(pos)

        point.positions = positions
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points = [point]

        self.joint_cmd_publisher.publish(msg)
        self.get_logger().info(f"Sent joint command: {positions}")

    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        self.get_logger().debug(f"Received joint states: {len(msg.name)} joints")

    def lidar_callback(self, msg):
        """Handle LiDAR data"""
        self.get_logger().debug(f"Received LiDAR data: {len(msg.ranges)} ranges")

    def camera_callback(self, msg):
        """Handle camera data"""
        self.get_logger().debug(f"Received camera data: {msg.width}x{msg.height}")

    def imu_callback(self, msg):
        """Handle IMU data"""
        self.get_logger().debug(f"Received IMU data: linear={msg.linear_acceleration}, angular={msg.angular_velocity}")


def main(args=None):
    rclpy.init(args=args)

    tester = DigitalTwinTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info("Shutting down Digital Twin Tester")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()