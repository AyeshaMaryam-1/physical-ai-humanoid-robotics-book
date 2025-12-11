#!/usr/bin/env python3

"""
Navigation Goal Sender

This script sends navigation goals to the Nav2 stack for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import time


class NavigationGoalSender(Node):
    def __init__(self):
        super().__init__('navigation_goal_sender')

        # Create action client for navigation
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Wait for the action server to be available
        self.nav_to_pose_client.wait_for_server()

        self.get_logger().info("Navigation goal sender initialized")

    def send_goal(self, x, y, theta):
        """Send a navigation goal to the specified pose"""
        goal_msg = NavigateToPose.Goal()

        # Set the goal pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion
        import math
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        # Send the goal
        self.get_logger().info(f"Sending navigation goal to ({x}, {y}, {theta})")
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result of the navigation"""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Create the navigation goal sender
    nav_sender = NavigationGoalSender()

    # Send a navigation goal (example: go to x=5, y=5, theta=0)
    nav_sender.send_goal(5.0, 5.0, 0.0)

    # Spin to process callbacks
    rclpy.spin(nav_sender)


if __name__ == '__main__':
    main()