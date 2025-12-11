#!/usr/bin/env python3

"""
Physical Demonstration Script for Sim-to-Real Transfer

This script orchestrates the physical demonstration of sim-to-real transfer
by managing the execution of navigation tasks on the physical robot and
collecting performance metrics for comparison with simulation.
"""

import rospy
import time
import json
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from humanoid_nav2_config.scripts.send_navigation_goal import NavigationGoalSender
import datetime

class PhysicalDemonstrationRunner:
    def __init__(self):
        rospy.init_node('physical_demonstration_runner')

        # Initialize navigation goal sender
        self.nav_sender = NavigationGoalSender()

        # Publishers for metrics
        self.success_rate_pub = rospy.Publisher('/demonstration/success_rate', Float32, queue_size=1)
        self.time_efficiency_pub = rospy.Publisher('/demonstration/time_efficiency', Float32, queue_size=1)
        self.path_efficiency_pub = rospy.Publisher('/demonstration/path_efficiency', Float32, queue_size=1)

        # Subscribers for robot state
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Store robot trajectory for analysis
        self.trajectory = []
        self.start_time = None
        self.task_start_pos = None

        # Demo configuration
        self.demo_config = {
            'waypoints': [
                {'x': 2.0, 'y': 2.0, 'theta': 0.0},
                {'x': 4.0, 'y': 1.0, 'theta': 1.57},
                {'x': 3.0, 'y': 4.0, 'theta': 3.14}
            ],
            'obstacles': [
                {'x': 1.5, 'y': 1.5, 'radius': 0.3},
                {'x': 3.5, 'y': 2.5, 'radius': 0.4},
                {'x': 2.5, 'y': 3.5, 'radius': 0.25}
            ],
            'max_time': 300  # 5 minutes maximum
        }

        rospy.loginfo("Physical demonstration runner initialized")

    def odom_callback(self, msg):
        """Store robot trajectory for path efficiency calculation"""
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.trajectory.append(pos)

        if self.task_start_pos is None:
            self.task_start_pos = pos
            self.start_time = rospy.Time.now()

    def calculate_metrics(self, success, end_time, goal_pos):
        """Calculate performance metrics for the demonstration"""
        if not self.trajectory or not self.task_start_pos:
            return None

        # Calculate actual path length
        total_distance = 0.0
        for i in range(1, len(self.trajectory)):
            dx = self.trajectory[i][0] - self.trajectory[i-1][0]
            dy = self.trajectory[i][1] - self.trajectory[i-1][1]
            total_distance += np.sqrt(dx*dx + dy*dy)

        # Calculate straight-line distance to goal
        start_to_goal = np.sqrt(
            (goal_pos[0] - self.task_start_pos[0])**2 +
            (goal_pos[1] - self.task_start_pos[1])**2
        )

        # Calculate time efficiency (actual time vs optimal time)
        elapsed_time = (end_time - self.start_time).to_sec() if self.start_time else 0
        estimated_optimal_time = start_to_goal / 0.5  # Assuming 0.5 m/s optimal speed
        time_efficiency = estimated_optimal_time / elapsed_time if elapsed_time > 0 else 0.0

        # Calculate path efficiency
        path_efficiency = start_to_goal / total_distance if total_distance > 0 else 0.0

        return {
            'success': success,
            'time_efficiency': min(time_efficiency, 1.0),  # Cap at 1.0
            'path_efficiency': min(path_efficiency, 1.0),  # Cap at 1.0
            'actual_time': elapsed_time,
            'optimal_time': estimated_optimal_time,
            'actual_distance': total_distance,
            'straight_line_distance': start_to_goal
        }

    def execute_single_task(self, goal_x, goal_y, goal_theta):
        """Execute a single navigation task and collect metrics"""
        rospy.loginfo(f"Starting navigation task to ({goal_x}, {goal_y}, {goal_theta})")

        # Reset trajectory tracking
        self.trajectory = []
        self.start_time = rospy.Time.now()
        self.task_start_pos = None

        # Send navigation goal
        self.nav_sender.send_goal(goal_x, goal_y, goal_theta)

        # Wait for completion or timeout
        start_wait = rospy.Time.now()
        max_wait_time = rospy.Duration(self.demo_config['max_time'])

        success = False
        while (rospy.Time.now() - start_wait) < max_wait_time:
            # Check if goal is reached (simple distance check)
            if self.trajectory and self.task_start_pos:
                current_pos = self.trajectory[-1]
                dist_to_goal = np.sqrt((current_pos[0] - goal_x)**2 + (current_pos[1] - goal_y)**2)

                if dist_to_goal < 0.3:  # Within 30cm of goal
                    success = True
                    break

            rospy.sleep(0.1)

        end_time = rospy.Time.now()
        metrics = self.calculate_metrics(success, end_time, (goal_x, goal_y))

        if metrics:
            rospy.loginfo(f"Task completed with metrics: {metrics}")

            # Publish metrics
            self.success_rate_pub.publish(Float32(metrics['success']))
            self.time_efficiency_pub.publish(Float32(metrics['time_efficiency']))
            self.path_efficiency_pub.publish(Float32(metrics['path_efficiency']))

            return metrics
        else:
            rospy.logwarn("Could not calculate metrics for task")
            return None

    def run_demonstration_sequence(self):
        """Run the complete demonstration sequence"""
        rospy.loginfo("Starting physical demonstration sequence")

        results = []

        # Execute tasks at each waypoint
        for i, waypoint in enumerate(self.demo_config['waypoints']):
            rospy.loginfo(f"Executing task {i+1}/{len(self.demo_config['waypoints'])}")

            metrics = self.execute_single_task(
                waypoint['x'],
                waypoint['y'],
                waypoint['theta']
            )

            if metrics:
                results.append(metrics)

            # Brief pause between tasks
            rospy.sleep(2.0)

        # Calculate overall performance
        if results:
            avg_success_rate = sum(1 for r in results if r['success']) / len(results)
            avg_time_efficiency = sum(r['time_efficiency'] for r in results if r) / len([r for r in results if r])
            avg_path_efficiency = sum(r['path_efficiency'] for r in results if r) / len([r for r in results if r])

            overall_metrics = {
                'total_tasks': len(results),
                'successful_tasks': sum(1 for r in results if r['success']),
                'avg_success_rate': avg_success_rate,
                'avg_time_efficiency': avg_time_efficiency,
                'avg_path_efficiency': avg_path_efficiency,
                'timestamp': str(datetime.datetime.now())
            }

            rospy.loginfo(f"Demonstration completed with overall metrics: {overall_metrics}")

            # Save results to file
            with open('/tmp/physical_demonstration_results.json', 'w') as f:
                json.dump(overall_metrics, f, indent=2)

            return overall_metrics

        return None

    def run(self):
        """Main execution loop"""
        rospy.loginfo("Physical demonstration runner starting...")

        # Wait for services to be available
        rospy.sleep(5.0)

        # Run demonstration sequence
        results = self.run_demonstration_sequence()

        if results:
            rospy.loginfo("Demonstration completed successfully!")
            rospy.loginfo(f"Final success rate: {results['avg_success_rate']*100:.1f}%")
        else:
            rospy.logerr("Demonstration failed to produce results")


if __name__ == '__main__':
    demo_runner = PhysicalDemonstrationRunner()
    demo_runner.run()