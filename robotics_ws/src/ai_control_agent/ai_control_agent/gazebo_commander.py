import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import random
import time


class GazeboCommanderNode(Node):
    def __init__(self):
        super().__init__('gazebo_commander')

        # Publisher for joint trajectory commands to control the simulated robot
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states from Gazebo
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for commanding the robot in Gazebo
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.gazebo_command_callback)

        # Robot state tracking
        self.current_joint_positions = {}
        self.command_counter = 0

        # Define joint names for the simple humanoid (from URDF)
        self.joint_names = [
            'left_shoulder', 'left_elbow', 'right_shoulder', 'right_elbow',
            'left_hip', 'left_knee', 'right_hip', 'right_knee'
        ]

        self.get_logger().info('Gazebo Commander Node initialized - Ready to command simulated humanoid robot')

    def joint_state_callback(self, msg):
        """Update current joint positions from joint state messages"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joint_positions[name] = msg.position[i]

    def gazebo_command_callback(self):
        """Send commands to the simulated robot in Gazebo"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # Generate different movement patterns based on command counter
        if self.command_counter % 4 == 0:
            # Wave motion with arms
            self.get_logger().info('Executing wave motion pattern')
            point.positions = [
                0.5,   # left_shoulder
                0.5,   # left_elbow
                -0.5,  # right_shoulder
                0.5,   # right_elbow
                0.0,   # left_hip
                0.0,   # left_knee
                0.0,   # right_hip
                0.0    # right_knee
            ]
        elif self.command_counter % 4 == 1:
            # Standing position
            self.get_logger().info('Returning to standing position')
            point.positions = [0.0] * len(self.joint_names)
        elif self.command_counter % 4 == 2:
            # Squat position
            self.get_logger().info('Executing squat position')
            point.positions = [
                0.0,   # left_shoulder
                0.0,   # left_elbow
                0.0,   # right_shoulder
                0.0,   # right_elbow
                0.3,   # left_hip
                0.6,   # left_knee
                0.3,   # right_hip
                0.6    # right_knee
            ]
        else:
            # Random position
            self.get_logger().info('Executing random position')
            point.positions = [random.uniform(-0.5, 0.5) for _ in self.joint_names]

        # Set velocity and acceleration to 0 for simplicity
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)

        # Set the time from start (3 seconds)
        point.time_from_start = Duration(sec=3, nanosec=0)
        trajectory_msg.points = [point]

        self.joint_cmd_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Commanded joint positions: {[round(p, 2) for p in point.positions]}')

        self.command_counter += 1

        # Log current joint states if available
        if self.current_joint_positions:
            current_pos_list = [round(self.current_joint_positions.get(name, 0.0), 2)
                              for name in self.joint_names]
            self.get_logger().info(f'Current joint positions: {current_pos_list}')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCommanderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Gazebo Commander Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()