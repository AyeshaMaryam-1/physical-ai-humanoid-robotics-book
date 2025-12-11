import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class VelPublisherNode(Node):
    def __init__(self):
        super().__init__('vel_publisher')

        # Create publisher for cmd_vel topic
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscriber for target velocity
        self.target_subscriber = self.create_subscription(
            Float64,
            'target_velocity',
            self.target_velocity_callback,
            10
        )

        # Declare parameters
        self.declare_parameter('linear_velocity', 0.5)
        self.declare_parameter('angular_velocity', 0.25)

        # Get parameter values
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value

        # Timer for publishing velocity commands
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Velocity Publisher Node initialized')

    def target_velocity_callback(self, msg):
        self.linear_vel = msg.data
        self.get_logger().info(f'Updated linear velocity to: {self.linear_vel}')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        self.vel_publisher.publish(msg)
        self.get_logger().info(f'Publishing velocity: linear={msg.linear.x}, angular={msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = VelPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()