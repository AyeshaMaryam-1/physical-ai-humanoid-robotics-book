import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanSubscriberNode(Node):
    def __init__(self):
        super().__init__('scan_subscriber')

        # Create subscriber for laser scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Create publisher for obstacle detection
        self.obstacle_publisher = self.create_publisher(String, 'obstacle_alert', 10)

        # Declare parameters
        self.declare_parameter('obstacle_distance_threshold', 1.0)

        # Get parameter values
        self.threshold = self.get_parameter('obstacle_distance_threshold').value

        self.get_logger().info('Scan Subscriber Node initialized')

    def scan_callback(self, msg):
        # Check if any range reading is below the threshold
        min_distance = min(msg.ranges)

        if min_distance < self.threshold:
            alert_msg = String()
            alert_msg.data = f'OBSTACLE_DETECTED: {min_distance:.2f}m'
            self.obstacle_publisher.publish(alert_msg)
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')
        else:
            self.get_logger().info(f'Clear path: {min_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()