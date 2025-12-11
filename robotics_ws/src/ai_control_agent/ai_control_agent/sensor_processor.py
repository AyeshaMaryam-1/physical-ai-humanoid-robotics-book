import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import math


class SensorProcessorNode(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Create subscribers for different sensor types
        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.depth_camera_subscriber = self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.depth_camera_callback,
            10
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10
        )

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publishers for processed sensor data
        self.obstacle_publisher = self.create_publisher(
            LaserScan,
            '/processed_scan',
            10
        )

        # Initialize CV Bridge for image processing
        self.cv_bridge = CvBridge()

        # Statistics tracking
        self.scan_count = 0
        self.camera_count = 0
        self.imu_count = 0

        self.get_logger().info('Sensor Processor Node initialized - Subscribed to all sensor topics')

    def camera_callback(self, msg):
        """Process RGB camera data"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process image (simple example: get image dimensions)
            height, width, channels = cv_image.shape

            self.camera_count += 1
            if self.camera_count % 30 == 0:  # Log every 30th image
                self.get_logger().info(f'Camera: Received image {self.camera_count} - Size: {width}x{height}')

        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')

    def depth_camera_callback(self, msg):
        """Process depth camera data"""
        try:
            # Convert ROS Image message to OpenCV image (depth data)
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "32FC1")

            # Get some statistics from the depth image
            valid_depths = depth_image[depth_image > 0]  # Filter out invalid depths

            if len(valid_depths) > 0:
                avg_depth = np.mean(valid_depths)
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)

                if self.camera_count % 30 == 0:  # Log every 30th depth image
                    self.get_logger().info(
                        f'Depth Camera: Avg: {avg_depth:.2f}m, Min: {min_depth:.2f}m, Max: {max_depth:.2f}m'
                    )
        except Exception as e:
            self.get_logger().error(f'Error processing depth camera image: {str(e)}')

    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection"""
        try:
            # Process laser scan data
            ranges = np.array(msg.ranges)

            # Filter out invalid ranges (inf, nan) and get valid distances
            valid_ranges = ranges[np.isfinite(ranges)]

            if len(valid_ranges) > 0:
                min_distance = np.min(valid_ranges)
                avg_distance = np.mean(valid_ranges)

                # Check for obstacles in front (first 30 degrees)
                front_ranges = ranges[:len(ranges)//12]  # First 30 degrees (360/12)
                front_valid = front_ranges[np.isfinite(front_ranges)]

                if len(front_valid) > 0 and np.min(front_valid) < 1.0:  # Obstacle within 1m
                    self.get_logger().warn(f'OBSTACLE DETECTED: {np.min(front_valid):.2f}m ahead!')
                else:
                    self.get_logger().info(f'Clear path ahead: min distance {min_distance:.2f}m')

                # Publish processed scan with obstacle information
                processed_scan = LaserScan()
                processed_scan.header = msg.header
                processed_scan.angle_min = msg.angle_min
                processed_scan.angle_max = msg.angle_max
                processed_scan.angle_increment = msg.angle_increment
                processed_scan.time_increment = msg.time_increment
                processed_scan.scan_time = msg.scan_time
                processed_scan.range_min = msg.range_min
                processed_scan.range_max = msg.range_max
                processed_scan.ranges = msg.ranges

                self.obstacle_publisher.publish(processed_scan)

                self.scan_count += 1
                if self.scan_count % 10 == 0:  # Log every 10th scan
                    self.get_logger().info(
                        f'LiDAR: {len(valid_ranges)} valid readings, min: {min_distance:.2f}m, avg: {avg_distance:.2f}m'
                    )
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {str(e)}')

    def imu_callback(self, msg):
        """Process IMU data for orientation and acceleration"""
        try:
            # Extract orientation (quaternion)
            orientation = msg.orientation
            # Convert quaternion to Euler angles (simplified)
            # Note: In a real implementation, use tf2 for proper conversion
            roll = math.atan2(
                2.0 * (orientation.w * orientation.x + orientation.y * orientation.z),
                1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y)
            )
            pitch = math.asin(
                2.0 * (orientation.w * orientation.y - orientation.z * orientation.x)
            )
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
            )

            # Extract linear acceleration
            linear_acc = msg.linear_acceleration

            self.imu_count += 1
            if self.imu_count % 50 == 0:  # Log every 50th IMU reading
                self.get_logger().info(
                    f'IMU: Roll: {math.degrees(roll):.1f}°, Pitch: {math.degrees(pitch):.1f}°, '
                    f'Yaw: {math.degrees(yaw):.1f}°, Acc: [{linear_acc.x:.2f}, {linear_acc.y:.2f}, {linear_acc.z:.2f}]'
                )
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Sensor Processor Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()