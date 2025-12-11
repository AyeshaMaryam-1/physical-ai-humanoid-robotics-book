#!/usr/bin/env python3

"""
Isaac ROS Sensor Fusion Node

This node fuses data from multiple sensors (camera, LiDAR, IMU) to create
a comprehensive perception of the environment for the humanoid robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point, Pose, PoseArray
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray, Marker


class IsaacSensorFusionNode(Node):
    def __init__(self):
        super().__init__('isaac_sensor_fusion_node')

        # Create subscribers for all sensor data
        self.image_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/isaac_object_detector/detections',
            self.detection_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.pointcloud_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publisher for fused perception results
        self.fused_objects_pub = self.create_publisher(
            PoseArray,
            '/isaac_sensor_fusion/fused_objects',
            10
        )

        # Create publisher for visualization markers
        self.fused_marker_pub = self.create_publisher(
            MarkerArray,
            '/isaac_sensor_fusion/fused_markers',
            10
        )

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Storage for sensor data
        self.latest_detections = None
        self.latest_pointcloud = None
        self.latest_imu = None

        # Fusion parameters
        self.fusion_time_window = 0.1  # 100ms window for fusion
        self.confidence_threshold = 0.6

        self.get_logger().info("Isaac Sensor Fusion Node initialized")

    def image_callback(self, msg):
        """Process incoming camera image"""
        # Store timestamp for synchronization
        self.image_timestamp = msg.header.stamp

    def detection_callback(self, msg):
        """Process incoming object detections"""
        self.latest_detections = msg
        self.process_fusion()

    def pointcloud_callback(self, msg):
        """Process incoming point cloud"""
        try:
            # Convert to numpy array for processing
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            self.latest_pointcloud = np.array(points_list) if points_list else np.array([])
        except Exception as e:
            self.get_logger().error(f"Could not read point cloud: {e}")

        self.process_fusion()

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        self.latest_imu = msg
        self.process_fusion()

    def process_fusion(self):
        """Process and fuse sensor data"""
        if self.latest_detections is None or self.latest_pointcloud is None or len(self.latest_pointcloud) == 0:
            return

        # Project 2D detections into 3D space using point cloud data
        fused_objects = self.project_detections_to_3d(
            self.latest_detections,
            self.latest_pointcloud
        )

        # Create PoseArray message with fused objects
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"  # Assuming a map frame
        pose_array.poses = fused_objects

        # Publish fused objects
        self.fused_objects_pub.publish(pose_array)

        # Create visualization markers
        marker_array = self.create_fusion_markers(fused_objects)
        self.fused_marker_pub.publish(marker_array)

        self.get_logger().info(f"Published {len(fused_objects)} fused objects")

    def project_detections_to_3d(self, detections, pointcloud):
        """Project 2D image detections into 3D space using point cloud data"""
        fused_poses = []

        for detection in detections.detections:
            # Get bounding box center in image coordinates
            center_x = int(detection.bbox.center.x)
            center_y = int(detection.bbox.center.y)

            # Find corresponding 3D points in the point cloud
            # This is a simplified approach - in reality, you'd need camera intrinsics
            # and a more sophisticated projection method
            if len(pointcloud) > 0:
                # For demonstration, we'll take the nearest point to the detection center
                # In a real implementation, you'd use camera projection matrices
                avg_point = np.mean(pointcloud, axis=0) if len(pointcloud) < 100 else pointcloud[len(pointcloud)//2]

                pose = Pose()
                pose.position.x = float(avg_point[0])
                pose.position.y = float(avg_point[1])
                pose.position.z = float(avg_point[2])
                # Set orientation based on IMU data if available
                if self.latest_imu:
                    pose.orientation = self.latest_imu.orientation
                else:
                    pose.orientation.w = 1.0  # Default orientation

                fused_poses.append(pose)

        return fused_poses

    def create_fusion_markers(self, poses):
        """Create visualization markers for fused objects"""
        marker_array = MarkerArray()

        for i, pose in enumerate(poses):
            # Create a marker for each fused object
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"
            marker.ns = "fused_objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose = pose
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        return marker_array


def main(args=None):
    rclpy.init(args=args)

    fusion_node = IsaacSensorFusionNode()

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        fusion_node.get_logger().info("Shutting down Isaac Sensor Fusion Node")
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()