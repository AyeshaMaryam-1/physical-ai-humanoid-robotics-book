#!/usr/bin/env python3

"""
Isaac ROS Point Cloud Processor

This node processes LiDAR point cloud data from Isaac Sim
for segmentation and object detection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point as GeometryPoint


class IsaacPointCloudProcessor(Node):
    def __init__(self):
        super().__init__('isaac_pointcloud_processor')

        # Create subscriber for point cloud
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.pointcloud_callback,
            10
        )

        # Create publisher for processed point cloud
        self.processed_pc_pub = self.create_publisher(
            PointCloud2,
            '/isaac_pointcloud_processor/processed_points',
            10
        )

        # Create publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/isaac_pointcloud_processor/markers',
            10
        )

        # Parameters for ground plane segmentation
        self.ground_threshold = 0.1  # Threshold for ground plane detection
        self.segmentation_distance = 0.2  # Distance threshold for clustering

        self.get_logger().info("Isaac Point Cloud Processor initialized")

    def pointcloud_callback(self, msg):
        """Process incoming point cloud data"""
        try:
            # Convert PointCloud2 to list of points
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            points = np.array(points_list)
        except Exception as e:
            self.get_logger().error(f"Could not read point cloud: {e}")
            return

        if len(points) == 0:
            return

        # Segment ground plane using simple height-based filtering
        ground_points, obstacle_points = self.segment_ground_plane(points)

        # Perform clustering on obstacle points
        clusters = self.cluster_points(obstacle_points)

        # Publish processed point cloud (obstacle points)
        processed_pc_msg = self.create_pointcloud_msg(obstacle_points, msg.header)
        self.processed_pc_pub.publish(processed_pc_msg)

        # Publish visualization markers for clusters
        marker_array = self.create_cluster_markers(clusters, msg.header)
        self.marker_pub.publish(marker_array)

    def segment_ground_plane(self, points):
        """Simple ground plane segmentation based on Z coordinate"""
        # For a humanoid robot, assume ground is around z=0
        ground_mask = np.abs(points[:, 2]) < self.ground_threshold
        ground_points = points[ground_mask]
        obstacle_points = points[~ground_mask]

        return ground_points, obstacle_points

    def cluster_points(self, points):
        """Simple clustering algorithm for obstacle detection"""
        if len(points) == 0:
            return []

        clusters = []
        visited = set()

        for i, point in enumerate(points):
            if i in visited:
                continue

            # Start a new cluster
            cluster = [point]
            visited.add(i)

            # Find neighboring points
            for j, other_point in enumerate(points):
                if j in visited:
                    continue

                # Calculate distance
                dist = np.linalg.norm(point - other_point)
                if dist < self.segmentation_distance:
                    cluster.append(other_point)
                    visited.add(j)

            if len(cluster) > 1:  # Only consider clusters with multiple points
                clusters.append(np.array(cluster))

        return clusters

    def create_pointcloud_msg(self, points, header):
        """Create a PointCloud2 message from numpy array"""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Create list of points in the required format
        point_list = []
        for point in points:
            point_list.append([point[0], point[1], point[2]])

        return pc2.create_cloud(header, fields, point_list)

    def create_cluster_markers(self, clusters, header):
        """Create visualization markers for clusters"""
        marker_array = MarkerArray()

        for i, cluster in enumerate(clusters):
            # Create a marker for the cluster centroid
            marker = Marker()
            marker.header = header
            marker.ns = "clusters"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Calculate centroid
            centroid = np.mean(cluster, axis=0)
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.pose.orientation.w = 1.0

            # Set size and color
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        return marker_array


def main(args=None):
    rclpy.init(args=args)

    pc_processor = IsaacPointCloudProcessor()

    try:
        rclpy.spin(pc_processor)
    except KeyboardInterrupt:
        pc_processor.get_logger().info("Shutting down Isaac Point Cloud Processor")
    finally:
        pc_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()