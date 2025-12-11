#!/usr/bin/env python3

"""
Isaac ROS Object Detection Node

This node processes camera images from Isaac Sim to detect objects
using a simple color-based detection algorithm (as a starting point).
In a real implementation, this would use Isaac ROS extensions for
GPU-accelerated object detection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class IsaacObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('isaac_object_detector_node')

        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/isaac_object_detector/detections',
            10
        )

        # Create publisher for annotated images
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/isaac_object_detector/annotated_image',
            10
        )

        # Initialize OpenCV bridge
        self.cv_bridge = CvBridge()

        # Detection parameters
        self.min_area = 100  # Minimum area for valid detection
        self.confidence_threshold = 0.5

        # Define colors to detect (in BGR format)
        self.colors_to_detect = {
            'red': ([0, 0, 100], [50, 50, 255]),
            'blue': ([100, 0, 0], [255, 50, 50]),
            'green': ([0, 100, 0], [50, 255, 50])
        }

        self.get_logger().info("Isaac Object Detector Node initialized")

    def image_callback(self, msg):
        """Process incoming camera image and detect objects"""
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # Perform object detection
        detections = self.detect_objects(cv_image)

        # Create Detection2DArray message
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        # Annotate image with detections
        annotated_image = cv_image.copy()

        for detection in detections:
            # Create detection message
            detection_msg = Detection2D()
            detection_msg.header = msg.header

            # Set bounding box
            bbox = detection['bbox']
            detection_msg.bbox.size_x = bbox[2]
            detection_msg.bbox.size_y = bbox[3]
            detection_msg.bbox.center.x = bbox[0] + bbox[2] / 2
            detection_msg.bbox.center.y = bbox[1] + bbox[3] / 2

            # Set object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = detection['label']
            hypothesis.hypothesis.score = detection['confidence']
            detection_msg.results.append(hypothesis)

            # Add to array
            detection_array_msg.detections.append(detection_msg)

            # Draw bounding box on annotated image
            cv2.rectangle(
                annotated_image,
                (int(bbox[0]), int(bbox[1])),
                (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])),
                (0, 255, 0),
                2
            )

            # Add label text
            cv2.putText(
                annotated_image,
                f"{detection['label']}: {detection['confidence']:.2f}",
                (int(bbox[0]), int(bbox[1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

        # Publish detections
        self.detection_pub.publish(detection_array_msg)

        # Publish annotated image
        try:
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Could not convert annotated image: {e}")

    def detect_objects(self, image):
        """Detect objects in the image using color-based detection"""
        detections = []

        # Convert image to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for label, (lower, upper) in self.colors_to_detect.items():
            # Create mask for the color range
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")
            mask = cv2.inRange(hsv, lower, upper)

            # Apply morphological operations to clean up the mask
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Calculate area and filter small contours
                area = cv2.contourArea(contour)
                if area > self.min_area:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Calculate confidence based on area
                    # In a real implementation, this would come from a detection model
                    confidence = min(0.9, area / 1000.0)

                    if confidence > self.confidence_threshold:
                        detection = {
                            'label': label,
                            'confidence': confidence,
                            'bbox': [x, y, w, h]
                        }
                        detections.append(detection)

        return detections


def main(args=None):
    rclpy.init(args=args)

    detector_node = IsaacObjectDetectorNode()

    try:
        rclpy.spin(detector_node)
    except KeyboardInterrupt:
        detector_node.get_logger().info("Shutting down Isaac Object Detector Node")
    finally:
        detector_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()