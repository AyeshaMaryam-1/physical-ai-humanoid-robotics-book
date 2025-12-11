#!/bin/bash

# Script to verify sensor data streaming from the simulated humanoid robot
# This script checks if sensor topics are publishing data correctly

set -e  # Exit on any error

echo "==========================================="
echo "Sensor Data Verification Script"
echo "==========================================="

# Function to check if a topic is publishing data
check_topic() {
    local topic_name=$1
    local topic_type=$2
    local description=$3

    echo ""
    echo "Checking $description on topic: $topic_name"
    echo "Topic type: $topic_type"

    # Timeout after 5 seconds to check if topic exists
    if timeout 5s ros2 topic info $topic_name >/dev/null 2>&1; then
        echo "✓ Topic $topic_name exists"

        # Try to echo the topic for 2 seconds to see if it's publishing
        if timeout 3s ros2 topic echo $topic_name --field data --field header --field ranges --field image --field depth --field point --field orientation --field linear_acceleration --field angular_velocity $topic_type 2>/dev/null | head -n 5; then
            echo "✓ Topic $topic_name is publishing data"
        else
            echo "⚠ Topic $topic_name may not be publishing data (timeout or no messages)"
        fi
    else
        echo "✗ Topic $topic_name does not exist"
    fi
}

echo "Starting Gazebo simulation with sensors in background..."
# Start the sensor simulation in the background
ros2 launch ai_control_agent sensor_simulation.launch.py > /tmp/sensor_sim.log 2>&1 &
SIM_PID=$!

# Wait a few seconds for the simulation to start
sleep 10

echo "Checking sensor topics..."

# Check camera topics
check_topic "/camera/image_raw" "sensor_msgs/msg/Image" "RGB Camera"
check_topic "/depth_camera/image_raw" "sensor_msgs/msg/Image" "Depth Camera RGB"
check_topic "/depth_camera/depth/image_raw" "sensor_msgs/msg/Image" "Depth Camera Depth"
check_topic "/depth_camera/points" "sensor_msgs/msg/PointCloud2" "Depth Camera Point Cloud"

# Check LiDAR topic
check_topic "/lidar/scan" "sensor_msgs/msg/LaserScan" "3D LiDAR"

# Check IMU topic
check_topic "/imu/data" "sensor_msgs/msg/Imu" "IMU Sensor"

# Check joint states
check_topic "/joint_states" "sensor_msgs/msg/JointState" "Joint States"

# Check robot state publisher
check_topic "/tf" "tf2_msgs/msg/TFMessage" "Transforms (TF)"

echo ""
echo "==========================================="
echo "Sensor Verification Complete"
echo "==========================================="
echo ""
echo "To manually verify sensor data, use these commands:"
echo "  - Camera: ros2 topic echo /camera/image_raw"
echo "  - Depth: ros2 topic echo /depth_camera/depth/image_raw"
echo "  - LiDAR: ros2 topic echo /lidar/scan"
echo "  - IMU: ros2 topic echo /imu/data"
echo "  - All topics: ros2 topic list"
echo ""
echo "To visualize in RViz2:"
echo "  - Run: rviz2"
echo "  - Add displays for Image, LaserScan, RobotModel, etc."
echo ""
echo "Stopping simulation..."
kill $SIM_PID 2>/dev/null || true

echo "Verification script completed!"