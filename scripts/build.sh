#!/bin/bash

# Build script for Physical AI & Humanoid Robotics book project
# This script builds all project components: Docusaurus site and ROS 2 packages

set -e  # Exit on any error

echo "==========================================="
echo "Physical AI & Humanoid Robotics Build Script"
echo "==========================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if running on a supported platform
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    PLATFORM="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    PLATFORM="macos"
else
    echo "Unsupported platform: $OSTYPE"
    exit 1
fi

echo "Detected platform: $PLATFORM"

# Build Docusaurus site
echo "Building Docusaurus site..."
cd book
if command_exists npm; then
    npm run build
elif command_exists yarn; then
    yarn build
else
    echo "❌ Neither npm nor yarn found. Cannot build Docusaurus site."
    exit 1
fi
cd ..

echo "✓ Docusaurus site built successfully"

# Build ROS 2 packages if workspace exists
if [ -d "robotics_ws" ] && [ -d "robotics_ws/src" ]; then
    echo "Building ROS 2 packages..."
    cd robotics_ws

    # Source ROS 2 environment if not already sourced
    if ! command_exists ros2; then
        echo "Sourcing ROS 2 environment..."
        source /opt/ros/*/setup.bash 2>/dev/null || echo "Could not source ROS 2 environment. Make sure ROS 2 is installed and sourced."
    fi

    # Build the workspace using colcon
    colcon build --packages-select humanoid_description sim_control isaac_perception nav2_integration vla_agents vla_voice_input vla_llm_planner vla_multi_modal vla_system_manager 2>/dev/null || {
        echo "Building all packages in workspace..."
        colcon build
    }

    # Source the built packages
    source install/setup.bash

    cd ..
    echo "✓ ROS 2 packages built successfully"
else
    echo "⚠ ROS 2 workspace not found or not properly set up. Skipping ROS 2 build."
fi

# Verify Unity project structure
if [ -d "unity_project" ]; then
    echo "✓ Unity project structure exists"
else
    echo "⚠ Unity project structure not found"
fi

# Verify Isaac Sim assets structure
if [ -d "isaac_sim_assets" ]; then
    echo "✓ Isaac Sim assets structure exists"
else
    echo "⚠ Isaac Sim assets structure not found"
fi

# Verify research notes structure
if [ -f "research_notes/citations.bib" ]; then
    echo "✓ Research notes structure exists"
else
    echo "⚠ Research notes structure not found"
fi

echo "==========================================="
echo "Build completed successfully!"
echo "==========================================="
echo ""
echo "Build artifacts:"
echo "- Docusaurus site: book/build/ (static HTML files)"
echo "- ROS 2 packages: robotics_ws/install/ (if built successfully)"
echo ""
echo "To serve the Docusaurus site locally: cd book && npm run serve"
echo "To run ROS 2 nodes: source robotics_ws/install/setup.bash && ros2 run <package> <node>"