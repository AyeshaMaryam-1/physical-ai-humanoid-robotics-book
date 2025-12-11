#!/bin/bash

# Setup script for Physical AI & Humanoid Robotics book project
# This script sets up the development environment for ROS 2, Python dependencies, and Docusaurus

set -e  # Exit on any error

echo "==========================================="
echo "Physical AI & Humanoid Robotics Setup Script"
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

# Check for ROS 2 installation
if command_exists ros2; then
    echo "✓ ROS 2 is already installed"
    ROS_DISTRO=$(ros2 --version | cut -d' ' -f3)
    echo "ROS 2 distribution: $ROS_DISTRO"
else
    echo "⚠ ROS 2 is not installed. Please install ROS 2 (Humble Hawksbill or Iron Irwini) before running this script."
    echo "For Ubuntu: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    echo "For macOS: https://docs.ros.org/en/humble/Installation/macOS.html"
    exit 1
fi

# Check for Python 3
if command_exists python3; then
    echo "✓ Python 3 is installed"
    PYTHON_VERSION=$(python3 --version)
    echo "Python version: $PYTHON_VERSION"
else
    echo "❌ Python 3 is not installed. Please install Python 3.8 or higher."
    exit 1
fi

# Check for pip
if command_exists pip3; then
    echo "✓ pip is installed"
else
    echo "❌ pip is not installed. Please install pip for Python 3."
    exit 1
fi

# Install Python dependencies for ROS 2 packages
echo "Installing Python dependencies..."
pip3 install -r requirements.txt 2>/dev/null || echo "requirements.txt not found, creating a default one..."

# Create default requirements.txt if it doesn't exist
if [ ! -f "requirements.txt" ]; then
    echo "Creating default requirements.txt..."
    cat > requirements.txt << EOF
# Python dependencies for Physical AI & Humanoid Robotics book
# ROS 2 dependencies are typically installed separately via apt/brew

numpy>=1.19.0
scipy>=1.5.0
matplotlib>=3.3.0
opencv-python>=4.5.0
Pillow>=8.0.0
transformers>=4.20.0
torch>=1.12.0
torchvision>=0.13.0
openai-whisper
pyyaml
requests
tqdm
EOF
fi

pip3 install -r requirements.txt

# Setup ROS 2 workspace
echo "Setting up ROS 2 workspace..."
cd robotics_ws
if [ ! -f "src/CMakeLists.txt" ]; then
    echo "Creating ROS 2 workspace structure..."
    ln -s /opt/ros/$ROS_DISTRO/share/ament_cmake_core/cmake/package.cmake src/CMakeLists.txt 2>/dev/null || echo "Using colcon to create workspace structure..."
    # If src directory is empty, initialize it properly
    touch src/.gitkeep
fi
cd ..

# Install Node.js dependencies for Docusaurus
echo "Installing Docusaurus dependencies..."
cd book
if command_exists npm; then
    npm install
else
    if command_exists yarn; then
        yarn install
    else
        echo "❌ Neither npm nor yarn found. Please install Node.js and npm."
        exit 1
    fi
fi
cd ..

# Setup Unity project (just create basic structure if not already done)
echo "Setting up Unity project structure..."
if [ ! -f "unity_project/Packages/manifest.json" ]; then
    echo "Creating Unity project manifest..."
    mkdir -p unity_project/Packages
    cat > unity_project/Packages/manifest.json << EOF
{
  "dependencies": {
    "com.unity.ros-tcp-connector": "https://github.com/Unity-Technologies/ROS-TCP-Connector.git",
    "com.unity.robotics.ros-bridge": "https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.ros-bridge#v0.7.0"
  }
}
EOF
fi

# Setup Isaac Sim assets directory
echo "Setting up Isaac Sim assets structure..."
if [ ! -f "isaac_sim_assets/config.json" ]; then
    echo "Creating Isaac Sim config..."
    cat > isaac_sim_assets/config.json << EOF
{
  "isaac_sim_project": {
    "name": "Physical AI & Humanoid Robotics",
    "description": "Isaac Sim project for the Physical AI & Humanoid Robotics book",
    "version": "1.0.0"
  }
}
EOF
fi

# Setup research notes structure
echo "Setting up research notes structure..."
if [ ! -f "research_notes/README.md" ]; then
    echo "Creating research notes README..."
    cat > research_notes/README.md << EOF
# Research Notes for Physical AI & Humanoid Robotics Book

This directory contains research notes, citations, and references for the book.

## Structure:
- \`notes/\`: Individual research notes by topic
- \`citations.bib\`: BibTeX file for APA-style citations
EOF
fi

echo "==========================================="
echo "Setup completed successfully!"
echo "==========================================="
echo ""
echo "Next steps:"
echo "1. Source your ROS 2 environment: source /opt/ros/\$ROS_DISTRO/setup.bash"
echo "2. Build ROS 2 packages: cd robotics_ws && colcon build"
echo "3. Start Docusaurus development server: cd book && npm run start"
echo ""
echo "For more information, check the book documentation once it's generated."