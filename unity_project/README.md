# Unity Humanoid Robot Visualization Project

This Unity project provides high-fidelity visualization for the humanoid robot, connecting to ROS 2 via rosbridge for real-time robot state visualization.

## Project Structure
```
unity_project/
├── Assets/
│   ├── Scripts/              # C# scripts for ROS integration and robot control
│   ├── Materials/            # Materials for robot visualization
│   ├── Models/               # Robot models and environment assets
│   ├── Scenes/               # Unity scenes
│   └── Prefabs/              # Reusable robot and environment prefabs
├── Packages/                 # Unity package dependencies
├── ProjectSettings/          # Unity project settings
└── unity_project.sln         # Visual Studio solution file
```

## Setup Instructions

1. **Install Unity Hub and Unity 2021.3 LTS or later**
2. **Open this project in Unity**
3. **Install required packages via Package Manager:**
   - Unity-Rosbridge (or ROS Integration package)
   - Any additional dependencies

## ROS Integration

This project uses rosbridge to connect to ROS 2. Make sure to:
1. Start the rosbridge server: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
2. Ensure your ROS 2 simulation is running with joint states published
3. Connect Unity to the rosbridge WebSocket at ws://127.0.0.1:9090

## Running the Visualization

1. Start your ROS 2 simulation with the humanoid robot
2. Launch the rosbridge server
3. Open this Unity project and run the main scene
4. The Unity robot model should mirror the simulated robot's movements