# Isaac Sim Assets for Humanoid Robotics

This directory contains assets and projects for NVIDIA Isaac Sim, used in the "Physical AI & Humanoid Robotics" book.

## Project Structure

```
isaac_sim_assets/
├── isaac_sim_project/          # Main Isaac Sim project
│   ├── assets/                 # 3D assets and models
│   │   └── robots/             # Robot-specific assets
│   ├── scenes/                 # Isaac Sim scene files
│   ├── usd/                    # Universal Scene Description files
│   ├── scripts/                # Python scripts for Isaac Sim automation
│   └── configs/                # Configuration files
├── usd/                        # Standalone USD files
│   └── humanoid_robot/         # Humanoid robot USD assets
│       ├── parts/              # Individual robot parts
│       └── humanoid.usda       # Main robot assembly
└── README.md                   # This file
```

## Isaac Sim Project Structure

The `isaac_sim_project/` directory follows Isaac Sim conventions:

- `assets/` - Contains reusable 3D assets (ground planes, objects, etc.)
- `scenes/` - Contains complete scene compositions for different experiments
- `usd/` - Contains USD files for importing into scenes
- `scripts/` - Contains Python scripts for automating Isaac Sim tasks
- `configs/` - Contains configuration files for physics, materials, etc.

## Getting Started

1. Install NVIDIA Isaac Sim from the Omniverse Launcher
2. Open Isaac Sim and navigate to this directory
3. Load scenes from the `scenes/` directory
4. Use scripts from the `scripts/` directory for automation

## USD Robot Model

The humanoid robot model is defined in USD format in the `usd/humanoid_robot/` directory. This allows for easy integration with Isaac Sim and provides a foundation for more complex humanoid robots.

## Isaac Sim Scripts

Python scripts in the `scripts/` directory can be run within Isaac Sim to automate robot setup, configuration, and testing workflows.