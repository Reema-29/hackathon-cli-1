# ROS 2 Examples: Robotic Nervous System

This directory contains educational examples for the ROS 2 Robotic Nervous System module. These examples demonstrate fundamental ROS 2 concepts including Nodes, Topics, Services, and URDF (Unified Robot Description Format).

## Directory Structure

```
src/ros2_examples/
├── publisher_subscriber/     # Basic publisher/subscriber examples
│   ├── talker.py            # Publisher node example
│   ├── listener.py          # Subscriber node example
│   └── README.md            # Usage instructions
├── urdf_examples/           # URDF robot description examples
│   ├── simple_robot.urdf    # Basic robot model
│   └── README.md            # URDF explanation and usage
└── ros2_basics/             # Basic ROS 2 patterns and utilities
    ├── node_template.py     # Template for creating new nodes
    └── service_example.py   # Service server/client example
```

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Ubuntu 22.04 LTS (recommended)

## Getting Started

1. Source ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Navigate to the workspace:
   ```bash
   cd ~/ros2_ws
   ```

3. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. Run examples from the appropriate subdirectories.

## Educational Content

For detailed explanations of ROS 2 concepts and lab exercises, see the documentation in the `docs/ros2_nervous_system/` directory.

## Learning Objectives

By working through these examples, you will understand:
- How to create and run ROS 2 nodes
- How to implement publisher/subscriber communication patterns
- How to create and use services for request/response communication
- How to define robot models using URDF
- How these concepts apply to the Jetson Orin Nano hardware platform