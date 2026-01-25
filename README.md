# ROS 2 Robotic Nervous System

This project implements the foundational ROS 2 concepts module covering Nodes, Topics, Services, and URDF for the robotic nervous system curriculum. It includes educational content and practical lab exercises focused on Python-based Publisher/Subscriber implementations for the Jetson Orin Nano hardware platform.

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Project Structure](#project-structure)
4. [Getting Started](#getting-started)
5. [Learning Modules](#learning-modules)
6. [Examples](#examples)
7. [Testing](#testing)
8. [Hardware Integration](#hardware-integration)
9. [Troubleshooting](#troubleshooting)

## Overview

The ROS 2 Robotic Nervous System module teaches students how to create the nervous system of a robot using ROS 2. This includes:

- **Nodes**: Individual processes that perform computation
- **Topics**: Named buses for publish/subscribe communication
- **Services**: Request/response communication patterns
- **URDF**: Unified Robot Description Format for robot modeling
- **Jetson Orin Nano Integration**: Hardware-specific considerations

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Basic Python programming knowledge
- Understanding of robotics concepts (helpful but not required)

## Project Structure

```
hackathon-cli/
├── src/
│   └── ros2_examples/              # ROS 2 code examples
│       ├── publisher_subscriber/    # Basic pub/sub examples
│       ├── urdf_examples/          # URDF robot models
│       └── ros2_basics/            # Basic ROS 2 patterns
├── docs/
│   └── ros2_nervous_system/        # Educational content
│       ├── concepts.md             # ROS 2 fundamentals
│       ├── architecture.md         # System architecture
│       ├── lab_exercises.md        # Step-by-step labs
│       └── jetson_integration.md   # Jetson-specific guide
├── tests/
│   └── integration/                # Integration tests
│       └── test_ros2_communication.py
├── specs/
│   └── 001-ros2-nervous-system/    # Specification documents
│       ├── spec.md                 # Feature specification
│       ├── plan.md                 # Implementation plan
│       ├── research.md             # Research findings
│       ├── data-model.md           # Data models
│       ├── quickstart.md           # Quick start guide
│       ├── tasks.md                # Implementation tasks
│       ├── contracts/              # API contracts
│       └── checklists/             # Quality checklists
└── history/
    └── prompts/                    # Prompt history records
```

## Getting Started

### 1. Environment Setup

First, ensure you have ROS 2 Humble installed:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Create a workspace (if you don't have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Clone and Build

This project should be placed in your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
# Copy this project to the src directory
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. Run Basic Examples

#### Publisher/Subscriber Example

1. Run the publisher (in one terminal):
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/hackathon-cli/src/ros2_examples/publisher_subscriber/talker.py
```

2. Run the subscriber (in another terminal):
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/hackathon-cli/src/ros2_examples/publisher_subscriber/listener.py
```

3. Observe the communication between nodes.

## Learning Modules

### Module 1: ROS 2 Fundamentals

Start with the documentation in `docs/ros2_nervous_system/`:
- `concepts.md`: Core ROS 2 concepts explained
- `architecture.md`: System architecture overview
- `lab_exercises.md`: Hands-on lab exercises

### Module 2: Practical Implementation

Work through the examples in `src/ros2_examples/`:
- `publisher_subscriber/`: Basic communication patterns
- `urdf_examples/`: Robot description examples
- `ros2_basics/`: Basic node templates

### Module 3: Hardware Integration

For Jetson Orin Nano deployment:
- `docs/ros2_nervous_system/jetson_integration.md`: Hardware-specific guide
- Performance considerations and optimization tips

## Examples

### Publisher/Subscriber Pattern
- **Location**: `src/ros2_examples/publisher_subscriber/`
- **Files**: `talker.py`, `listener.py`
- **Concept**: Asynchronous communication between nodes

### Service Pattern
- **Location**: `src/ros2_examples/ros2_basics/service_example.py`
- **Concept**: Synchronous request/response communication

### URDF Robot Model
- **Location**: `src/ros2_examples/urdf_examples/simple_robot.urdf`
- **Concept**: Robot description and modeling

## Testing

Integration tests are available in `tests/integration/`:

```bash
# Run the integration tests
python3 tests/integration/test_ros2_communication.py
```

## Hardware Integration

This module is designed with the Jetson Orin Nano in mind. For deployment considerations, see `docs/ros2_nervous_system/jetson_integration.md` which covers:

- Power management
- Thermal considerations
- Performance optimization
- Resource monitoring
- Package installation for Jetson

## Troubleshooting

### Common Issues

1. **Nodes don't communicate**: Ensure both terminals have sourced the ROS 2 environment
2. **Import errors**: Verify ROS 2 Humble is properly installed
3. **Permission errors**: Check network and file permissions
4. **Performance issues**: Monitor system resources on embedded platforms

### Useful Commands

```bash
# List active nodes
ros2 node list

# List active topics
ros2 topic list

# Show topic information
ros2 topic info /chatter

# Echo messages on a topic
ros2 topic echo /chatter std_msgs/msg/String
```

## Educational Objectives

After completing this module, students will be able to:
- Explain the difference between Nodes, Topics, and Services in ROS 2
- Create and run simple Python nodes that communicate via topics
- Understand how ROS 2 concepts apply to the Jetson Orin Nano platform
- Successfully execute a "Hello World" Publisher/Subscriber example

## Next Steps

For advanced learning, consider:
- Creating custom message types
- Implementing action servers for long-running tasks
- Exploring navigation and perception packages
- Developing robot-specific applications
- Deploying on physical hardware

## References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [NVIDIA Jetson Documentation](https://developer.nvidia.com/embedded/jetson-developer-kit)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)