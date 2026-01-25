# Research: ROS 2 Robotic Nervous System

## Decision: ROS 2 Distribution Selection
**Rationale**: Selected ROS 2 Humble Hawksbill (LTS) as it's the long-term support version recommended for production use and educational purposes. It provides stability and extensive documentation for students.

**Alternatives considered**:
- ROS 2 Rolling Ridley (latest but not LTS)
- ROS 2 Foxy Fitzroy (older LTS but reaching EOL)

## Decision: Python as Primary Language for Examples
**Rationale**: Python is more accessible for students learning robotics concepts. The rclpy client library provides excellent support for ROS 2 functionality and is well-documented for educational use.

**Alternatives considered**:
- C++ (more performant but steeper learning curve)
- Both C++ and Python (too broad for initial learning module)

## Decision: Publisher/Subscriber Pattern as First Example
**Rationale**: The pub/sub pattern is the most fundamental ROS 2 communication paradigm. The "Hello World" talker/listener example clearly demonstrates the distributed nature of ROS 2 systems.

**Alternatives considered**:
- Service-based example (more complex for initial learning)
- Action-based example (too advanced for foundational module)

## Decision: Jetson Orin Nano Hardware Context
**Rationale**: The Jetson Orin Nano is a powerful edge AI platform that's commonly used in robotics education and research. It provides sufficient computational power for ROS 2 while being accessible to students.

**Hardware Specifications Relevant to ROS 2**:
- NVIDIA Ampere architecture GPU
- ARM Cortex-A78AE (12-core) CPU
- 32GB LPDDR5 Memory
- Ubuntu 22.04 LTS support
- ROS 2 Humble compatibility

## Decision: URDF Format for Robot Description
**Rationale**: URDF (Unified Robot Description Format) is the standard for robot modeling in ROS ecosystem. It's essential for students to understand for any robotic system development.

**Key URDF Concepts to Cover**:
- Links (rigid bodies)
- Joints (connections between links)
- Visual and collision properties
- Inertial properties

## Decision: Educational Content Structure
**Rationale**: Following the "Concepts → Architecture → Code → Lab → Quiz" structure as specified in the constitution ensures comprehensive learning.

**Structure Elements**:
- Theoretical concepts explanation
- System architecture visualization
- Practical code examples
- Hands-on lab exercises
- Assessment quizzes

## Technical Research: ROS 2 Nodes, Topics, and Services

### Nodes
- ROS 2 nodes are processes that perform computation
- Each node runs independently and communicates with other nodes
- Created using rclpy.create_node() in Python
- Nodes contain publishers, subscribers, services, and clients

### Topics and Message Passing
- Topics enable publish/subscribe communication pattern
- Messages are passed asynchronously between nodes
- Multiple publishers and subscribers can use the same topic
- Quality of Service (QoS) settings control communication behavior

### Services
- Services enable request/response communication pattern
- Synchronous communication between nodes
- One service server handles requests from multiple clients
- Useful for actions that require a response

### Graph Architecture
- All nodes form a distributed graph
- Nodes discover each other through DDS (Data Distribution Service)
- Communication is peer-to-peer without a central master (unlike ROS 1)
- Nodes can be started and stopped independently

## Decision: Development Environment Setup
**Rationale**: Students need clear, reproducible setup instructions to avoid environment-related issues that can hinder learning.

**Requirements**:
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3.8+
- Development tools (colcon for building packages)

## References
- ROS 2 Humble Hawksbill Documentation
- Official ROS 2 Tutorials
- NVIDIA Jetson Documentation
- URDF Tutorials and Examples