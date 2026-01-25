# ROS 2 System Architecture

This document describes the architecture of the ROS 2 robotic nervous system, explaining how the different components work together to create a distributed robot control system.

## Overview

The ROS 2 robotic nervous system follows a distributed architecture where multiple nodes communicate through topics, services, and other communication patterns. This architecture mimics biological nervous systems where sensory inputs are processed and motor commands are executed across multiple interconnected components.

## Core Architecture Components

### 1. Node Layer
The node layer consists of individual processes that perform specific functions:

- **Sensor Nodes**: Collect data from physical sensors (cameras, lidars, IMUs, etc.)
- **Processing Nodes**: Process sensor data and make decisions
- **Actuator Nodes**: Control physical actuators (motors, servos, etc.)
- **Management Nodes**: Coordinate system behavior and monitor status

### 2. Communication Layer
The communication layer handles message passing between nodes:

- **Topics**: Asynchronous publish/subscribe for continuous data streams
- **Services**: Synchronous request/response for discrete actions
- **Actions**: Long-running tasks with feedback and status updates
- **Parameters**: Shared configuration values across nodes

### 3. Middleware Layer
ROS 2 uses DDS (Data Distribution Service) as its middleware:

- **Discovery**: Automatic discovery of nodes and their interfaces
- **Transport**: Reliable message delivery between nodes
- **Quality of Service**: Configurable delivery guarantees
- **Security**: Authentication and encryption capabilities

## ROS 2 Graph Architecture

### Distributed Design
Unlike ROS 1 which used a central master, ROS 2 uses a peer-to-peer architecture:

- Each node can discover other nodes directly
- No single point of failure
- Nodes can be added or removed dynamically
- Communication continues even if individual nodes fail

### Naming and Namespacing
- **Node Names**: Unique identifiers for each node
- **Topic/Service Names**: Paths that nodes use to communicate
- **Namespaces**: Hierarchical organization of nodes and topics
- **Remapping**: Ability to change names at runtime

## Quality of Service (QoS) Settings

ROS 2 provides configurable QoS settings for different communication needs:

### Reliability
- **Reliable**: All messages are delivered (like TCP)
- **Best Effort**: Messages may be dropped (like UDP)

### Durability
- **Volatile**: Only new messages are received
- **Transient Local**: Historical messages are available to new subscribers

### History
- **Keep Last**: Only the most recent N messages
- **Keep All**: All messages are stored

## Implementation Patterns

### Publisher-Subscriber Pattern
Used for continuous data streams like sensor data or robot state:

```
Sensor Node (Publisher) → Topic → Processing Node (Subscriber)
```

### Client-Server Pattern
Used for discrete requests like configuration changes or specific computations:

```
Controller Node (Client) → Service → Processing Node (Server)
```

### Action Pattern
Used for long-running tasks with feedback:

```
Controller Node → Action Server → Feedback/Result
```

## Jetson Orin Nano Considerations

When deploying this architecture on the Jetson Orin Nano:

### Computational Constraints
- Optimize algorithms for ARM architecture
- Consider power consumption when designing computation-intensive nodes
- Use efficient data structures and algorithms

### Memory Management
- Monitor memory usage to prevent resource exhaustion
- Use appropriate buffer sizes for message queues
- Consider garbage collection impact on real-time performance

### Real-time Considerations
- Use appropriate QoS settings for time-critical communications
- Minimize latency in critical communication paths
- Consider using real-time scheduling policies when needed

## System Design Principles

### Modularity
- Each node should have a single, well-defined responsibility
- Nodes should be independently deployable and testable
- Clear interfaces between components

### Robustness
- Handle node failures gracefully
- Implement appropriate error handling and recovery
- Use timeouts and retry mechanisms appropriately

### Scalability
- Design for multiple instances of nodes when needed
- Consider resource usage as the system grows
- Use appropriate data structures for the expected scale

## Visualization of Architecture

```
                    +------------------+
                    |   Management     |
                    |     Node         |
                    +--------+---------+
                             |
                    +--------v---------+
                    |  Communication   |
                    |    Layer (DDS)   |
                    +--------+---------+
                             |
        +--------------------+--------------------+
        |                                         |
+-------v--------+                    +-----------v---------+
|   Sensor       |                    |   Processing        |
|   Nodes        |                    |     Nodes           |
|                |                    |                     |
| - Camera       |                    | - Perception        |
| - Lidar        |                    | - Planning          |
| - IMU          |                    | - Control           |
| - GPS          |                    |                     |
+-------+--------+                    +-----------+---------+
        |                                         |
        +-----------------+-----------------------+
                          |
                  +-------v--------+
                  |  Actuator      |
                  |    Nodes       |
                  |                |
                  | - Motor Ctrl   |
                  | - Servo Ctrl   |
                  | - Gripper Ctrl |
                  +----------------+
```

This architecture enables a flexible, robust robotic nervous system that can be adapted to various robot platforms and applications while maintaining the distributed nature of ROS 2.

## ROS 2 Graph Architecture

### Overview
The ROS 2 graph represents the network of nodes, topics, services, and other communication entities. Unlike ROS 1 which used a central master, ROS 2 uses a distributed architecture based on DDS (Data Distribution Service):

- Each node can discover other nodes directly through DDS
- No single point of failure (distributed design)
- Nodes can be added or removed dynamically
- Communication continues even if individual nodes fail
- Automatic discovery of new nodes and communication endpoints

### Graph Components
- **Nodes**: Individual processes performing computation
- **Topics**: Named buses for publish/subscribe communication
- **Services**: Named endpoints for request/response communication
- **Actions**: Named endpoints for long-running tasks with feedback
- **Parameters**: Configuration values shared between nodes
- **Interfaces**: Message and service type definitions

### Discovery Process
1. Nodes join the ROS 2 domain
2. DDS middleware automatically discovers other nodes
3. Communication endpoints (topics, services) are discovered
4. Connections are established based on matching names and types
5. Communication begins automatically

### Domain Isolation
- Multiple ROS 2 domains can run on the same network
- Nodes in different domains are isolated from each other
- Domain ID is used to separate communication spaces
- Useful for testing and multi-robot scenarios