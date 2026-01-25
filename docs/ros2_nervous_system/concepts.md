# ROS 2 Concepts Explained

This document provides detailed explanations of the core ROS 2 concepts that form the foundation of a robot's nervous system.

## Table of Contents
1. [Nodes](#nodes)
2. [Topics](#topics)
3. [Services](#services)
4. [URDF](#urdf)
5. [ROS 2 Graph Architecture](#ros-2-graph-architecture)

## Nodes

### Definition
A ROS 2 node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Each node runs independently and communicates with other nodes to form a distributed system.

### Key Characteristics
- Nodes are processes that perform computation
- Each node is identified by a unique name
- Nodes can be written in different programming languages
- Nodes can be started and stopped independently
- Nodes communicate with each other through topics, services, and actions

### Node Lifecycle
- **Unconfigured**: Node is created but not yet configured
- **Inactive**: Node is configured but not active
- **Active**: Node is running and can perform operations
- **Finalized**: Node has been shut down

### Creating Nodes in Python
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize node components here
```

### Node Components
- **Publishers**: Send messages to topics
- **Subscribers**: Receive messages from topics
- **Services**: Provide request/response functionality
- **Clients**: Make service requests
- **Timers**: Execute callbacks at regular intervals
- **Parameters**: Configuration values that can be changed at runtime
- **Actions**: Long-running tasks with feedback

## Topics

### Definition
Topics enable publish/subscribe communication between nodes. This is an asynchronous communication pattern where messages flow from publishers to subscribers.

### Key Characteristics
- Topics use a publish/subscribe pattern
- Multiple publishers can send to the same topic
- Multiple subscribers can receive from the same topic
- Communication is asynchronous
- Quality of Service (QoS) settings control communication behavior

### Topic Communication Flow
1. Publisher sends messages to a named topic
2. The ROS 2 middleware routes messages to all subscribers
3. Subscribers receive messages from the topic

### Quality of Service (QoS) Settings
- **Reliability**: Reliable (all messages delivered) or Best Effort (some messages may be dropped)
- **Durability**: Volatile (only new messages) or Transient Local (historical messages available)
- **History**: Keep Last N messages or Keep All messages
- **Depth**: Size of the message queue

## Services

### Definition
Services enable request/response communication between nodes. This is a synchronous communication pattern where one node requests data or action from another.

### Key Characteristics
- Services use a request/response pattern
- One service server handles requests
- Multiple service clients can make requests
- Communication is synchronous
- Request and response types are defined in service definitions

### Service Communication Flow
1. Service client sends a request to the service
2. Service server processes the request
3. Service server sends response back to the client

### Service vs. Topic
- **Services**: Synchronous, request/response, one-to-one
- **Topics**: Asynchronous, publish/subscribe, one-to-many or many-to-many

## URDF (Unified Robot Description Format)

### Definition
URDF (Unified Robot Description Format) is an XML format for representing robot models and their properties. It describes the physical structure of a robot.

### Key Components
- **Links**: Rigid body elements that represent parts of the robot
- **Joints**: Connections between links that define how they move relative to each other
- **Visual**: How the robot appears visually
- **Collision**: How the robot interacts with its environment physically
- **Inertial**: Mass and inertial properties for physics simulation
- **Materials**: Visual appearance properties
- **Gazebo Extensions**: Simulation-specific properties

### URDF Example Structure
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Sensor link -->
  <link name="sensor_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting base and sensor -->
  <joint name="sensor_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_link"/>
    <origin xyz="0.2 0 0.15"/>
  </joint>
</robot>
```

### Joint Types
- **Revolute**: Rotational joint with limits
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint with limits
- **Fixed**: No movement, rigid connection
- **Floating**: 6 degrees of freedom
- **Planar**: Movement in a plane

## Actions

### Definition
Actions are a communication pattern for long-running tasks that provide feedback and can be canceled. They're built on top of services and topics.

### Key Components
- **Goal**: Request to start an action
- **Feedback**: Regular updates during execution
- **Result**: Final outcome when action completes

### Action vs. Service vs. Topic
- **Topics**: Asynchronous data streaming
- **Services**: Synchronous request/response
- **Actions**: Asynchronous long-running tasks with feedback

## ROS 2 Graph Architecture

### Overview
The ROS 2 graph represents the network of nodes, topics, services, and other communication entities. Unlike ROS 1, ROS 2 uses a distributed architecture without a central master.

### Components
- **Nodes**: Individual processes performing computation
- **Topics**: Named buses for publish/subscribe communication
- **Services**: Named endpoints for request/response communication
- **Actions**: Named endpoints for long-running tasks
- **Parameters**: Configuration values shared between nodes

### Communication Patterns
- **Peer-to-peer**: Nodes communicate directly through DDS
- **Distributed**: No single point of failure
- **Discovery**: Nodes automatically discover each other
- **QoS**: Quality of Service settings for different communication needs