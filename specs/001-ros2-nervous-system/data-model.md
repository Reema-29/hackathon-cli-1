# Data Model: ROS 2 Robotic Nervous System

## Key Entities

### ROS 2 Node
**Description**: A process that performs computation and communicates with other nodes
**Attributes**:
- `node_name`: String identifier for the node
- `namespace`: Optional namespace for organizing nodes
- `parameters`: Configuration values for the node
- `publishers`: List of publishers the node owns
- `subscribers`: List of subscribers the node owns
- `services`: List of services the node provides
- `clients`: List of service clients the node uses

**Relationships**:
- One-to-many with Publishers (one node can have multiple publishers)
- One-to-many with Subscribers (one node can have multiple subscribers)
- One-to-many with Services (one node can provide multiple services)
- One-to-many with Clients (one node can use multiple service clients)

### Topic
**Description**: A named bus over which nodes exchange messages in a publish/subscribe pattern
**Attributes**:
- `topic_name`: String identifier for the topic
- `message_type`: Type of messages published to the topic
- `qos_profile`: Quality of Service settings
- `publishers`: List of nodes publishing to this topic
- `subscribers`: List of nodes subscribed to this topic

**Relationships**:
- Many-to-many with Nodes (via publishers and subscribers)

### Message
**Description**: Data structure passed between nodes via topics
**Attributes**:
- `type`: Message type definition (e.g., std_msgs/String)
- `timestamp`: When the message was created
- `data`: The actual message content (structure varies by type)

### Service
**Description**: A request/response communication pattern between nodes
**Attributes**:
- `service_name`: String identifier for the service
- `service_type`: Type definition for the service
- `qos_profile`: Quality of Service settings
- `server`: The node providing the service
- `clients`: List of nodes using the service

**Relationships**:
- One-to-one with Service Server (one service per server node)
- One-to-many with Service Clients (one service can have multiple clients)

### URDF Robot Model
**Description**: XML representation of a robot's structure and properties
**Attributes**:
- `robot_name`: Name of the robot
- `links`: List of rigid body components
- `joints`: List of connections between links
- `materials`: Visual materials used in the model
- `gazebo_extensions`: Simulation-specific extensions

**Relationships**:
- One-to-many with Links (one robot has multiple links)
- One-to-many with Joints (one robot has multiple joints)

### Link
**Description**: A rigid body element in a URDF robot model
**Attributes**:
- `link_name`: Name of the link
- `visual`: Visual representation properties
- `collision`: Collision detection properties
- `inertial`: Mass and inertial properties

**Relationships**:
- Many-to-one with URDF Robot Model (many links belong to one robot)

### Joint
**Description**: Connection between two links in a URDF robot model
**Attributes**:
- `joint_name`: Name of the joint
- `joint_type`: Type of joint (revolute, continuous, prismatic, etc.)
- `parent_link`: Link that is the parent in the kinematic chain
- `child_link`: Link that is the child in the kinematic chain
- `origin`: Position and orientation of the joint relative to parent

**Relationships**:
- Many-to-one with URDF Robot Model (many joints belong to one robot)
- One-to-one with Parent Link
- One-to-one with Child Link

## State Transitions

### Node Lifecycle
- `UNCONFIGURED` → `INACTIVE` (configure)
- `INACTIVE` → `ACTIVE` (activate)
- `ACTIVE` → `INACTIVE` (deactivate)
- `INACTIVE` → `UNCONFIGURED` (cleanup)
- Any state → `FINALIZED` (shutdown)

### Communication States
- `NOT_CONNECTED` → `CONNECTED` (when nodes discover each other)
- `CONNECTED` → `DISCONNECTED` (when nodes shut down)

## Validation Rules

1. **Node Naming**: Node names must be unique within a ROS domain
2. **Topic Naming**: Topic names must follow ROS naming conventions (start with / or be relative)
3. **Message Compatibility**: Publishers and subscribers on the same topic must use compatible message types
4. **URDF Validity**: URDF files must be well-formed XML and follow URDF schema
5. **Joint Limits**: Joint limits must be physically realistic and within actuator capabilities
6. **Namespace Scoping**: Namespaced nodes must properly resolve to their full names

## Relationships Summary

```
[ROS 2 Node] 1 ---- * [Publisher]
[ROS 2 Node] 1 ---- * [Subscriber]
[ROS 2 Node] 1 ---- * [Service Server]
[ROS 2 Node] 1 ---- * [Service Client]

[Topic] * ---- * [Node] (via publishers/subscribers)
[Service] 1 ---- * [Service Client]

[URDF Robot Model] 1 ---- * [Link]
[URDF Robot Model] 1 ---- * [Joint]
[Joint] 1 ---- 1 [Parent Link]
[Joint] 1 ---- 1 [Child Link]
```