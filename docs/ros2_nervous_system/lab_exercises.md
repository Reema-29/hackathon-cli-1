# ROS 2 Lab Exercises

This document contains hands-on lab exercises for the ROS 2 Robotic Nervous System module. Each exercise builds on the concepts learned and provides practical experience with ROS 2.

## Lab Exercise 1: Basic Publisher/Subscriber Communication

### Objective
Create a simple publisher/subscriber system to understand the basic communication pattern in ROS 2.

### Prerequisites
- ROS 2 Humble installed
- Basic Python knowledge
- Understanding of ROS 2 nodes and topics

### Steps

#### 1. Create the Publisher Node (Talker)
1. Navigate to `src/ros2_examples/publisher_subscriber/`
2. Create a file named `talker.py`
3. Implement a node that publishes "Hello World" messages to a topic called `chatter`

#### 2. Create the Subscriber Node (Listener)
1. In the same directory, create a file named `listener.py`
2. Implement a node that subscribes to the `chatter` topic
3. Print received messages to the console

#### 3. Run the Communication
1. Open a terminal and source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Run the talker in one terminal:
   ```bash
   python3 talker.py
   ```
3. Run the listener in another terminal:
   ```bash
   python3 listener.py
   ```
4. Observe the communication between nodes

### Expected Output
- Talker should print "Publishing: 'Hello World: X'" where X is an incrementing number
- Listener should print "I heard: 'Hello World: X'" with the same message

### Questions for Understanding
1. What happens if you start the listener before the talker?
2. What happens if you start multiple listeners?
3. What happens if you stop the talker while listeners are running?

## Lab Exercise 2: Service Communication

### Objective
Create a service server and client to understand request/response communication in ROS 2.

### Steps

#### 1. Create the Service Server
1. Use the service example in `src/ros2_examples/ros2_basics/service_example.py`
2. Implement a service that adds two integers (AddTwoInts)

#### 2. Create the Service Client
1. Create a client that calls the service with different parameters
2. Print the results returned by the server

#### 3. Run the Service Communication
1. Start the service server
2. Run the client with different integer values
3. Observe the request/response pattern

### Expected Output
- Server should process requests and return results
- Client should receive and display the computed results

## Lab Exercise 3: URDF Robot Model

### Objective
Create and visualize a simple robot model using URDF.

### Steps

#### 1. Create a Simple Robot URDF
1. Navigate to `src/ros2_examples/urdf_examples/`
2. Create a file named `simple_robot.urdf`
3. Define a robot with at least 2 links connected by a joint

#### 2. Validate the URDF
1. Use ROS 2 tools to check if the URDF is valid
2. Visualize the robot model using RViz or other tools

#### 3. Modify the Robot
1. Add more links and joints to create a more complex robot
2. Add visual and collision properties
3. Test the URDF with different configurations

### Expected Output
- Valid URDF file that can be parsed by ROS 2 tools
- Robot model that can be visualized
- Understanding of link, joint, and material properties

## Lab Exercise 4: Quality of Service Settings

### Objective
Experiment with different QoS settings to understand their impact on communication.

### Background
Quality of Service (QoS) settings in ROS 2 control how messages are delivered between nodes. The main QoS settings are:
- **Reliability**: Whether messages are guaranteed to be delivered
  - RELIABLE: All messages are delivered (like TCP)
  - BEST_EFFORT: Messages may be dropped (like UDP)
- **Durability**: Whether late-joining subscribers receive historical messages
  - VOLATILE: Only new messages are received
  - TRANSIENT_LOCAL: Historical messages are available to new subscribers
- **Depth**: How many messages to keep in the queue

### Steps

#### 1. Examine Current QoS Settings
1. Look at the QoS settings in your talker.py and listener.py files
2. Note that they currently use RELIABLE reliability and VOLATILE durability
3. This ensures all messages are delivered but only to currently active subscribers

#### 2. Experiment with Different Settings
1. Change the talker to use BEST_EFFORT reliability
2. Run the talker and listener and observe any differences
3. Try TRANSIENT_LOCAL durability to see how it affects new subscribers

#### 3. Test Different Scenarios
1. Test with network interruptions (simulate by stopping/starting nodes)
2. Test with high message rates by reducing the timer period
3. Compare performance with different QoS settings

### Expected Output
- Understanding of when to use different QoS settings
- Ability to choose appropriate settings for different use cases:
  - Use RELIABLE for critical data that must not be lost
  - Use BEST_EFFORT for real-time data where old messages are not useful
  - Use TRANSIENT_LOCAL when new subscribers need to receive recent messages

## Lab Exercise 5: Node Parameters

### Objective
Learn to use parameters to configure nodes at runtime.

### Steps

#### 1. Add Parameters to a Node
1. Modify one of your existing nodes to accept parameters
2. Use `declare_parameter` and `get_parameter` methods

#### 2. Configure at Runtime
1. Launch the node with different parameter values
2. Observe how the behavior changes based on parameters

### Expected Output
- Node that can be configured with different parameters
- Understanding of parameter usage in ROS 2

## Assessment Questions

After completing these exercises, you should be able to answer:

1. Explain the difference between publish/subscribe and request/response communication patterns.
2. Describe the role of topics, services, and nodes in ROS 2.
3. What is URDF and why is it important for robotics?
4. How does the ROS 2 graph architecture differ from traditional client-server architectures?
5. What are Quality of Service (QoS) settings and when would you use different settings?

## Troubleshooting Common Issues

### Nodes Not Communicating
- Ensure both nodes are on the same ROS domain
- Check that topic names match exactly
- Verify that message types are compatible

### Service Calls Failing
- Confirm service name matches exactly
- Check that service server is running before client calls
- Verify request/response message types match

### URDF Validation Errors
- Check XML syntax is correct
- Ensure all required fields are present
- Validate that joint connections are properly defined

## Extension Activities

For advanced students:
1. Create a custom message type and use it in communication
2. Implement an action server for a long-running task
3. Create a launch file to start multiple nodes together
4. Add logging and monitoring to your nodes
5. Experiment with multi-robot communication