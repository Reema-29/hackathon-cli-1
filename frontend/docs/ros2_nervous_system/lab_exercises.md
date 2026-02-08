# Lab Exercises

This chapter provides hands-on exercises to reinforce the concepts covered in the ROS2 Nervous System.

## Exercise 1: Basic Node Communication

### Objective
Implement a simple publisher-subscriber pair to understand ROS2 communication.

### Steps
1. Create a new ROS2 package: `ros2 pkg create --build-type ament_python nervous_system_examples`
2. Create a publisher node that publishes sensor data
3. Create a subscriber node that processes the sensor data
4. Launch both nodes and observe communication

### Expected Outcome
Understanding of ROS2 topic communication and message passing.

## Exercise 2: Nervous System Node Implementation

### Objective
Implement a basic nervous system node with sensory and motor capabilities.

### Steps
1. Create a node that subscribes to sensor data
2. Implement processing logic to interpret sensor inputs
3. Create publisher for motor commands
4. Add timer-based heartbeat to monitor node health

### Sample Code Structure
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class NervousSystemNode(Node):
    def __init__(self):
        super().__init__('nervous_system_node')
        self.sensor_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.sensor_callback,
            10)
        self.motor_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.heartbeat)
    
    def sensor_callback(self, msg):
        # Process sensor data and generate motor commands
        pass
    
    def heartbeat(self):
        self.get_logger().info('Nervous system node active')
```

## Exercise 3: Multi-Node Coordination

### Objective
Create multiple coordinated nodes that simulate a simple nervous system.

### Steps
1. Create a sensory node that detects obstacles
2. Create a motor node that controls movement
3. Create a coordination node that manages priorities
4. Implement inter-node communication using services

### Expected Outcome
Understanding of multi-node coordination and priority management.

## Exercise 4: Jetson Deployment

### Objective
Deploy the nervous system on a Jetson platform.

### Steps
1. Cross-compile the ROS2 packages for Jetson
2. Transfer packages to Jetson device
3. Configure GPU acceleration
4. Run performance benchmarks

### Expected Outcome
Practical experience with edge AI deployment.

## Exercise 5: Adaptive Behavior

### Objective
Implement adaptive behavior that learns from environmental feedback.

### Steps
1. Implement a learning algorithm (e.g., Q-learning)
2. Create reward signals based on environmental outcomes
3. Adjust behavior based on learned patterns
4. Evaluate performance improvements

### Expected Outcome
Understanding of adaptive systems and machine learning integration.

## Exercise 6: Safety Mechanisms

### Objective
Implement safety mechanisms for reliable operation.

### Steps
1. Add watchdog timers to prevent stuck conditions
2. Implement emergency stop functionality
3. Add resource monitoring and limits
4. Test failure recovery procedures

### Expected Outcome
Knowledge of safety-critical system design.

## Exercise 7: Real-World Integration

### Objective
Integrate the nervous system with real hardware.

### Steps
1. Connect to actual sensors (LIDAR, cameras, IMU)
2. Interface with motor controllers
3. Test in real-world environments
4. Debug and optimize performance

### Expected Outcome
Experience with real-world robotics integration.

## Evaluation Criteria

Each exercise will be evaluated based on:
- Code quality and documentation
- Functional correctness
- Performance efficiency
- Safety considerations
- Innovation in implementation

## Submission Requirements

For each exercise, submit:
- Source code with comments
- Configuration files
- Performance benchmarks
- A brief report describing challenges and solutions