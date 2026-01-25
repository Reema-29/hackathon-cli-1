# Quickstart: ROS 2 Robotic Nervous System

## Overview
This quickstart guide will help you get up and running with the fundamental ROS 2 concepts including Nodes, Topics, Services, and URDF. You'll learn how to create a simple Publisher/Subscriber system and understand how these components work together to form a robot's "nervous system".

## Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Basic Python programming knowledge

## Setup ROS 2 Environment

1. **Source ROS 2 setup**:
```bash
source /opt/ros/humble/setup.bash
```

2. **Create a workspace** (if not already created):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Your First ROS 2 Publisher/Subscriber

### 1. Create the Publisher (Talker)

Create a file `talker.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass

    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Create the Subscriber (Listener)

Create a file `listener.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Run the Example

1. **Open a terminal and start the talker**:
```bash
python3 talker.py
```

2. **Open another terminal and start the listener**:
```bash
python3 listener.py
```

3. **Observe the communication**: You should see messages being published and received between the nodes.

## Understanding the Components

### Nodes
- The `TalkerNode` and `ListenerNode` are both ROS 2 nodes
- Each runs as a separate process
- Nodes are created using `rclpy.create_node()`

### Topics
- The `chatter` topic is the communication channel
- Publisher sends messages to the topic
- Subscriber receives messages from the topic
- Multiple nodes can publish/subscribe to the same topic

### Messages
- `String` is the message type being used
- Messages are passed asynchronously between nodes
- The message content is the "Hello World" string with a counter

## Creating a Simple Service

### Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()

    try:
        rclpy.spin(add_two_ints_server)
    except KeyboardInterrupt:
        pass

    add_two_ints_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    client = AddTwoIntsClient()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    if response is not None:
        print(f'Result of add_two_ints: {response.sum}')
    else:
        print('Service call failed')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Simple URDF Example

Create a file `simple_robot.urdf`:

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

## Jetson Orin Nano Specific Setup

When running on the Jetson Orin Nano:

1. **Ensure proper power mode**:
```bash
sudo nvpmodel -m 0  # Maximum performance mode
sudo jetson_clocks  # Lock clocks to maximum frequency
```

2. **Verify ROS 2 installation**:
```bash
echo $ROS_DISTRO  # Should show 'humble'
```

3. **Consider computational constraints** when designing nodes for the embedded platform.

## Next Steps

1. Explore the lab exercises in `lab_exercises.md`
2. Review the detailed concepts in `concepts.md`
3. Try creating your own custom message types
4. Experiment with different Quality of Service (QoS) settings
5. Build more complex URDF models for different robot types