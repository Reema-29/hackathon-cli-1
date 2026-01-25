# Publisher/Subscriber Examples

This directory contains the basic publisher/subscriber examples that demonstrate the fundamental communication pattern in ROS 2.

## Overview

The publisher/subscriber pattern is the most common communication method in ROS 2. It uses an asynchronous, one-way transmission of data:
- **Publisher**: Sends messages to a topic
- **Subscriber**: Receives messages from a topic
- **Topic**: Named bus over which messages are sent

## Files

- `talker.py`: Publisher node that sends "Hello World" messages
- `listener.py`: Subscriber node that receives and prints messages
- `README.md`: This file

## How to Run

### 1. Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

### 2. Run the Publisher (Talker)
In one terminal:
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/ros2_examples/publisher_subscriber/talker.py
```

### 3. Run the Subscriber (Listener)
In another terminal:
```bash
cd ~/ros2_ws
source install/setup.bash
python3 src/ros2_examples/publisher_subscriber/listener.py
```

### 4. Observe Communication
- The talker will publish messages like: `Publishing: "Hello World: X"`
- The listener will receive and display messages like: `I heard: "Hello World: X"`

## Understanding the Code

### Talker (Publisher)
1. Creates a publisher for String messages on the 'chatter' topic
2. Uses a timer to publish messages at regular intervals (0.5 seconds)
3. Increments a counter in each message to show sequence

### Listener (Subscriber)
1. Creates a subscription to the 'chatter' topic
2. Defines a callback function that executes when a message is received
3. Prints the received message to the console

## Key Concepts Demonstrated

- **Node Creation**: How to create ROS 2 nodes in Python
- **Publisher**: How to create and use publishers
- **Subscriber**: How to create and use subscribers
- **Message Types**: Using standard message types (std_msgs/String)
- **Logging**: Using ROS 2 logging functionality
- **Spin**: How to keep the node running and processing callbacks

## Quality of Service (QoS)

The publisher and subscriber use the default QoS profile with a depth of 10, which means:
- Up to 10 messages can be queued if the subscriber is not ready to process them
- Messages are delivered on a best-effort basis

## Troubleshooting

### Common Issues

1. **Nodes don't see each other**: Make sure both terminals have sourced the ROS 2 environment
2. **Wrong topic names**: Verify both nodes are using the same topic name ('chatter')
3. **Permission errors**: Ensure you have proper permissions for network communication

### Verifying Communication

You can verify the communication using ROS 2 command-line tools:
```bash
# List active topics
ros2 topic list

# Echo messages on the chatter topic
ros2 topic echo /chatter std_msgs/msg/String

# Show topic information
ros2 topic info /chatter
```

## Next Steps

After understanding this basic example, try:
1. Modifying the message content
2. Changing the publishing rate
3. Adding multiple subscribers
4. Creating custom message types
5. Experimenting with different QoS settings