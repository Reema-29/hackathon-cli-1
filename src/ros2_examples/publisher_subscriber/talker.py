#!/usr/bin/env python3
#
# Publisher Node Example (Talker)
# This node publishes "Hello World" messages to a topic called 'chatter'
#

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """
    A publisher node that sends messages to the 'chatter' topic.
    Demonstrates the basic pattern of ROS 2 publisher nodes.
    """

    def __init__(self):
        # Initialize the node with the name 'talker'
        super().__init__('talker')

        # Import QoS modules
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        # Create a QoS profile for the publisher
        # Using reliable delivery to ensure all messages are received
        qos_profile = QoSProfile(
            depth=10,  # Number of messages to keep in history
            reliability=ReliabilityPolicy.RELIABLE,  # Ensure all messages are delivered
            durability=DurabilityPolicy.VOLATILE,  # Only deliver to active subscribers
        )

        # Create a publisher for String messages on the 'chatter' topic with QoS
        self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)

        # Set timer period to 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for the messages
        self.i = 0

        # Log that the node has started
        self.get_logger().info('Talker node started, publishing to /chatter topic with QoS settings')

    def timer_callback(self):
        """
        Callback function that is called every timer period.
        Creates and publishes a message to the 'chatter' topic.
        """
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to initialize and run the talker node
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the talker node
    talker = TalkerNode()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(talker)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up when shutting down
        talker.get_logger().info('Shutting down Talker node')
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()