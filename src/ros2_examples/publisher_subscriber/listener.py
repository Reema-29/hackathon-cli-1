#!/usr/bin/env python3
#
# Subscriber Node Example (Listener)
# This node subscribes to the 'chatter' topic and prints received messages
#

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """
    A subscriber node that listens to messages on the 'chatter' topic.
    Demonstrates the basic pattern of ROS 2 subscriber nodes.
    """

    def __init__(self):
        # Initialize the node with the name 'listener'
        super().__init__('listener')

        # Import QoS modules
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        # Create a QoS profile for the subscriber
        # Using reliable delivery to match the publisher's settings
        qos_profile = QoSProfile(
            depth=10,  # Number of messages to keep in history
            reliability=ReliabilityPolicy.RELIABLE,  # Ensure all messages are delivered
            durability=DurabilityPolicy.VOLATILE,  # Only receive from active publishers
        )

        # Create a subscription to String messages on the 'chatter' topic with QoS
        # The callback function will be called when a message is received
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos_profile
        )

        # Prevent unused variable warning
        self.subscription  # prevent unused variable warning

        # Log that the node has started
        self.get_logger().info('Listener node started, subscribing to /chatter topic with QoS settings')

    def listener_callback(self, msg):
        """
        Callback function that is called when a message is received on the 'chatter' topic.
        Prints the received message to the console.
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize and run the listener node
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the listener node
    listener = ListenerNode()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(listener)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up when shutting down
        listener.get_logger().info('Shutting down Listener node')
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()