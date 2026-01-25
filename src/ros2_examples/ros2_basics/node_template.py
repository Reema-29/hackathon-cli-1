#!/usr/bin/env python3
#
# Basic ROS 2 Node Template
# This template provides the foundation for creating new ROS 2 nodes in Python
#

import rclpy
from rclpy.node import Node


class BasicNodeTemplate(Node):
    """
    A basic ROS 2 node template that can be used as a starting point for new nodes.
    This template includes the standard structure and common patterns for ROS 2 nodes.
    """

    def __init__(self):
        # Initialize the node with a name
        super().__init__('basic_node_template')

        # Example parameter declaration (optional)
        self.declare_parameter('example_param', 'default_value')
        self.example_param = self.get_parameter('example_param').value

        # Log that the node has started
        self.get_logger().info(f'BasicNodeTemplate node started with parameter: {self.example_param}')

        # Initialize any required components here
        # - Publishers
        # - Subscribers
        # - Services
        # - Timers
        # - etc.

    def example_method(self):
        """
        Example method that can be called by the node's functionality
        """
        self.get_logger().info('Example method called')


def main(args=None):
    """
    Main function to initialize and run the node
    """
    rclpy.init(args=args)

    # Create the node instance
    basic_node = BasicNodeTemplate()

    try:
        # Spin the node to process callbacks
        rclpy.spin(basic_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up when shutting down
        basic_node.get_logger().info('Shutting down BasicNodeTemplate node')
        basic_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()