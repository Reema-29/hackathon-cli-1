#!/usr/bin/env python3
#
# Service Server and Client Example
# This example demonstrates request/response communication in ROS 2
#

import sys
import rclpy
from rclpy.node import Node

# Import the standard AddTwoInts service
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """
    A service server that provides an 'add_two_ints' service.
    This service takes two integers and returns their sum.
    """

    def __init__(self):
        # Initialize the node with the name 'add_two_ints_server'
        super().__init__('add_two_ints_server')

        # Create a service server for AddTwoInts
        # The service name is 'add_two_ints'
        # The callback function is add_two_ints_callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        # Log that the service server has started
        self.get_logger().info('AddTwoInts service server started')

    def add_two_ints_callback(self, request, response):
        """
        Callback function that is called when a service request is received.
        Takes the request (with a and b values), computes the sum, and returns it.
        Includes error handling for invalid inputs.
        """
        try:
            # Calculate the sum
            response.sum = request.a + request.b

            # Log the computation
            self.get_logger().info(
                f'Request received: {request.a} + {request.b} = {response.sum}'
            )
        except Exception as e:
            # Handle any errors in computation
            self.get_logger().error(f'Error processing request: {e}')
            response.sum = 0  # Default value on error

        # Return the response
        return response


class AddTwoIntsClient(Node):
    """
    A service client that calls the 'add_two_ints' service.
    This client sends two integers and receives their sum.
    """

    def __init__(self):
        # Initialize the node with the name 'add_two_ints_client'
        super().__init__('add_two_ints_client')

        # Create a service client for AddTwoInts
        # The service name is 'add_two_ints'
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to the service with two integers.
        Returns the response containing the sum.
        """
        # Set the request values
        self.request.a = a
        self.request.b = b

        # Call the service asynchronously
        self.future = self.cli.call_async(self.request)

        # Wait for the result
        rclpy.spin_until_future_complete(self, self.future)

        # Return the result
        return self.future.result()


def main_server(args=None):
    """
    Main function to run the service server
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the service server node
    add_two_ints_server = AddTwoIntsServer()

    try:
        # Keep the node running and processing callbacks
        rclpy.spin(add_two_ints_server)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up when shutting down
        add_two_ints_server.destroy_node()
        rclpy.shutdown()


def main_client(args=None):
    """
    Main function to run the service client
    """
    # Check command line arguments
    if len(sys.argv) != 3:
        print('Usage: ros2 run ros2_examples service_example_client <int1> <int2>')
        return

    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the service client node
    client = AddTwoIntsClient()

    # Parse command line arguments
    a = int(sys.argv[1])
    b = int(sys.argv[2])

    # Send the request and get the response
    response = client.send_request(a, b)

    if response is not None:
        print(f'Result of {a} + {b} = {response.sum}')
    else:
        print('Service call failed')

    # Clean up
    client.destroy_node()
    rclpy.shutdown()


def main(args=None):
    """
    Main function that can run either the server or client based on command line arguments
    """
    if len(sys.argv) > 1 and sys.argv[1] == '--client':
        # Run as client if --client argument is provided
        main_client(args)
    else:
        # Run as server by default
        main_server(args)


if __name__ == '__main__':
    main()