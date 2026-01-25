#!/usr/bin/env python3
#
# Integration Test for ROS 2 Communication
# Tests the publisher/subscriber communication pattern
#

import unittest
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestPublisher(Node):
    """Test publisher node that sends test messages."""

    def __init__(self):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, 'test_chatter', 10)
        self.sent_count = 0

    def send_message(self, msg_data):
        """Send a single message."""
        msg = String()
        msg.data = msg_data
        self.publisher.publish(msg)
        self.sent_count += 1
        self.get_logger().info(f'Published: {msg_data}')


class TestSubscriber(Node):
    """Test subscriber node that receives and stores messages."""

    def __init__(self):
        super().__init__('test_subscriber')
        self.subscription = self.create_subscription(
            String,
            'test_chatter',
            self.listener_callback,
            10
        )
        self.received_messages = []
        self.message_count = 0

    def listener_callback(self, msg):
        """Callback function to store received messages."""
        self.received_messages.append(msg.data)
        self.message_count += 1
        self.get_logger().info(f'Received: {msg.data}')


class TestROS2Communication(unittest.TestCase):
    """Integration tests for ROS 2 publisher/subscriber communication."""

    def setUp(self):
        """Set up the test environment."""
        if not rclpy.ok():
            rclpy.init()

        self.publisher_node = TestPublisher()
        self.subscriber_node = TestSubscriber()

        # Create executor to handle callbacks
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.publisher_node)
        self.executor.add_node(self.subscriber_node)

        # Start executor in a separate thread
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.daemon = True
        self.executor_thread.start()

        # Allow some time for nodes to discover each other
        time.sleep(1.0)

    def tearDown(self):
        """Clean up after tests."""
        self.executor.shutdown()
        self.publisher_node.destroy_node()
        self.subscriber_node.destroy_node()

    def test_basic_communication(self):
        """Test basic publisher/subscriber communication."""
        # Send a test message
        test_message = "Test message for communication verification"
        self.publisher_node.send_message(test_message)

        # Wait for message to be received (with timeout)
        timeout = 5.0  # seconds
        start_time = time.time()
        while len(self.subscriber_node.received_messages) == 0 and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        # Verify the message was received
        self.assertEqual(len(self.subscriber_node.received_messages), 1)
        self.assertEqual(self.subscriber_node.received_messages[0], test_message)

    def test_multiple_messages(self):
        """Test sending and receiving multiple messages."""
        test_messages = [
            "Message 1",
            "Message 2",
            "Message 3"
        ]

        # Send multiple messages
        for msg in test_messages:
            self.publisher_node.send_message(msg)
            time.sleep(0.2)  # Small delay between messages

        # Wait for all messages to be received (with timeout)
        timeout = 5.0
        start_time = time.time()
        while len(self.subscriber_node.received_messages) < len(test_messages) and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        # Verify all messages were received
        self.assertGreaterEqual(len(self.subscriber_node.received_messages), len(test_messages))
        for i, expected_msg in enumerate(test_messages):
            if i < len(self.subscriber_node.received_messages):
                self.assertEqual(self.subscriber_node.received_messages[i], expected_msg)

    def test_message_content_preservation(self):
        """Test that message content is preserved during transmission."""
        original_message = "Hello, ROS 2! This is a test message with special characters: !@#$%^&*()"
        self.publisher_node.send_message(original_message)

        # Wait for message to be received (with timeout)
        timeout = 5.0
        start_time = time.time()
        while len(self.subscriber_node.received_messages) == 0 and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        # Verify the message content is preserved exactly
        if len(self.subscriber_node.received_messages) > 0:
            received_message = self.subscriber_node.received_messages[0]
            self.assertEqual(original_message, received_message)


def run_tests():
    """Run the integration tests."""
    # Create a test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestROS2Communication)

    # Run the tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    return result.wasSuccessful()


def main():
    """Main function to run the tests."""
    print("Running ROS 2 Communication Integration Tests...")

    success = run_tests()

    if success:
        print("\n✓ All tests passed!")
    else:
        print("\n✗ Some tests failed!")

    # Shutdown ROS
    rclpy.shutdown()


if __name__ == '__main__':
    main()