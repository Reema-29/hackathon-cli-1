#!/usr/bin/env python3
#
# Main launcher script for ROS 2 Examples
# Provides an easy way to run all examples from a single entry point
#

import sys
import os
import subprocess
import argparse
from pathlib import Path


def run_command(cmd, description):
    """Run a command and display its output."""
    print(f"\n--- {description} ---")
    print(f"Running: {' '.join(cmd)}")

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if result.stdout:
            print("STDOUT:")
            print(result.stdout)
        if result.stderr:
            print("STDERR:")
            print(result.stderr)
        return result.returncode == 0
    except Exception as e:
        print(f"Error running command: {e}")
        return False


def list_examples():
    """List all available examples."""
    examples = [
        "publisher_subscriber.talker - Publishes 'Hello World' messages",
        "publisher_subscriber.listener - Subscribes to and prints messages",
        "ros2_basics.service_server - Provides AddTwoInts service",
        "ros2_basics.service_client - Calls AddTwoInts service",
        "help - Show this help message"
    ]

    print("Available examples:")
    for example in examples:
        print(f"  - {example}")


def run_publisher():
    """Run the publisher example."""
    script_path = Path(__file__).parent / "publisher_subscriber" / "talker.py"
    if script_path.exists():
        os.execv(sys.executable, [sys.executable, str(script_path)] + sys.argv[2:])
    else:
        print(f"Error: {script_path} not found")
        return False


def run_subscriber():
    """Run the subscriber example."""
    script_path = Path(__file__).parent / "publisher_subscriber" / "listener.py"
    if script_path.exists():
        os.execv(sys.executable, [sys.executable, str(script_path)] + sys.argv[2:])
    else:
        print(f"Error: {script_path} not found")
        return False


def run_service_server():
    """Run the service server example."""
    script_path = Path(__file__).parent / "ros2_basics" / "service_example.py"
    if script_path.exists():
        os.execv(sys.executable, [sys.executable, str(script_path)] + sys.argv[2:])
    else:
        print(f"Error: {script_path} not found")
        return False


def run_service_client():
    """Run the service client example."""
    script_path = Path(__file__).parent / "ros2_basics" / "service_example.py"
    if script_path.exists():
        # Add --client flag to run as client
        os.execv(sys.executable, [sys.executable, str(script_path), "--client"] + sys.argv[2:])
    else:
        print(f"Error: {script_path} not found")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="ROS 2 Examples Launcher - Run different ROS 2 examples easily",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python launch_all_examples.py publisher        # Run publisher example
  python launch_all_examples.py subscriber      # Run subscriber example
  python launch_all_examples.py service-server  # Run service server
  python launch_all_examples.py service-client 5 3  # Run service client with args
  python launch_all_examples.py list            # List all available examples
        """
    )

    parser.add_argument(
        'example',
        nargs='?',
        choices=[
            'publisher', 'talker',
            'subscriber', 'listener',
            'service-server', 'service_client',
            'service-client', 'list', 'help'
        ],
        default='help',
        help='Choose which example to run'
    )

    parser.add_argument(
        'args',
        nargs=argparse.REMAINDER,
        help='Additional arguments to pass to the example'
    )

    args = parser.parse_args()

    if args.example in ['help', None]:
        parser.print_help()
        list_examples()
        return 0

    if args.example in ['list']:
        list_examples()
        return 0

    if args.example in ['publisher', 'talker']:
        print("Starting Publisher Example (Talker)...")
        print("Run this in one terminal, then run subscriber in another terminal")
        return run_publisher()

    elif args.example in ['subscriber', 'listener']:
        print("Starting Subscriber Example (Listener)...")
        print("Run this in one terminal, then run publisher in another terminal")
        return run_subscriber()

    elif args.example == 'service-server':
        print("Starting Service Server Example...")
        print("Run this in one terminal, then run service-client in another terminal")
        return run_service_server()

    elif args.example in ['service-client', 'service_client']:
        print("Starting Service Client Example...")
        print("Make sure service server is running first")
        if len(args.args) < 2:
            print("Usage: python launch_all_examples.py service-client <num1> <num2>")
            print("Example: python launch_all_examples.py service-client 5 3")
            return 1
        return run_service_client()

    else:
        print(f"Unknown example: {args.example}")
        list_examples()
        return 1


if __name__ == '__main__':
    sys.exit(main())