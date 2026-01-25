#!/bin/bash

# Jetson-specific launch script for ROS 2 Examples
# Sets up Jetson-specific configurations before running ROS 2 nodes

echo "Starting ROS 2 Examples on Jetson Orin Nano..."

# Set Jetson to maximum performance mode (requires sudo)
echo "Setting Jetson to maximum performance mode..."
sudo nvpmodel -m 0  # 0 is MAXN mode, MAXQ is 2
sudo jetson_clocks  # Lock clocks to maximum frequency

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Set additional Jetson-specific environment variables
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp  # Use CycloneDDS which works well on embedded systems
export ROS_DOMAIN_ID=20  # Use a specific domain ID to avoid interference

echo "ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION set to $RMW_IMPLEMENTATION"

# Check if additional arguments were provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <example_name> [args]"
    echo "Available examples:"
    echo "  publisher     - Run publisher example"
    echo "  subscriber  - Run subscriber example"
    echo "  service       - Run service example"
    echo "  monitor       - Run Jetson resource monitor"
    exit 1
fi

EXAMPLE=$1
shift  # Remove the first argument

case $EXAMPLE in
    "publisher")
        echo "Starting Publisher Example..."
        python3 src/ros2_examples/publisher_subscriber/talker.py "$@"
        ;;
    "subscriber")
        echo "Starting Subscriber Example..."
        python3 src/ros2_examples/publisher_subscriber/listener.py "$@"
        ;;
    "service")
        echo "Starting Service Example..."
        python3 src/ros2_examples/ros2_basics/service_example.py "$@"
        ;;
    "monitor")
        echo "Starting Jetson Resource Monitor..."
        python3 src/ros2_examples/ros2_basics/jetson_resource_monitor.py "$@"
        ;;
    *)
        echo "Unknown example: $EXAMPLE"
        echo "Available examples: publisher, subscriber, service, monitor"
        exit 1
        ;;
esac

# Optionally return Jetson to default power mode when done (commented out to keep performance)
# echo "Returning Jetson to default power mode..."
# sudo nvpmodel -m 2  # Return to default mode