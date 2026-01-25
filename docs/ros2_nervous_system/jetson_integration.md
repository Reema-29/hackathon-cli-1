# Jetson Orin Nano Integration Guide

This document provides specific information about deploying and running the ROS 2 Robotic Nervous System on the NVIDIA Jetson Orin Nano platform.

## Jetson Orin Nano Specifications

### Hardware Overview
- **GPU**: NVIDIA Ampere architecture with 1024 CUDA cores
- **CPU**: ARM Cortex-A78AE 6-core (12-core) processor
- **Memory**: 32GB LPDDR5
- **Power**: 15W to 60W depending on configuration
- **Connectivity**: Gigabit Ethernet, M.2 Key E for Wi-Fi/Bluetooth
- **Expansion**: M.2 Key M for NVMe SSD, GPIO headers

### ROS 2 Compatibility
- **OS**: Ubuntu 22.04 LTS with JetPack 5.x
- **ROS 2 Distribution**: ROS 2 Humble Hawksbill (LTS)
- **Architecture**: ARM64 (aarch64)

## Setup Instructions

### 1. JetPack Installation
1. Download JetPack SDK from NVIDIA Developer website
2. Flash the Jetson Orin Nano with JetPack 5.x
3. Complete initial setup and network configuration
4. Update the system:
   ```bash
   sudo apt update && sudo apt upgrade
   ```

### 2. ROS 2 Humble Installation
1. Add ROS 2 repository:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros- humble.repos | sudo tee /etc/ros/ros.repos
   ```
2. Install ROS 2 packages:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop
   ```

### 3. Environment Setup
1. Source ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Add to your `.bashrc` for persistent setup:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   ```

## Performance Considerations

### Power Management
The Jetson Orin Nano has different power modes that affect performance:

1. **Check current power mode**:
   ```bash
   sudo nvpmodel -q
   ```

2. **Set to maximum performance**:
   ```bash
   sudo nvpmodel -m 0
   sudo jetson_clocks
   ```

### Memory Management
- Monitor memory usage with `tegrastats`
- Use appropriate buffer sizes in ROS 2 nodes to avoid memory pressure
- Consider using swap space for memory-intensive operations

### Thermal Management
- Monitor temperature with `tegrastats`
- Ensure adequate cooling for sustained performance
- Implement thermal throttling awareness in your applications

## ROS 2 Specific Considerations

### 1. Node Optimization
- Use efficient data structures and algorithms
- Minimize memory allocations in critical loops
- Consider using ROS 2 intra-process communication for performance

### 2. Jetson-Specific ROS 2 Package Setup
For ROS 2 development on Jetson Orin Nano, you may want to install additional packages:

```bash
# Install perception packages for sensor processing
sudo apt install ros-humble-perception

# Install navigation packages
sudo apt install ros-humble-navigation2

# Install simulation tools
sudo apt install ros-humble-gazebo-ros-pkgs

# Install computer vision packages
sudo apt install ros-humble-vision-opencv
```

### 3. Resource Management
- Monitor CPU and GPU usage with `tegrastats`
- Set appropriate process priorities for real-time performance
- Use `cgroups` to limit resource usage of individual nodes if needed

### 4. QoS Settings for Edge Computing
For Jetson Orin Nano applications, consider these QoS settings:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For real-time sensor data (allows some message drops for real-time performance)
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)

# For critical control commands (ensure delivery)
control_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)
```

### 5. Computational Constraints and Optimizations
When running ROS 2 nodes on Jetson Orin Nano, consider these optimizations:

- **Threading**: Use multi-threading appropriately but don't exceed available CPU cores
- **Memory Management**: Be mindful of memory usage, especially with large data like images
- **GPU Acceleration**: Leverage Jetson's GPU for compute-intensive tasks like image processing
- **Power Management**: Monitor power consumption and thermal conditions

### 6. Resource Monitoring
Create a monitoring node to track system resources:

```python
import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class JetsonMonitor(Node):
    def __init__(self):
        super().__init__('jetson_monitor')
        self.monitor_publisher = self.create_publisher(String, 'jetson_status', 10)
        self.timer = self.create_timer(1.0, self.monitor_callback)

    def monitor_callback(self):
        status = {
            'cpu_percent': psutil.cpu_percent(interval=1),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_percent': psutil.disk_usage('/').percent,
            'temperature': self.get_temperature()
        }

        msg = String()
        msg.data = json.dumps(status)
        self.monitor_publisher.publish(msg)

    def get_temperature(self):
        # Implementation for getting Jetson temperature
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = int(f.read()) / 1000.0
                return temp
        except:
            return 0.0
```

## Hardware Integration

### GPIO Access
For direct hardware control, use Jetson.GPIO:

```python
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')

        # Set GPIO mode
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(18, GPIO.OUT)  # Example pin

        # Create timer for hardware control
        self.timer = self.create_timer(0.1, self.hardware_callback)

    def hardware_callback(self):
        # Control hardware based on ROS 2 messages
        pass

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()
```

### Camera Integration
The Jetson Orin Nano supports multiple camera interfaces:

1. **MIPI CSI-2**: Direct camera sensor connection
2. **USB**: Standard USB cameras
3. **GMSL**: Gigabit Multimedia Serial Link for automotive applications

## Deployment Strategies

### 1. Containerized Deployment
Consider using Docker for consistent deployments:

```dockerfile
FROM nvcr.io/nvidia/ros:humble-ros-base-l4t-r35.4.1

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy ROS 2 packages
COPY src/ /ros_ws/src/
WORKDIR /ros_ws

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select your_package_name

# Source environment and run
CMD ["bash", "-c", "source install/setup.bash && ros2 run your_package_name your_node_name"]
```

### 2. Autostart Configuration
Create a systemd service for automatic startup:

```ini
[Unit]
Description=ROS 2 Nervous System
After=network.target

[Service]
Type=simple
User=nvidia
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 launch your_package launch_file.py'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

## Troubleshooting

### Common Issues

1. **Package Installation Failures**:
   - Ensure proper repository setup
   - Check architecture compatibility (ARM64)

2. **Performance Issues**:
   - Check power mode settings
   - Monitor thermal throttling
   - Optimize node implementations

3. **Hardware Interface Problems**:
   - Verify GPIO permissions
   - Check device tree configuration
   - Ensure proper hardware connections

### Debugging Tools

1. **System Monitoring**:
   ```bash
   tegrastats  # Real-time system stats
   jtop        # Jetson system monitor
   ```

2. **ROS 2 Tools**:
   ```bash
   ros2 topic list
   ros2 node list
   ros2 doctor  # Check ROS 2 installation
   ```

## Best Practices

### 1. Resource Efficiency
- Profile your nodes to identify bottlenecks
- Use appropriate QoS settings for your use case
- Implement proper error handling and recovery

### 2. Security
- Use ROS 2 security features for production deployments
- Keep the system updated
- Implement proper access controls

### 3. Maintainability
- Document hardware-specific configurations
- Use parameter files for different deployment scenarios
- Implement proper logging and monitoring

## Example: Jetson-Specific Node

Here's a complete example of a node designed for Jetson Orin Nano:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json

class JetsonSystemNode(Node):
    def __init__(self):
        super().__init__('jetson_system_monitor')

        # Publisher for system status
        self.status_publisher = self.create_publisher(String, 'jetson_system_status', 10)

        # Timer for regular status updates
        self.status_timer = self.create_timer(2.0, self.publish_system_status)

        self.get_logger().info('Jetson System Monitor Node Started')

    def publish_system_status(self):
        status = {
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'cpu_percent': psutil.cpu_percent(interval=1),
            'memory_percent': psutil.virtual_memory().percent,
            'disk_percent': psutil.disk_usage('/').percent,
            'temperature': self.get_jetson_temperature(),
            'power_mode': self.get_power_mode()
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_publisher.publish(msg)

        self.get_logger().debug(f'System status: {status}')

    def get_jetson_temperature(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = int(f.read()) / 1000.0
                return temp
        except:
            return 0.0

    def get_power_mode(self):
        try:
            import subprocess
            result = subprocess.run(['sudo', 'nvpmodel', '-q'],
                                  capture_output=True, text=True)
            return result.stdout.strip()
        except:
            return "unknown"

def main(args=None):
    rclpy.init(args=args)
    node = JetsonSystemNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This guide provides the foundation for deploying your ROS 2 nervous system on the Jetson Orin Nano platform while considering the specific constraints and capabilities of this powerful edge AI computer.