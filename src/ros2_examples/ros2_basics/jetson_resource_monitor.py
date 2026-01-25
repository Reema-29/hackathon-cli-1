#!/usr/bin/env python3
#
# Jetson Resource Monitor Example
# Monitors CPU, GPU, memory, and temperature on Jetson platforms
#

import rclpy
from rclpy.node import Node
import psutil
import time
import subprocess
from std_msgs.msg import String
from ros2_nervous_system_msgs.msg import SystemStatus  # Using the custom message defined in contracts


class JetsonResourceMonitor(Node):
    """
    A node that monitors resource usage on Jetson platforms.
    This demonstrates how to access system-level information on the Jetson Orin Nano.
    """

    def __init__(self):
        super().__init__('jetson_resource_monitor')

        # Create publisher for system status
        self.status_publisher = self.create_publisher(SystemStatus, 'jetson_system_status', 10)

        # Timer to periodically check resources
        self.timer = self.create_timer(2.0, self.check_resources)  # Check every 2 seconds

        self.get_logger().info('Jetson Resource Monitor node started')

    def get_jetson_temperatures(self):
        """Get temperature information from Jetson thermal zones."""
        temperatures = {}
        try:
            # Look for thermal zones
            import glob
            thermal_zones = glob.glob('/sys/class/thermal/thermal_zone*')
            for tz in thermal_zones:
                try:
                    with open(f'{tz}/type', 'r') as f:
                        type_name = f.read().strip()
                    with open(f'{tz}/temp', 'r') as f:
                        temp = float(f.read().strip()) / 1000.0  # Convert from millidegrees
                    temperatures[type_name] = temp
                except:
                    continue
        except:
            # If direct access fails, try using tegrastats (if available)
            try:
                result = subprocess.run(['tegrastats'], capture_output=True, text=True, timeout=1)
                # This is a simplified approach - in practice, you'd parse tegrastats output
                temperatures['tegrastats'] = 'available'
            except:
                temperatures['tegrastats'] = 'not_available'

        return temperatures

    def check_resources(self):
        """Check system resources and publish status."""
        # Get system information
        cpu_percent = psutil.cpu_percent(interval=None)
        memory_percent = psutil.virtual_memory().percent
        disk_percent = psutil.disk_usage('/').percent

        # Get temperatures (if available)
        temperatures = self.get_jetson_temperatures()

        # Create system status message
        status_msg = SystemStatus()
        status_msg.system_name = 'jetson_orin_nano'
        status_msg.cpu_usage = float(cpu_percent)
        status_msg.memory_usage = float(memory_percent)
        status_msg.active_nodes = 1  # This node is active
        status_msg.connected_sensors = 0  # Placeholder - would be updated based on actual sensors
        status_msg.timestamp = float(time.time())

        # Determine status based on resource usage
        if cpu_percent > 90 or memory_percent > 90:
            status_msg.status = 'degraded'
        elif cpu_percent > 95 or memory_percent > 95:
            status_msg.status = 'error'
        else:
            status_msg.status = 'operational'

        # Publish the message
        self.status_publisher.publish(status_msg)

        # Log the information
        self.get_logger().info(
            f'System Status - CPU: {cpu_percent:.1f}%, '
            f'Memory: {memory_percent:.1f}%, '
            f'Temp: {list(temperatures.values())[:3]}...'  # Show first 3 temps
        )


def main(args=None):
    rclpy.init(args=args)
    monitor = JetsonResourceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()