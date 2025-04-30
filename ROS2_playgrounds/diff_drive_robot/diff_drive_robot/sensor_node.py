import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Imu
from nav_msgs.msg import Odometry
import socket
import time
import re

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.0)  # Increased timeout
        self.connect_to_esp32()
        self.range_pub = self.create_publisher(Range, '/range', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_time = time.time()
        self.get_logger().info('Sensor node started')

    def connect_to_esp32(self):
        retries = 3
        for i in range(retries):
            try:
                if self.sock:
                    self.sock.close()
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2.0)
                self.sock.connect(('192.168.82.100', 80))
                self.get_logger().info('Connected to ESP32')
                return
            except Exception as e:
                self.get_logger().error(f'Connection failed: {str(e)}')
                time.sleep(1)
        self.get_logger().error('Failed to connect after retries')

    def timer_callback(self):
        try:
            self.sock.send("SENS\n".encode())
            response = self.sock.recv(1024).decode().strip()
            if not response:
                self.get_logger().warn('Empty or invalid response, skipping')
                return
            # Validate response format: leftPos,rightPos,distance,yaw
            if not re.match(r'^-?\d+\.\d+,-?\d+\.\d+,\d+,-?\d+\.\d+$', response):
                self.get_logger().warn(f'Invalid response format: {response}')
                return
            self.get_logger().info(f'Received response: "{response}"')
            left_pos, right_pos, distance, yaw = map(float, response.split(','))

            # Publish Range
            range_msg = Range()
            range_msg.header.stamp = self.get_current_timestamp()
            range_msg.header.frame_id = 'range_sensor'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.52  # ~30 degrees
            range_msg.min_range = 0.02
            range_msg.max_range = 4.0
            range_msg.range = distance / 1000.0  # Convert mm to m
            self.range_pub.publish(range_msg)

            # Publish Imu (simplified)
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_current_timestamp()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.angular_velocity.z = yaw  # Simplified
            self.imu_pub.publish(imu_msg)

            # Publish Odometry (simplified)
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_current_timestamp()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            odom_msg.twist.twist.linear.x = 0.0  # Placeholder
            self.odom_pub.publish(odom_msg)

        except Exception as e:
            self.get_logger().error(f'Sensor error: {str(e)}')
            self.connect_to_esp32()

    def get_current_timestamp(self):
        return self.get_clock().now().to_msg()

    def destroy_node(self):
        if self.sock:
            self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
