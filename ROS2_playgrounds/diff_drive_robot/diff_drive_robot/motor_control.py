import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(1.0)
        self.connect_to_esp32()
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.last_cmd_time = 0
        self.get_logger().info('Motor control node started')

    def connect_to_esp32(self):
        try:
            if self.sock:
                self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1.0)
            self.sock.connect(('192.168.82.100', 80))
            self.get_logger().info('Connected to ESP32')
        except Exception as e:
            self.get_logger().error(f'Connection failed: {str(e)}')

    def cmd_vel_callback(self, msg):
        try:
            current_time = time.time()
            if current_time - self.last_cmd_time < 0.2:  # Limit to 5 Hz
                return
            linear = msg.linear.x  # m/s
            angular = msg.angular.z  # rad/s
            # Deadband to ignore small inputs
            if abs(linear) < 0.01 and abs(angular) < 0.01:
                linear = 0.0
                angular = 0.0
            self.get_logger().info(f'cmd_vel: linear={linear:.3f}, angular={angular:.3f}')
            # Robot parameters: wheel base = 0.04 m, max PWM = 180
            wheel_base = 0.04
            max_pwm = 180  # Increased for faster speed
            # Convert to wheel velocities (m/s)
            v_left = linear - (angular * wheel_base / 2)
            v_right = linear + (angular * wheel_base / 2)
            # Convert to PWM (scale based on max speed, e.g., 0.75 m/s = 180 PWM)
            max_speed = 0.75  # Increased for faster speed
            left_pwm = int((v_left / max_speed) * max_pwm)
            right_pwm = int((v_right / max_speed) * max_pwm)
            left_pwm = max(min(left_pwm, max_pwm), -max_pwm)
            right_pwm = max(min(right_pwm, max_pwm), -max_pwm)
            self.get_logger().info(f'PWM: left={left_pwm}, right={right_pwm}')
            cmd = f'MOT:{left_pwm},{right_pwm}\n'
            self.sock.send(cmd.encode())
            response = self.sock.recv(1024).decode().strip()
            if response != 'OK':
                self.get_logger().warn(f'Unexpected response: {response}')
            self.last_cmd_time = current_time
        except Exception as e:
            self.get_logger().error(f'Motor error: {str(e)}')
            self.connect_to_esp32()

    def destroy_node(self):
        if self.sock:
            self.sock.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
