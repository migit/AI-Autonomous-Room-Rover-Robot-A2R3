import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import time

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.range_sub = self.create_subscription(
            Range, '/range', self.range_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        self.state = 'FORWARD'
        self.backup_start_time = 0
        self.turn_start_time = 0
        self.last_distance = 2000  # mm
        self.get_logger().info('Obstacle avoidance node started')

    def range_callback(self, msg):
        self.last_distance = msg.range * 1000  # Convert to mm

    def timer_callback(self):
        distance = self.last_distance
        self.get_logger().info(f'Range: {distance:.1f} mm, State: {self.state}')
        twist = Twist()

        if self.state == 'FORWARD':
            if distance < 550:  # 550 mm threshold
                self.state = 'STOP'
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Obstacle detected, stopping')
            else:
                twist.linear.x = 0.3  # Faster forward speed
                twist.angular.z = 0.0
        elif self.state == 'STOP':
            self.state = 'BACKUP'
            self.backup_start_time = time.time()
            twist.linear.x = -0.3  # Faster backward speed
            twist.angular.z = 0.0
            self.get_logger().info('Backing up')
        elif self.state == 'BACKUP':
            if time.time() - self.backup_start_time >= 0.2:  # 200 ms backup
                self.state = 'TURN'
                self.turn_start_time = time.time()
                twist.linear.x = 0.0
                twist.angular.z = 1.57  # 90°/s
                self.get_logger().info('Turning 90 degrees')
            else:
                twist.linear.x = -0.3
                twist.angular.z = 0.0
        elif self.state == 'TURN':
            if time.time() - self.turn_start_time >= 0.8:  # ~90° turn
                self.state = 'FORWARD'
                twist.linear.x = 0.3
                twist.angular.z = 0.0
                self.get_logger().info('Resuming forward')
            else:
                twist.linear.x = 0.0
                twist.angular.z = 1.57

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
