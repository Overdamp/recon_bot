#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MecanumControlNode(Node):
    def __init__(self):
        super().__init__('mecanum_control_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = 'forward'
        self.state_start_time = time.time()
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        self.duration = 2.0  # seconds

    def timer_callback(self):
        msg = Twist()
        current_time = time.time()

        # Change state every 2 seconds
        if current_time - self.state_start_time >= self.duration:
            if self.state == 'forward':
                self.state = 'backward'
            elif self.state == 'backward':
                self.state = 'left'
            elif self.state == 'left':
                self.state = 'right'
            elif self.state == 'right':
                self.state = 'forward'
            self.state_start_time = current_time
            self.get_logger().info(f'Moving {self.state}')

        # Set velocity based on state
        if self.state == 'forward':
            msg.linear.x = self.linear_speed
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif self.state == 'backward':
            msg.linear.x = -self.linear_speed
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif self.state == 'left':
            msg.linear.x = 0.0
            msg.linear.y = self.linear_speed
            msg.angular.z = 0.0
        elif self.state == 'right':
            msg.linear.x = 0.0
            msg.linear.y = -self.linear_speed
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()