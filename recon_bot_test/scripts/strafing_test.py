#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class StrafingTest(Node):
    def __init__(self):
        super().__init__('strafing_test')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        self.duration = 5.0  # Run for 5 seconds
        self.speed = 0.2     # 0.2 m/s

    def timer_callback(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        msg = Twist()

        if elapsed_time < self.duration:
            # Move Left (Positive Y)
            msg.linear.x = 0.0
            msg.linear.y = self.speed
            msg.angular.z = 0.0
            self.get_logger().info(f'Strafing Left: {elapsed_time:.2f}s')
        else:
            # Stop
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Test Complete. Stopping.')
            self.publisher_.publish(msg)
            # Exit after stopping
            raise SystemExit

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StrafingTest()

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('strafing_test').info('Done.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
