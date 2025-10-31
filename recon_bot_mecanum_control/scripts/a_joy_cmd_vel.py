#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

max_velocity = 0.1785 # max_velocity m/s 

class JoystickCommandVelocity(Node):

    def __init__(self):
        super().__init__('joy_cmd_vel')
        self.joystick_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joystick_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Initialize low-pass filter variables
        self.alpha = 0.2  # Filter coefficient (0 < alpha <= 1, lower = smoother)
        self.filtered_linear_x = 0.0
        self.filtered_linear_y = 0.0
        self.filtered_angular_z = 0.0

    def joystick_callback(self, joy_msg):
        try:
            # ตรวจสอบขนาดของ axes ก่อนใช้งาน
            if len(joy_msg.axes) < 4:
                self.get_logger().warn('Joy message axes length is less than 4')
                return

            linear_x = joy_msg.axes[3] * max_velocity  # Forward/Backward movement
            linear_y = joy_msg.axes[2] * max_velocity  # Left/Right strafing
            angular_z = joy_msg.axes[0] * max_velocity # Rotation

            # Apply low-pass filter
            self.filtered_linear_x = self.alpha * linear_x + (1.0 - self.alpha) * self.filtered_linear_x
            self.filtered_linear_y = self.alpha * linear_y + (1.0 - self.alpha) * self.filtered_linear_y
            self.filtered_angular_z = self.alpha * angular_z + (1.0 - self.alpha) * self.filtered_angular_z

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.filtered_linear_x
            cmd_vel_msg.linear.y = self.filtered_linear_y
            cmd_vel_msg.angular.z = self.filtered_angular_z
            # print(f"cmd_vel_msg: linear.x={cmd_vel_msg.linear.x}, linear.y={cmd_vel_msg.linear.y}, angular.z={cmd_vel_msg.angular.z}")
            
            self.cmd_vel_pub.publish(cmd_vel_msg)
        except Exception as e:
            self.get_logger().error(f'Error in joystick_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickCommandVelocity()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()