#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class OpenLoopSquare(Node):
    def __init__(self):
        super().__init__('open_loop_square')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.state = 0  # 0:Forward, 1:Left, 2:Backward, 3:Right, 4:Stop
        self.start_time = time.time()
        
        # Configuration
        self.speed = 0.2        # m/s
        self.distance = 2.0     # meters
        self.duration = self.distance / self.speed # 5.0 seconds
        self.pause_duration = 2.0 # seconds between moves
        
        self.is_moving = True
        self.last_switch_time = time.time()
        
        self.get_logger().info(f'Starting Open Loop Square Test (Side: {self.distance}m, Speed: {self.speed}m/s)')

    def timer_callback(self):
        current_time = time.time()
        elapsed_in_state = current_time - self.last_switch_time
        
        msg = Twist()

        if self.state == 0: # Forward (+X)
            if elapsed_in_state < self.duration:
                msg.linear.x = self.speed
                self.get_logger().info_once('Moving Forward...')
            else:
                self.switch_state(1)
                
        elif self.state == 1: # Left (+Y)
            if elapsed_in_state < self.duration:
                msg.linear.y = self.speed
                self.get_logger().info_once('Moving Left (Strafing)...')
            else:
                self.switch_state(2)
                
        elif self.state == 2: # Backward (-X)
            if elapsed_in_state < self.duration:
                msg.linear.x = -self.speed
                self.get_logger().info_once('Moving Backward...')
            else:
                self.switch_state(3)
                
        elif self.state == 3: # Right (-Y)
            if elapsed_in_state < self.duration:
                msg.linear.y = -self.speed
                self.get_logger().info_once('Moving Right (Strafing)...')
            else:
                self.switch_state(4)
                
        elif self.state == 4: # Stop
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Test Complete. Stopping.')
            self.publisher_.publish(msg)
            raise SystemExit

        # Pause logic (optional, simplified here to continuous move for smoother square, 
        # but for precise measurement, pauses are better. Let's add a small pause state if needed, 
        # but for now continuous is fine for "Square Path")
        
        self.publisher_.publish(msg)

    def switch_state(self, new_state):
        # Stop briefly to reset inertia (optional but good for mecanum)
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        time.sleep(1.0) # Pause for 1 second
        
        self.state = new_state
        self.last_switch_time = time.time()
        self.get_logger().info(f'Switching to State {new_state}')

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopSquare()

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('open_loop_square').info('Done.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
