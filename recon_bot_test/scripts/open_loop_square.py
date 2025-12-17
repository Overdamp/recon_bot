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
        
        # State definitions
        # 0: Forward (+X)
        # 1: Pause 1
        # 2: Left (+Y)
        # 3: Pause 2
        # 4: Backward (-X)
        # 5: Pause 3
        # 6: Right (-Y)
        # 7: Pause 4
        # 8: Stop
        self.state = 0
        self.start_time = time.time()
        
        # Configuration
        self.max_speed_mps = 0.25 # Must match max_linear_speed in mecanum_controller.py
        self.speed_ratio = 0.5    # 50% of max speed
        
        self.actual_speed = self.max_speed_mps * self.speed_ratio # 0.125 m/s
        self.distance = 1.0       # meters
        
        # Duration = Distance / Actual Speed
        self.move_duration = self.distance / self.actual_speed 
        self.pause_duration = 2.0 # seconds
        
        self.last_switch_time = time.time()
        
        self.get_logger().info(f'Starting Open Loop Square Test (Side: {self.distance}m)')
        self.get_logger().info(f'Speed: {self.speed_ratio*100}% of Max ({self.actual_speed:.3f} m/s)')
        self.get_logger().info('Path: Forward -> Pause -> Left -> Pause -> Backward -> Pause -> Right -> Stop')

    def timer_callback(self):
        current_time = time.time()
        elapsed_in_state = current_time - self.last_switch_time
        
        msg = Twist()

        # --- State Machine ---
        
        # Move 1: Forward (+X)
        if self.state == 0:
            if elapsed_in_state < self.move_duration:
                msg.linear.x = self.speed_ratio
                self.get_logger().info('State 0: Moving Forward (+X)...', throttle_duration_sec=1.0)
            else:
                self.switch_state(1)

        # Pause 1
        elif self.state == 1:
            if elapsed_in_state < self.pause_duration:
                # Stop
                pass 
            else:
                self.switch_state(2)

        # Move 2: Left (+Y)
        elif self.state == 2:
            if elapsed_in_state < self.move_duration:
                msg.linear.y = self.speed_ratio
                self.get_logger().info('State 2: Moving Left (+Y)...', throttle_duration_sec=1.0)
            else:
                self.switch_state(3)

        # Pause 2
        elif self.state == 3:
            if elapsed_in_state < self.pause_duration:
                pass
            else:
                self.switch_state(4)

        # Move 3: Backward (-X)
        elif self.state == 4:
            if elapsed_in_state < self.move_duration:
                msg.linear.x = -self.speed_ratio
                self.get_logger().info('State 4: Moving Backward (-X)...', throttle_duration_sec=1.0)
            else:
                self.switch_state(5)

        # Pause 3
        elif self.state == 5:
            if elapsed_in_state < self.pause_duration:
                pass
            else:
                self.switch_state(6)

        # Move 4: Right (-Y)
        elif self.state == 6:
            if elapsed_in_state < self.move_duration:
                msg.linear.y = -self.speed_ratio
                self.get_logger().info('State 6: Moving Right (-Y)...', throttle_duration_sec=1.0)
            else:
                self.switch_state(7)

        # Pause 4 (Final pause before finish)
        elif self.state == 7:
            if elapsed_in_state < self.pause_duration:
                pass
            else:
                self.switch_state(8)

        # Stop
        elif self.state == 8:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Test Complete. Stopping.')
            # Send a few stop commands to be sure
            for _ in range(5):
                self.publisher_.publish(msg)
                time.sleep(0.1)
            raise SystemExit

        self.publisher_.publish(msg)

    def switch_state(self, new_state):
        # Force a stop command when switching states to prevent drift/inertia carry-over
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        
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
        # Ensure robot stops on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
