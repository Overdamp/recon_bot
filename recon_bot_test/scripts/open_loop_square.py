#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import os
import csv
from datetime import datetime
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import qos_profile_sensor_data

class OpenLoopSquare(Node):
    def __init__(self):
        super().__init__('open_loop_square')
        
        # --- Control Publishers ---
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- Logging Setup ---
        self.declare_parameter('log_dir', os.path.expanduser('~/recon_bot_logs'))
        self.log_dir = self.get_parameter('log_dir').value
        
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f'test_data_{timestamp}.csv')
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow([
            'timestamp', 
            'odom_x', 'odom_y', 'odom_yaw', 
            'apriltag_x', 'apriltag_y', 'apriltag_z', 
            'odom_dist_from_start', 'apriltag_dist_from_start',
            'tag_visible',
            'state' # Added state logging
        ])
        
        self.get_logger().info(f'Logging to: {self.filename}')
        
        # --- Logging State ---
        self.start_odom_pose = None
        self.start_apriltag_pose = None
        self.current_odom = None
        
        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- Subscribers ---
        self.create_subscription(Odometry, '/wheel_odom', self.odom_callback, qos_profile_sensor_data)

        # --- Control State Machine ---
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
        self.max_speed_mps = 0.25 
        self.speed_ratio = 0.5    
        self.actual_speed = self.max_speed_mps * self.speed_ratio 
        self.distance = 1.0       
        
        self.move_duration = self.distance / self.actual_speed 
        self.pause_duration = 2.0 
        
        self.last_switch_time = time.time()
        
        self.get_logger().info(f'Starting Open Loop Square Test (Side: {self.distance}m)')
        self.get_logger().info(f'Speed: {self.speed_ratio*100}% of Max ({self.actual_speed:.3f} m/s)')
        self.get_logger().info('Path: Forward -> Pause -> Left -> Pause -> Backward -> Pause -> Right -> Stop')

        # --- Main Timer (Control + Logging) ---
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        self.current_odom = msg

    def timer_callback(self):
        current_time = time.time()
        elapsed_in_state = current_time - self.last_switch_time
        
        # --- 1. Logging Logic ---
        self.perform_logging()

        # --- 2. Control Logic ---
        msg = Twist()

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

        # Pause 4
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
            # Send a few stop commands
            for _ in range(5):
                self.publisher_.publish(msg)
                time.sleep(0.1)
            raise SystemExit

        self.publisher_.publish(msg)

    def perform_logging(self):
        if self.current_odom is None:
            return

        # Extract Odom Data
        odom_x = self.current_odom.pose.pose.position.x
        odom_y = self.current_odom.pose.pose.position.y
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.start_odom_pose is None:
            self.start_odom_pose = (odom_x, odom_y)
            self.get_logger().info('Odom Start Pose Recorded')

        odom_dist = math.sqrt((odom_x - self.start_odom_pose[0])**2 + (odom_y - self.start_odom_pose[1])**2)

        # Extract AprilTag Data
        apriltag_x, apriltag_y, apriltag_z = 0.0, 0.0, 0.0
        apriltag_dist = 0.0
        tag_visible = False
        
        try:
            t = self.tf_buffer.lookup_transform(
                'tag36h11:23', 
                'base_footprint',     
                rclpy.time.Time()
            )
            
            ax = t.transform.translation.x
            ay = t.transform.translation.y
            az = t.transform.translation.z
            
            apriltag_x, apriltag_y, apriltag_z = ax, ay, az
            tag_visible = True
            
            if self.start_apriltag_pose is None:
                self.start_apriltag_pose = (ax, ay, az)
                self.get_logger().info('AprilTag Start Pose Recorded')
            
            dx = ax - self.start_apriltag_pose[0]
            dy = ay - self.start_apriltag_pose[1]
            dz = az - self.start_apriltag_pose[2]
            apriltag_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        self.csv_writer.writerow([
            time.time(),
            f'{odom_x:.4f}', f'{odom_y:.4f}', f'{odom_yaw:.4f}',
            f'{apriltag_x:.4f}', f'{apriltag_y:.4f}', f'{apriltag_z:.4f}',
            f'{odom_dist:.4f}', f'{apriltag_dist:.4f}',
            tag_visible,
            self.state
        ])
        self.csv_file.flush()

    def switch_state(self, new_state):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.state = new_state
        self.last_switch_time = time.time()
        self.get_logger().info(f'Switching to State {new_state}')

    def destroy_node(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        super().destroy_node()

def plot_graph(filename):
    try:
        import matplotlib.pyplot as plt
        import pandas as pd
        
        print(f"Plotting data from {filename}...")
        
        # Read CSV
        df = pd.read_csv(filename)
        
        # Create Plot
        plt.figure(figsize=(10, 6))
        
        # Plot Odom Path
        plt.plot(df['odom_x'].to_numpy(), df['odom_y'].to_numpy(), label='Odometry', color='blue', linestyle='--')
        plt.scatter(df['odom_x'].iloc[0], df['odom_y'].iloc[0], color='blue', marker='o', label='Odom Start')
        
        # Plot AprilTag Path (Only visible tags)
        visible_tags = df[df['tag_visible'] == 'True'] # Check string/bool format in CSV
        if visible_tags.empty:
             # Try boolean check if pandas parsed it as bool
             visible_tags = df[df['tag_visible'] == True]
             
        if not visible_tags.empty:
            plt.plot(visible_tags['apriltag_x'].to_numpy(), visible_tags['apriltag_y'].to_numpy(), label='AprilTag (Ground Truth)', color='green', marker='.', linestyle='-')
            plt.scatter(visible_tags['apriltag_x'].iloc[0], visible_tags['apriltag_y'].iloc[0], color='green', marker='x', label='Tag Start')
        else:
            print("Warning: No AprilTag detections found in log.")

        plt.title('Robot Path: Odometry vs AprilTag')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # Save plot
        plot_filename = filename.replace('.csv', '.png')
        plt.savefig(plot_filename)
        print(f"Plot saved to {plot_filename}")
        
        # Show plot
        plt.show()
        
    except ImportError as e:
        print(f"Could not plot results: {e}")
        print("Please ensure matplotlib and pandas are installed: pip install matplotlib pandas")
    except Exception as e:
        print(f"Error during plotting: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopSquare()
    
    log_file = node.filename # Save filename for plotting

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('open_loop_square').info('Done.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        # Plot results after shutdown
        if os.path.exists(log_file):
            plot_graph(log_file)

if __name__ == '__main__':
    main()
