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

class OpenLoopSquareSLAM(Node):
    def __init__(self):
        super().__init__('open_loop_square_slam')
        
        # --- Control Publishers ---
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- Logging Setup ---
        self.declare_parameter('log_dir', os.path.expanduser('~/recon_bot_logs'))
        self.log_dir = self.get_parameter('log_dir').value
        
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f'slam_test_data_{timestamp}.csv')
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow([
            'timestamp', 
            'odom_x', 'odom_y', 'odom_yaw', 
            'slam_x', 'slam_y', 'slam_yaw', 
            'odom_dist_from_start', 'slam_dist_from_start',
            'slam_valid',
            'state'
        ])
        
        self.get_logger().info(f'Logging to: {self.filename}')
        
        # --- Logging State ---
        self.start_odom_pose = None
        self.start_slam_pose = None
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

        # Extract SLAM Pose (Map Frame)
        slam_x, slam_y, slam_yaw = 0.0, 0.0, 0.0
        slam_dist = 0.0
        slam_valid = False
        
        try:
            # Lookup transform from 'map' to 'base_footprint'
            t = self.tf_buffer.lookup_transform(
                'map', 
                'base_footprint',     
                rclpy.time.Time()
            )
            
            slam_x = t.transform.translation.x
            slam_y = t.transform.translation.y
            
            # Extract Yaw from Quaternion
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            slam_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            slam_valid = True
            
            if self.start_slam_pose is None:
                self.start_slam_pose = (slam_x, slam_y)
                self.get_logger().info('SLAM Start Pose Recorded')
                
            slam_dist = math.sqrt((slam_x - self.start_slam_pose[0])**2 + (slam_y - self.start_slam_pose[1])**2)
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        self.csv_writer.writerow([
            time.time(),
            f'{odom_x:.4f}', f'{odom_y:.4f}', f'{odom_yaw:.4f}',
            f'{slam_x:.4f}', f'{slam_y:.4f}', f'{slam_yaw:.4f}',
            f'{odom_dist:.4f}', f'{slam_dist:.4f}',
            slam_valid,
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
        
        # Normalize Odom to start at (0,0)
        odom_x = df['odom_x'].to_numpy()
        odom_y = df['odom_y'].to_numpy()
        odom_x -= odom_x[0]
        odom_y -= odom_y[0]
        
        plt.plot(odom_x, odom_y, label='Odometry (Relative)', color='blue', linestyle='--')
        plt.scatter(0, 0, color='blue', marker='o', label='Start')
        
        # Plot SLAM
        if df['slam_valid'].dtype == object:
            valid_slam = df[df['slam_valid'] == 'True']
        else:
            valid_slam = df[df['slam_valid'] == True]
            
        if not valid_slam.empty:
            slam_x = valid_slam['slam_x'].to_numpy()
            slam_y = valid_slam['slam_y'].to_numpy()
            
            # Normalize to start at (0,0)
            slam_x -= slam_x[0]
            slam_y -= slam_y[0]
            
            plt.plot(slam_x, slam_y, label='SLAM (Ground Truth)', color='green', marker='.', linestyle='-')
            plt.scatter(slam_x[0], slam_y[0], color='green', marker='x', label='SLAM Start')
        else:
            print("Warning: No valid SLAM data found.")

        plt.title('Robot Path: Odometry vs SLAM')
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
    except Exception as e:
        print(f"Error during plotting: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopSquareSLAM()
    
    log_file = node.filename # Save filename for plotting

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger('open_loop_square_slam').info('Done.')
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
