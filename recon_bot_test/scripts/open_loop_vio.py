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
import matplotlib.pyplot as plt
import pandas as pd

class OpenLoopVIO(Node):
    def __init__(self):
        super().__init__('open_loop_vio')
        
        # --- Control Publishers ---
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- Logging Setup ---
        self.declare_parameter('log_dir', os.path.expanduser('~/recon_bot_logs/vio'))
        self.log_dir = self.get_parameter('log_dir').value
        
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f'vio_test_{timestamp}.csv')
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow([
            'timestamp', 
            'odom_x', 'odom_y', 'odom_yaw', 
            'gt_x', 'gt_y', 'gt_yaw', # GT = Ground Truth (SLAM)
            'odom_dist', 'gt_dist',
            'gt_valid',
            'state'
        ])
        
        self.get_logger().info(f'Logging VIO Data to: {self.filename}')
        
        # --- State Variables ---
        self.start_odom_pose = None
        self.start_gt_pose = None
        
        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- Control Parameters ---
        self.state = 0
        self.max_speed = 0.25
        self.speed_ratio = 0.5
        self.actual_speed = self.max_speed * self.speed_ratio
        self.side_length = 1.0
        self.move_duration = self.side_length / self.actual_speed
        self.pause_duration = 2.0
        self.last_switch_time = time.time()
        
        self.get_logger().info('Starting VIO Test (Fused Odom vs SLAM)')
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        current_time = time.time()
        elapsed = current_time - self.last_switch_time
        
        # 1. Log Data
        self.perform_logging()
        
        # 2. Control Logic (Square Path)
        msg = Twist()
        
        if self.state == 0: # Forward
            if elapsed < self.move_duration: msg.linear.x = self.speed_ratio
            else: self.switch_state(1)
        elif self.state == 1: # Pause
            if elapsed > self.pause_duration: self.switch_state(2)
        elif self.state == 2: # Left
            if elapsed < self.move_duration: msg.linear.y = self.speed_ratio
            else: self.switch_state(3)
        elif self.state == 3: # Pause
            if elapsed > self.pause_duration: self.switch_state(4)
        elif self.state == 4: # Backward
            if elapsed < self.move_duration: msg.linear.x = -self.speed_ratio
            else: self.switch_state(5)
        elif self.state == 5: # Pause
            if elapsed > self.pause_duration: self.switch_state(6)
        elif self.state == 6: # Right
            if elapsed < self.move_duration: msg.linear.y = -self.speed_ratio
            else: self.switch_state(7)
        elif self.state == 7: # Pause
            if elapsed > self.pause_duration: self.switch_state(8)
        elif self.state == 8: # Stop
            self.stop_robot()
            raise SystemExit

        self.publisher_.publish(msg)

    def switch_state(self, new_state):
        self.stop_robot()
        self.state = new_state
        self.last_switch_time = time.time()
        self.get_logger().info(f'State -> {new_state}')

    def stop_robot(self):
        self.publisher_.publish(Twist())

    def perform_logging(self):
        try:
            # 1. Get Odom Pose (odom -> base_footprint)
            # In VIO mode, 'odom' is published by EKF (Fused)
            t_odom = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            odom_x = t_odom.transform.translation.x
            odom_y = t_odom.transform.translation.y
            odom_yaw = self.get_yaw(t_odom.transform.rotation)
            
            # 2. Get Ground Truth Pose (map -> base_footprint)
            t_gt = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            gt_x = t_gt.transform.translation.x
            gt_y = t_gt.transform.translation.y
            gt_yaw = self.get_yaw(t_gt.transform.rotation)
            gt_valid = True
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            return # Skip if TF not ready

        # Initialize Start Poses
        if self.start_odom_pose is None: self.start_odom_pose = (odom_x, odom_y)
        if self.start_gt_pose is None: self.start_gt_pose = (gt_x, gt_y)
        
        # Calculate Distances
        odom_dist = math.sqrt((odom_x - self.start_odom_pose[0])**2 + (odom_y - self.start_odom_pose[1])**2)
        gt_dist = math.sqrt((gt_x - self.start_gt_pose[0])**2 + (gt_y - self.start_gt_pose[1])**2)
        
        self.csv_writer.writerow([
            time.time(),
            f'{odom_x:.4f}', f'{odom_y:.4f}', f'{odom_yaw:.4f}',
            f'{gt_x:.4f}', f'{gt_y:.4f}', f'{gt_yaw:.4f}',
            f'{odom_dist:.4f}', f'{gt_dist:.4f}',
            gt_valid, self.state
        ])
        self.csv_file.flush()

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def plot_results(filename):
    print(f"Plotting results from {filename}...")
    try:
        df = pd.read_csv(filename)
        plt.figure(figsize=(10, 6))
        
        # Normalize Data
        odom_x = df['odom_x'].to_numpy(); odom_x -= odom_x[0]
        odom_y = df['odom_y'].to_numpy(); odom_y -= odom_y[0]
        
        gt_data = df[df['gt_valid'] == True]
        if not gt_data.empty:
            gt_x = gt_data['gt_x'].to_numpy(); gt_x -= gt_x[0]
            gt_y = gt_data['gt_y'].to_numpy(); gt_y -= gt_y[0]
            plt.plot(gt_x, gt_y, label='Ground Truth (SLAM)', color='green', linewidth=2)
            plt.scatter(gt_x[0], gt_y[0], color='green', marker='x')
            
        plt.plot(odom_x, odom_y, label='VIO Fused (EKF)', color='purple', linestyle='--')
        plt.scatter(0, 0, color='purple', marker='o', label='Start')
        
        plt.title('VIO Experiment: Fused Odom vs Ground Truth')
        plt.xlabel('X (m)'); plt.ylabel('Y (m)')
        plt.legend(); plt.grid(True); plt.axis('equal')
        
        plot_file = filename.replace('.csv', '.png')
        plt.savefig(plot_file)
        print(f"Saved plot to {plot_file}")
        plt.show()
    except Exception as e:
        print(f"Plotting error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopVIO()
    log_file = node.filename
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if os.path.exists(log_file):
            plot_results(log_file)

if __name__ == '__main__':
    main()
