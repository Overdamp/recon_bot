#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import csv
import time
import math
import os
from datetime import datetime

class AutomatedLogger(Node):
    def __init__(self):
        super().__init__('automated_logger')

        # Parameters
        self.declare_parameter('log_dir', os.path.expanduser('~/recon_bot_logs'))
        self.log_dir = self.get_parameter('log_dir').value
        
        # Create log directory if it doesn't exist
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        # Create CSV file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f'test_data_{timestamp}.csv')
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write Header
        self.csv_writer.writerow([
            'timestamp', 
            'odom_x', 'odom_y', 'odom_yaw', 
            'apriltag_x', 'apriltag_y', 'apriltag_z', # AprilTag is 3D
            'odom_dist_from_start', 'apriltag_dist_from_start'
        ])
        
        self.get_logger().info(f'Logging to: {self.filename}')

        # State variables
        self.start_odom_pose = None
        self.start_apriltag_pose = None
        
        self.current_odom = None
        self.current_apriltag = None
        
        # Subscribers
        self.create_subscription(Odometry, '/wheel_odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/apriltag_marker_pose', self.apriltag_callback, 10)
        
        # Timer for synchronous logging (10 Hz)
        self.create_timer(0.1, self.logging_loop)

    def odom_callback(self, msg):
        self.current_odom = msg

    def apriltag_callback(self, msg):
        self.current_apriltag = msg

    def logging_loop(self):
        if self.current_odom is None:
            return # Wait for odom at least

        # Extract Odom Data
        odom_x = self.current_odom.pose.pose.position.x
        odom_y = self.current_odom.pose.pose.position.y
        # Simple Yaw from Quaternion
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Initialize start pose if needed
        if self.start_odom_pose is None:
            self.start_odom_pose = (odom_x, odom_y)
            self.get_logger().info('Odom Start Pose Recorded')

        # Calculate Odom Distance from Start
        odom_dist = math.sqrt((odom_x - self.start_odom_pose[0])**2 + (odom_y - self.start_odom_pose[1])**2)

        # Extract AprilTag Data (if available)
        apriltag_x, apriltag_y, apriltag_z = 0.0, 0.0, 0.0
        apriltag_dist = 0.0
        
        if self.current_apriltag:
            # Note: apriltag_marker.py publishes position of MARKER in CAMERA frame
            # So "Distance" is just the magnitude of the position vector
            # But we want "Distance Moved". 
            # If robot moves 1m away, the magnitude increases by 1m.
            # Let's record the raw vector.
            
            ax = self.current_apriltag.pose.position.x
            ay = self.current_apriltag.pose.position.y
            az = self.current_apriltag.pose.position.z
            
            apriltag_x, apriltag_y, apriltag_z = ax, ay, az
            
            if self.start_apriltag_pose is None:
                self.start_apriltag_pose = (ax, ay, az)
                self.get_logger().info('AprilTag Start Pose Recorded')
            
            # Calculate displacement from start vector
            # This is roughly the distance moved if rotation is small
            dx = ax - self.start_apriltag_pose[0]
            dy = ay - self.start_apriltag_pose[1]
            dz = az - self.start_apriltag_pose[2]
            apriltag_dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Write to CSV
        self.csv_writer.writerow([
            time.time(),
            f'{odom_x:.4f}', f'{odom_y:.4f}', f'{odom_yaw:.4f}',
            f'{apriltag_x:.4f}', f'{apriltag_y:.4f}', f'{apriltag_z:.4f}',
            f'{odom_dist:.4f}', f'{apriltag_dist:.4f}'
        ])
        self.csv_file.flush() # Ensure data is written

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AutomatedLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
