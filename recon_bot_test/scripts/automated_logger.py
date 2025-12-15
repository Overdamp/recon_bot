import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import csv
import time
import math
import os
from datetime import datetime
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import qos_profile_sensor_data

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
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(Odometry, '/wheel_odom', self.odom_callback, qos_profile_sensor_data)
        
        # Timer for synchronous logging (10 Hz)
        self.create_timer(0.1, self.logging_loop)

    def odom_callback(self, msg):
        self.current_odom = msg

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

        # Extract AprilTag Data (via TF)
        apriltag_x, apriltag_y, apriltag_z = 0.0, 0.0, 0.0
        apriltag_dist = 0.0
        
        try:
            # Look up transform from camera frame to tag frame
            # We want the position of the TAG relative to the CAMERA (or vice versa)
            # apriltag_ros publishes TF: camera_frame -> tag_frame (named 'apriltag_marker_23')
            # So we want transform: target='zed_left_camera_frame', source='apriltag_marker_23'
            # Wait, usually we want Tag position in Camera frame.
            # So target='zed_left_camera_frame', source='apriltag_marker_23' gives position of Tag in Camera frame.
            
            # Note: The frame name depends on how apriltag_ros is configured.
            # In tags_36h11.yaml we set: tag_frames: ["apriltag_marker_23"]
            
            t = self.tf_buffer.lookup_transform(
                'zed_left_camera_frame', 
                'apriltag_marker_23', 
                rclpy.time.Time()
            )
            
            ax = t.transform.translation.x
            ay = t.transform.translation.y
            az = t.transform.translation.z
            
            apriltag_x, apriltag_y, apriltag_z = ax, ay, az
            
            if self.start_apriltag_pose is None:
                self.start_apriltag_pose = (ax, ay, az)
                self.get_logger().info('AprilTag Start Pose Recorded')
            
            # Calculate displacement from start vector
            dx = ax - self.start_apriltag_pose[0]
            dy = ay - self.start_apriltag_pose[1]
            dz = az - self.start_apriltag_pose[2]
            apriltag_dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Tag not visible
            pass

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
