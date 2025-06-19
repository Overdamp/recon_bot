#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class LaserScanMerger(Node):
    def __init__(self):
        super().__init__('laser_scan_merger')

        # Subscribers
        self.sub_front = self.create_subscription(
            LaserScan,
            '/front/scan',
            self.front_callback,
            10)

        self.sub_rear = self.create_subscription(
            LaserScan,
            '/back/scan',
            self.rear_callback,
            10)

        # Publisher
        self.pub_merged = self.create_publisher(
            LaserScan,
            '/scan_merged',
            10)

        # Store latest scans
        self.front_scan = None
        self.rear_scan = None

        # Timer for merging
        self.timer = self.create_timer(0.05, self.merge_scans)  # 20 Hz merge rate

        # Define LiDAR offsets relative to base_link (ปรับตามการติดตั้งจริง)
        self.front_offset = (0.2, 0.22, -1.57)  # 180 degrees for upside-down
        self.rear_offset = (-0.2, -0.22, -1.57)  # 180 degrees for upside-down

        # Specify if the LiDAR is mounted upside down
        self.front_upside_down = True
        self.rear_upside_down = True

    def front_callback(self, msg):
        self.front_scan = msg

    def rear_callback(self, msg):
        self.rear_scan = msg

    def transform_point(self, r, angle, offset, upside_down=False):
        """Transform a single point with given offset and optional upside down correction."""
        if upside_down:
            angle = -angle  # พลิกมุมสำหรับ LiDAR ที่คว่ำ

        x = r * math.cos(angle)
        y = r * math.sin(angle)

        dx, dy, dtheta = offset

        # Apply rotation then translation
        x_new = math.cos(dtheta) * x - math.sin(dtheta) * y + dx
        y_new = math.sin(dtheta) * x + math.cos(dtheta) * y + dy

        return x_new, y_new

    def merge_scans(self):
        if self.front_scan is None or self.rear_scan is None:
            return

        # Prepare to collect all points
        points_x = []
        points_y = []

        # Transform front scan points
        angle = self.front_scan.angle_min
        for r in self.front_scan.ranges:
            if self.front_scan.range_min < r < self.front_scan.range_max:
                x, y = self.transform_point(r, angle, self.front_offset, self.front_upside_down)
                points_x.append(x)
                points_y.append(y)
            angle += self.front_scan.angle_increment

        # Transform rear scan points
        angle = self.rear_scan.angle_min
        for r in self.rear_scan.ranges:
            if self.rear_scan.range_min < r < self.rear_scan.range_max:
                x, y = self.transform_point(r, angle, self.rear_offset, self.rear_upside_down)
                points_x.append(x)
                points_y.append(y)
            angle += self.rear_scan.angle_increment

        # Convert merged points back into LaserScan
        num_points = 360
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = (angle_max - angle_min) / num_points

        merged_ranges = [float('inf')] * num_points

        for x, y in zip(points_x, points_y):
            angle = math.atan2(y, x)
            r = math.hypot(x, y)
            idx = int((angle - angle_min) / angle_increment)
            if 0 <= idx < num_points:
                merged_ranges[idx] = min(merged_ranges[idx], r)

        # Replace inf with max range for cleaner output
        max_range = max(self.front_scan.range_max, self.rear_scan.range_max)
        merged_ranges = [r if r != float('inf') else max_range for r in merged_ranges]

        # Create merged scan message
        merged_scan = LaserScan()
        merged_scan.header.stamp = self.get_clock().now().to_msg()
        merged_scan.header.frame_id = 'Mobile_Base'
        merged_scan.angle_min = angle_min
        merged_scan.angle_max = angle_max
        merged_scan.angle_increment = angle_increment
        merged_scan.time_increment = 0.0
        merged_scan.scan_time = 0.0
        merged_scan.range_min = min(self.front_scan.range_min, self.rear_scan.range_min)
        merged_scan.range_max = max(self.front_scan.range_max, self.rear_scan.range_max)
        merged_scan.ranges = merged_ranges

        self.pub_merged.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()