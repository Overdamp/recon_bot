#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        # ความเร็วสูงสุดที่หุ่นยนต์สามารถเคลื่อนไปได้
        self.max_speed = 0.174/4

        # ตั้งค่าระยะที่ต้องการไปที่ ArUco
        self.target_distance_to_marker = 0.2  # ระยะห่างจาก ArUco marker ที่ต้องการให้หุ่นหยุด
        self.y_offset = -0.08  # Offset สำหรับแกน Y เพื่อให้ชิดทางซ้ายมากขึ้น

        # Subscriber สำหรับข้อมูล ArUco marker
        self.aruco_subscription = self.create_subscription(
            PoseStamped, '/aruco_marker_pose', self.aruco_callback, 10
        )

        # Publisher สำหรับส่งคำสั่งการเคลื่อนที่
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ตั้งตัวแปรเพื่อเก็บข้อมูลจาก ArUco marker
        self.angle_to_marker = None
        self.distance_to_marker = None
        self.cmd = Twist()  # คำสั่งเคลื่อนที่

    def aruco_callback(self, msg: PoseStamped):
        # รับข้อมูลจาก PoseStamped ซึ่งเป็นตำแหน่งของ ArUco marker
        self.angle_to_marker = msg.pose.position.y  # ใช้ค่าแกน Y เพื่อเลื่อนให้ตรงกลาง
        self.distance_to_marker = msg.pose.position.x  # ระยะห่างจาก ArUco marker

        # ถ้า marker อยู่ข้างหน้า ให้ควบคุมหุ่นยนต์เพื่อวิ่งเข้าไปที่ marker
        if self.distance_to_marker > 0:
            self.control_docking(self.angle_to_marker, self.distance_to_marker)

    def control_docking(self, offset_y, distance_to_marker):
        # ปรับการเลื่อนในแกน Y ให้ชิดซ้ายมากขึ้นโดยใช้ y_offset
        adjusted_offset_y = offset_y - self.y_offset

        # ปรับการเลื่อนในแกน Y เพื่อเลื่อนให้ตรงกลางกับ ArUco marker
        if abs(adjusted_offset_y) > 0.05:  # ถ้าห่างจากตรงกลางมากกว่า 5 เซนติเมตร
            if adjusted_offset_y > 0:
                self.get_logger().info("เลื่อนเข้าไปทางซ้าย (แกน Y)")
                self.cmd.linear.y = max(min(self.max_speed, 0.1), self.max_speed)
            else:
                self.get_logger().info("เลื่อนออกจาก marker ทางขวา (แกน Y)")
                self.cmd.linear.y = max(min(-self.max_speed, -0.1), -self.max_speed)
        else:
            self.get_logger().info("อยู่ตรงกลางแล้ว")
            self.cmd.linear.y = 0.0  # หยุดการเลื่อนในแกน Y เมื่ออยู่ตรงกลาง

        # เคลื่อนที่เข้าไปข้างหน้าในแกน X
        if distance_to_marker > self.target_distance_to_marker:  # ถ้ายังไม่ถึงระยะที่กำหนด
            self.get_logger().info("เคลื่อนที่ไปข้างหน้า")
            self.cmd.linear.x = max(min(0.2, self.max_speed), -self.max_speed)
        else:
            self.get_logger().info("หยุดเพราะถึงระยะที่กำหนด")
            self.cmd.linear.x = 0.0

        # ส่งคำสั่งเคลื่อนที่
        self.cmd_vel_pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()