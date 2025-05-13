#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Float64MultiArray
import numpy as np

class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')
        
        # พารามิเตอร์ของหุ่นยนต์
        self.wheel_radius = 0.062  # รัศมีล้อ (เมตร)
        self.lx = 0.245            # ระยะแกน X (เมตร)
        self.ly = 0.2            # ระยะแกน Y (เมตร)
        
        # สร้าง Inverse Kinematics Matrix
        self.inv_kinematics = (1/self.wheel_radius) * np.array([
            [1, -1, -(self.lx + self.ly)],
            [1, 1, (self.lx + self.ly)],
            [1, -1, (self.lx + self.ly)],
            [1, 1, -(self.lx + self.ly)]
        ])
        
        # สร้าง Forward Kinematics Matrix
        self.fwd_kinematics = (self.wheel_radius/4) * np.array([
            [1, 1, 1, 1],
            [-1, 1, -1, 1],
            [-1/(self.lx + self.ly), 1/(self.lx + self.ly), 1/(self.lx + self.ly), -1/(self.lx + self.ly)]
        ])
        
        # Subscriber สำหรับอ่านค่า Joint State
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscriber สำหรับรับคำสั่งความเร็ว (cmd_vel)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher สำหรับส่งคำสั่งความเร็วไปยัง controller
        self.vel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )
        
        # ตัวแปรเก็บสถานะล้อ
        self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint']
        
    def joint_state_callback(self, msg):
        # อ่านความเร็วของล้อจาก JointState
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.wheel_velocities[i] = msg.velocity[idx]
        
        # คำนวณ Forward Kinematics เพื่อหาความเร็วหุ่นยนต์
        wheel_vel_array = np.array(self.wheel_velocities)
        robot_vel = np.dot(self.fwd_kinematics, wheel_vel_array)
        
        self.get_logger().info(f'Robot Velocity: vx={robot_vel[0]:.2f}, vy={robot_vel[1]:.2f}, omega={robot_vel[2]:.2f}')
    
    def cmd_vel_callback(self, msg):
        # รับคำสั่งความเร็วจาก cmd_vel
        robot_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        
        # คำนวณ Inverse Kinematics เพื่อหาความเร็วของล้อ
        wheel_speeds = np.dot(self.inv_kinematics, robot_vel)
        
        # สร้างข้อความสำหรับส่งคำสั่งความเร็ว
        vel_msg = Float64MultiArray()
        vel_msg.data = wheel_speeds.tolist()
        
        # ส่งคำสั่งไปยัง velocity controller
        self.vel_pub.publish(vel_msg)
        self.get_logger().info(f'Wheel Speeds: {wheel_speeds}')

def main(args=None):
    rclpy.init(args=args)
    node = MecanumController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()