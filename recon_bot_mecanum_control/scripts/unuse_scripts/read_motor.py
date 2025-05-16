#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from dynamixel_sdk import *  # Use Dynamixel SDK library
import math
from dxl_address import *

from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

# Dynamixel settings
DXL_ID_FL = 4  # Front Left
DXL_ID_FR = 3  # Front Right
DXL_ID_RL = 2  # Rear Left
DXL_ID_RR = 1  # Rear Right

VELOCITY_LIMIT = 1023  # Max velocity for MX-106R Dynamixel

Lx = 0.245  # Distance from center to wheel in x-axis (meters)
Ly = 0.2    # Distance from center to wheel in y-axis (meters)
WHEEL_RADIUS = 0.05  # Radius of the wheels (meters)

class ReadMotor(Node):
    def __init__(self):
        super().__init__('read_motor')

        # Create an odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Dynamixel setup
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open the port")
            quit()
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            quit()

        # Set up a timer to read motor speeds and calculate odometry
        self.create_timer(0.1, self.read_motor_speeds)

    def read_motor_speeds(self):
        # Read current speeds from each Dynamixel motor
        v_fl = self.read_wheel_velocity(DXL_ID_FL)
        v_fr = self.read_wheel_velocity(DXL_ID_FR)
        v_rl = self.read_wheel_velocity(DXL_ID_RL)
        v_rr = self.read_wheel_velocity(DXL_ID_RR)

        # Calculate odometry based on the wheel velocities
        self.calculate_odometry(v_fl, v_fr, v_rl, v_rr)

    def read_wheel_velocity(self, dxl_id):
        dxl_velocity, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_PRESENT_SPEED)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Error reading velocity for motor {dxl_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return 0.0
        elif dxl_error != 0:
            self.get_logger().error(f"Error for motor {dxl_id}: {self.packet_handler.getRxPacketError(dxl_error)}")
            return 0.0
        else:
            # Convert Dynamixel velocity to m/s
            return self.convert_velocity(dxl_velocity)

    def convert_velocity(self, velocity):
        # Dynamixel reports speed in a range from 0 to 1023, with 1023 being max speed
        # Convert to rad/s based on the motor's max speed in radians per second
        motor_max_rpm = 45.0  # Adjust based on MX-106R motor spec
        motor_max_rad_per_sec = (motor_max_rpm * 2 * math.pi) / 60.0
        return (velocity / VELOCITY_LIMIT) * motor_max_rad_per_sec * WHEEL_RADIUS

    def calculate_odometry(self, v_fl, v_fr, v_rl, v_rr):
        # Mecanum inverse kinematics to calculate robot velocities in x, y, and rotation
        self.vx = (v_fl + v_fr + v_rl + v_rr) / 4.0
        self.vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        self.omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (Lx + Ly))

        # Update robot pose using velocities
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publish odometry
        self.publish_odometry(current_time)

    def publish_odometry(self, current_time):
        # Create and populate odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = odom_quat[0]
        odom_msg.pose.pose.orientation.y = odom_quat[1]
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]

        # Set the velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.omega

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # Publish the transformation over TF
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = odom_quat[0]
        transform.transform.rotation.y = odom_quat[1]
        transform.transform.rotation.z = odom_quat[2]
        transform.transform.rotation.w = odom_quat[3]

        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = ReadMotor()
    rclpy.spin(odometry_publisher)

    # Shutdown
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
