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
import tf_transformations

# Dynamixel settings
DXL_ID_FL = 4  # Front Left
DXL_ID_FR = 3  # Front Right
DXL_ID_RL = 2  # Rear Left
DXL_ID_RR = 1  # Rear Right

VELOCITY_LIMIT = 1023  # Max velocity for MX-106R Dynamixel


Lx = 0.245  # Distance from center to wheel in x-axis (meters)
Ly = 0.2    # Distance from center to wheel in y-axis (meters)


class MecanumOdometry(Node):
    def __init__(self):
        super().__init__('mecanum_odometry')

        # Create an odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0  # Linear velocity in x-direction
        self.vy = 0.0  # Linear velocity in y-direction
        self.omega = 0.0 
        self.last_time = self.get_clock().now()

        # Dynamixel setup
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            print("Failed to open the port")
            quit()
        if not self.port_handler.setBaudRate(BAUDRATE):
            print("Failed to set baudrate")
            quit()

        # Initialize wheel velocities
        self.v_fl = 0.0
        self.v_fr = 0.0
        self.v_rl = 0.0
        self.v_rr = 0.0

        # Set up subscriptions to read the current speeds from the motors
        self.create_timer(0.1, self.read_motor_speeds)  # Read speeds at a fixed interval

    def read_motor_speeds(self):
        # Read current speeds from each Dynamixel motor
        self.v_fl = self.read_wheel_velocity(DXL_ID_FL)
        self.v_fr = self.read_wheel_velocity(DXL_ID_FR)
        self.v_rl = self.read_wheel_velocity(DXL_ID_RL)
        self.v_rr = self.read_wheel_velocity(DXL_ID_RR)

    def quaternion_from_euler(roll, pitch, yaw):
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def read_wheel_velocity(self, dxl_id):
        dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_PRESENT_SPEED)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Error reading velocity for motor {dxl_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return 0.0
        elif dxl_error != 0:
            print(f"Error for motor {dxl_id}: {self.packet_handler.getRxPacketError(dxl_error)}")
            return 0.0
        else:
            return self.convert_velocity(dxl_id, dxl_comm_result)

    def convert_velocity(self, dxl_id, velocity):
        # Convert the Dynamixel speed value to m/s (or your desired unit)
        # Scale based on your robot's configuration
        # Example scaling, adjust accordingly
        return velocity / 1023.0  # This assumes the maximum speed corresponds to 1 m/s

    def calculate_odometry(self):
        # Time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time difference in seconds
        self.last_time = current_time

        # Calculate the velocities for odometry calculation
        vx = (self.v_fr + self.v_fl + self.v_rl + self.v_rr) / 4.0
        vy = (-self.v_fr + self.v_fl + self.v_rl - self.v_rr) / 4.0
        omega = (self.v_fr - self.v_fl + self.v_rr - self.v_rl) / (Lx + Ly)

        # Calculate change in position
        delta_x = vx * math.cos(self.theta) * dt - vy * math.sin(self.theta) * dt
        delta_y = vx * math.sin(self.theta) * dt + vy * math.cos(self.theta) * dt
        delta_theta = omega * dt  # Change in orientation

        # Update the pose
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = self.normalize_angle(self.theta)

    def normalize_angle(self, angle):
        # Normalize the angle to be within [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


    def publish_odometry(self):
        # Create the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Corrected assignment of Quaternion
        odom_quat = self.quaternion_from_euler(0, 0, self.theta)  # This returns a Quaternion object
        odom_msg.pose.pose.orientation = odom_quat  # Correctly assigning the Quaternion object

        # Set the twist (velocity)
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.omega

        # Publish the odometry
        self.odom_pub.publish(odom_msg)

        # Create the TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom_quat  # Correctly assign the Quaternion here as well

        # Send the transform
        self.tf_broadcaster.sendTransform(transform)

    def main_loop(self):
        while rclpy.ok():
            self.calculate_odometry()
            self.publish_odometry()
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = MecanumOdometry()
    odometry_publisher.main_loop()

    # Shutdown
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
