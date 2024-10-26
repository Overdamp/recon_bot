#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from dynamixel_sdk import *  # Use Dynamixel SDK library
from dxl_address import *
import math
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import JointState

# Dynamixel Settings
DXL_ID_FL = 4  # Front Left
DXL_ID_FR = 3  # Front Right
DXL_ID_RL = 2  # Rear Left
DXL_ID_RR = 1  # Rear Right

VELOCITY_LIMIT = 1023  # Max velocity for MX-106R Dynamixel
MAX_LINEAR_VELOCITY = 1.0  # m/s
MAX_ANGULAR_VELOCITY = 1.0  # rad/s

# Direction multipliers for each motor
DIR_FL = 1
DIR_FR = -1
DIR_RL = 1
DIR_RR = -1

# Robot dimensions
Lx = 0.245  # Distance from center to wheel in x-axis (m)
Ly = 0.2    # Distance from center to wheel in y-axis (m)
WHEEL_RADIUS = 0.062  # Radius of the wheels (m)

class MecanumJoystickOdometryController(Node):

    def __init__(self):
        super().__init__('mecanum_joystick_odometry_controller')
        
        # Joystick subscription
        self.joystick_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joystick_callback,
            10
        )
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Dynamixel setup
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.init_dynamixel()

        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_time = self.get_clock().now()

        # Timer for odometry calculation
        self.create_timer(0.1, self.read_motor_speeds)

    def init_dynamixel(self):
        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open the port")
            quit()

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            quit()

        for id in range(1, 5):
            self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_MX_CW_ANGLE_LIMIT, DXL_CW_ANGLE_TO_Z)
            self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_MX_CCW_ANGLE_LIMIT, DXL_CCW_ANGLETO_Z)
            print(f"Dynamixel #{id} set to wheel mode")

    def joystick_callback(self, joy_msg):
        # Get values from joystick axes
        vx = joy_msg.axes[1] * MAX_LINEAR_VELOCITY  # Forward/backward
        vy = joy_msg.axes[0] * MAX_LINEAR_VELOCITY  # Left/right strafing
        wz = joy_msg.axes[3] * MAX_ANGULAR_VELOCITY  # Rotation

        # Mecanum kinematics
        v_fl = (vx - vy - wz * (Lx + Ly)) * DIR_FL
        v_fr = (vx + vy + wz * (Lx + Ly)) * DIR_FR
        v_rl = (vx + vy - wz * (Lx + Ly)) * DIR_RL
        v_rr = (vx - vy + wz * (Lx + Ly)) * DIR_RR

        # Set motor speeds
        self.set_wheel_velocity(DXL_ID_FL, self.convert_velocity(v_fl))
        self.set_wheel_velocity(DXL_ID_FR, self.convert_velocity(v_fr))
        self.set_wheel_velocity(DXL_ID_RL, self.convert_velocity(v_rl))
        self.set_wheel_velocity(DXL_ID_RR, self.convert_velocity(v_rr))

    def set_wheel_velocity(self, dxl_id, velocity):
        # Limit and map velocity for Dynamixel motors
        velocity = max(-VELOCITY_LIMIT, min(velocity, VELOCITY_LIMIT))
        if velocity < 0:
            velocity = 1024 + abs(velocity)

        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_ACCELERATION, 10)
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_MOVING_SPEED, int(velocity))

    def convert_velocity(self, velocity):
        # Scaling factor for motor velocities
        motor_velocity = velocity / MAX_LINEAR_VELOCITY * VELOCITY_LIMIT
        return int(motor_velocity)
    
    def publish_joint_states(self, current_time):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()

        # Joint names (these should match the URDF file)
        joint_state_msg.name = ['J_Wheel_LF', 'J_Wheel_LR', 
                                'J_Wheel_RF', 'J_Wheel_RR']

        # Update the joint positions based on wheel velocities
        # Convert the velocities to joint positions (in radians) based on time passed
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.joint_positions[0] += self.vx * dt / WHEEL_RADIUS  # FL
        self.joint_positions[1] += self.vx * dt / WHEEL_RADIUS  # FR
        self.joint_positions[2] += self.vx * dt / WHEEL_RADIUS  # RL
        self.joint_positions[3] += self.vx * dt / WHEEL_RADIUS  # RR

        # Assign joint positions
        joint_state_msg.position = self.joint_positions

        # Publish the joint states
        self.joint_state_pub.publish(joint_state_msg)


    def read_motor_speeds(self):
        # Read speeds from all motors
        v_fl = self.read_wheel_velocity(DXL_ID_FL)
        v_fr = self.read_wheel_velocity(DXL_ID_FR)
        v_rl = self.read_wheel_velocity(DXL_ID_RL)
        v_rr = self.read_wheel_velocity(DXL_ID_RR)

        # Calculate and publish odometry
        self.calculate_odometry(v_fl, v_fr, v_rl, v_rr)

    def read_wheel_velocity(self, dxl_id):
        dxl_velocity, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_PRESENT_SPEED)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Error reading velocity for motor {dxl_id}")
            return 0.0
        return self.convert_velocity(dxl_velocity)

    def calculate_odometry(self, v_fl, v_fr, v_rl, v_rr):
        # Mecanum kinematics for odometry
        self.vx = (v_fl + v_fr + v_rl + v_rr) / 4.0
        self.vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        self.omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (Lx + Ly))

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.publish_odometry(current_time)

    def publish_odometry(self, current_time):
        # Publish odometry and TF data
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_quat = quaternion_from_euler(0, 0, self.theta)
        
        # Assign Quaternion components explicitly
        odom_msg.pose.pose.orientation.x = odom_quat[0]
        odom_msg.pose.pose.orientation.y = odom_quat[1]
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]
        
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.omega
        self.odom_pub.publish(odom_msg)

        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = odom_quat[0]
        transform.transform.rotation.y = odom_quat[1]
        transform.transform.rotation.z = odom_quat[2]
        transform.transform.rotation.w = odom_quat[3]
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    controller = MecanumJoystickOdometryController()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
