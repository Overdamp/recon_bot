#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from dynamixel_sdk import *  # Use Dynamixel SDK library
import numpy as np

# Dynamixel Settings
DXL_ID_FL = 1  # Front Left
DXL_ID_FR = 2  # Front Right
DXL_ID_RL = 3  # Rear Left
DXL_ID_RR = 4  # Rear Right

BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'  # Adjust for your system

# Protocol version (2.0 for most modern Dynamixels)
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102  # Torque (current control)
ADDR_PRESENT_CURRENT = 126

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MAX_CURRENT_LIMIT = 2048  # Max current for torque (adjust based on your motor)

# Constants for joystick scaling
MAX_LINEAR_ACCELERATION = 1.0  # Maximum linear acceleration in m/s^2
MAX_ANGULAR_ACCELERATION = 1.0  # Maximum angular acceleration in rad/s^2

class MecanumFullDynamicController(Node):

    def __init__(self):
        super().__init__('mecanum_full_dynamic_controller')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joystick_callback,
            10)
        
        # Initialize dynamixel connection
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.port_handler.openPort()
        self.port_handler.setBaudRate(BAUDRATE)

        self.enable_torque(DXL_ID_FL)
        self.enable_torque(DXL_ID_FR)
        self.enable_torque(DXL_ID_RL)
        self.enable_torque(DXL_ID_RR)

        self.Lx = 0.2  # Distance from center to wheel in x-axis (meters)
        self.Ly = 0.2  # Distance from center to wheel in y-axis (meters)

        self.robot_mass = 10.0  # Mass of the robot (in kg)
        self.wheel_radius = 0.05  # Radius of the wheel (in meters)
        self.inertia_z = 0.5  # Moment of inertia around z-axis (adjust for your robot)

    def joystick_callback(self, joy_msg):
        # Get acceleration commands from joystick axes
        ax = joy_msg.axes[1] * MAX_LINEAR_ACCELERATION  # Forward/backward
        ay = joy_msg.axes[0] * MAX_LINEAR_ACCELERATION  # Left/right strafing
        az = joy_msg.axes[3] * MAX_ANGULAR_ACCELERATION  # Rotation
        
        # Compute the dynamic forces and torques
        Fx, Fy, Tz = self.compute_robot_forces(ax, ay, az)

        # Compute the torque for each wheel based on the dynamic forces and torques
        torque_fl, torque_fr, torque_rl, torque_rr = self.compute_wheel_torques(Fx, Fy, Tz)

        # Set torque (current) for each wheel
        self.set_wheel_torque(DXL_ID_FL, self.convert_torque_to_current(torque_fl))
        self.set_wheel_torque(DXL_ID_FR, self.convert_torque_to_current(torque_fr))
        self.set_wheel_torque(DXL_ID_RL, self.convert_torque_to_current(torque_rl))
        self.set_wheel_torque(DXL_ID_RR, self.convert_torque_to_current(torque_rr))

    def enable_torque(self, dxl_id):
        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def compute_robot_forces(self, ax, ay, az):
        # Calculate the forces in x and y directions and the torque around z
        Fx = self.robot_mass * ax  # Force in x-direction
        Fy = self.robot_mass * ay  # Force in y-direction
        Tz = self.inertia_z * az   # Torque around z (angular acceleration)
        return Fx, Fy, Tz

    def compute_wheel_torques(self, Fx, Fy, Tz):
        # The dynamic model equations for mecanum wheels
        # Inverse kinematics that map forces and torques to wheel torques
        torque_fl = (Fx - Fy - Tz / (self.Lx + self.Ly)) / self.wheel_radius
        torque_fr = (Fx + Fy + Tz / (self.Lx + self.Ly)) / self.wheel_radius
        torque_rl = (Fx + Fy - Tz / (self.Lx + self.Ly)) / self.wheel_radius
        torque_rr = (Fx - Fy + Tz / (self.Lx + self.Ly)) / self.wheel_radius

        return torque_fl, torque_fr, torque_rl, torque_rr

    def set_wheel_torque(self, dxl_id, current):
        # Dynamixel motors take current (in mA) to control torque
        current = max(-MAX_CURRENT_LIMIT, min(current, MAX_CURRENT_LIMIT))  # Limit current
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_CURRENT, int(current))

    def convert_torque_to_current(self, torque):
        # Convert torque to current for the Dynamixel motor
        # This depends on the motor's torque constant, typically provided in the datasheet
        torque_constant = 0.01  # Nm/A (example value, adjust based on your motor)
        current = torque / torque_constant
        return int(current)

def main(args=None):
    rclpy.init(args=args)
    mecanum_dynamic_controller = MecanumFullDynamicController()
    rclpy.spin(mecanum_dynamic_controller)
    
    # Shutdown
    mecanum_dynamic_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
