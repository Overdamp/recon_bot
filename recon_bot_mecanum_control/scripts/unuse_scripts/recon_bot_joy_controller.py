#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from dynamixel_sdk import *  # Use Dynamixel SDK library
from dxl_address import *

# Dynamixel Settings
DXL_ID_FL = 4  # Front Left
DXL_ID_FR = 3  # Front Right
DXL_ID_RL = 2  # Rear Left
DXL_ID_RR = 1  # Rear Right

VELOCITY_LIMIT = 1023  # Max velocity for MX-28T Dynamixel

# Constants for joystick scaling
MAX_LINEAR_VELOCITY = 1.0  # Maximum linear velocity in m/s
MAX_ANGULAR_VELOCITY = 1.0  # Maximum angular velocity in rad/s

# Direction multipliers for each motor
DIR_FL = 1  
DIR_FR = -1
DIR_RL = 1
DIR_RR = -1


class MecanumJoystickController(Node):

    def __init__(self):
        super().__init__('mecanum_joystick_controller')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
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

        self.Lx = 0.245  # Distance from center to wheel in x-axis (meters)
        self.Ly = 0.2  # Distance from center to wheel in y-axis (meters)

        #Dynamixel Port check
        if self.port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        if self.port_handler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        for id in range(1, 5):

            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_MX_CW_ANGLE_LIMIT, DXL_CW_ANGLE_TO_Z)

            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(dxl_error))
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, id, ADDR_MX_CCW_ANGLE_LIMIT, DXL_CCW_ANGLETO_Z)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully set to wheel mode" % id)

    def joystick_callback(self, joy_msg):
        # Get values from joystick axes
        vx = joy_msg.axes[1] * MAX_LINEAR_VELOCITY   # Forward/backward
        vy = joy_msg.axes[0] * MAX_LINEAR_VELOCITY   # Left/right strafing
        wz = joy_msg.axes[3] * MAX_ANGULAR_VELOCITY  # Rotation
        
        # Mecanum wheel inverse kinematics for 4 wheels
        v_fl = (vx - vy - wz * (self.Lx + self.Ly)) * DIR_FL
        v_fr = (vx + vy + wz * (self.Lx + self.Ly)) * DIR_FR
        v_rl = (vx + vy - wz * (self.Lx + self.Ly)) * DIR_RL
        v_rr = (vx - vy + wz * (self.Lx + self.Ly)) * DIR_RR

        # Convert velocities to Dynamixel-compatible values (scale as needed)
        self.set_wheel_velocity(DXL_ID_FL, self.convert_velocity(v_fl))
        self.set_wheel_velocity(DXL_ID_FR, self.convert_velocity(v_fr))
        self.set_wheel_velocity(DXL_ID_RL, self.convert_velocity(v_rl))
        self.set_wheel_velocity(DXL_ID_RR, self.convert_velocity(v_rr))
        

    def enable_torque(self, dxl_id):
        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_wheel_velocity(self, dxl_id, velocity):
        # Dynamixel motors need velocity to be in the form of a scaled value, so convert accordingly
        velocity = max(-VELOCITY_LIMIT, min(velocity, VELOCITY_LIMIT))  # Limit velocity to motor max
        if velocity >= 0:
        # Forward: map 0-1023 directly to Dynamixel's 0-1023
            velocity = velocity
        elif velocity <= 0:
        # Reverse: map -1023-0 to Dynamixel's 1024-2047
            velocity = 1024 + abs(velocity)
        else:
            velocity = 0

        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_ACCELERATION, 10)
        # print(velocity, dxl_id)
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_MOVING_SPEED , int(velocity))

        # vel1 = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_MOVING_SPEED)
        # print(vel1,dxl_id)

    def convert_velocity(self, velocity):
        # Scale the velocity from m/s to motor value (e.g., scaling factor is based on motor specification)
        motor_velocity = velocity * 1023  # Example scaling factor, adjust for your motor
        return int(motor_velocity)

def main(args=None):
    rclpy.init(args=args)
    mecanum_joystick_controller = MecanumJoystickController()
    rclpy.spin(mecanum_joystick_controller)
    
    # Shutdown
    mecanum_joystick_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
