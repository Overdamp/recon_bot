#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
from dxl_address import *
from sensor_msgs.msg import JointState

DXL_ID_FL = 4
DXL_ID_FR = 3
DXL_ID_RL = 2
DXL_ID_RR = 1

class MotorReader(Node):

    def __init__(self):
        super().__init__('motor_read')
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.init_dynamixel()

        # Timer to read motor speeds
        self.create_timer(0.1, self.read_motor_speeds)

    def init_dynamixel(self):
        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open port')
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to set baudrate')

    def read_motor_speeds(self):
        # Read motor speeds from all motors
        speed_fl = self.read_motor_speed(DXL_ID_FL)
        speed_fr = self.read_motor_speed(DXL_ID_FR)
        speed_rl = self.read_motor_speed(DXL_ID_RL)
        speed_rr = self.read_motor_speed(DXL_ID_RR)

        # Convert the motor speeds to floats and create JointState message
        joint_state_msg = JointState()
        # Ensure that the motor speeds are floats and handle cases where they may be invalid
        joint_state_msg.velocity = [float(speed_fl) if speed_fl is not None else 0.0,
                                    float(speed_fr) if speed_fr is not None else 0.0,
                                    float(speed_rl) if speed_rl is not None else 0.0,
                                    float(speed_rr) if speed_rr is not None else 0.0]
        self.joint_state_pub.publish(joint_state_msg)

    def read_motor_speed(self, motor_id):
        # Read current speed of a motor
        try:
            speed, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, ADDR_MX_PRESENT_SPEED)
            if speed is None:
                self.get_logger().error(f"Failed to read speed for motor {motor_id}")
                return 0.0
            return speed
        except Exception as e:
            self.get_logger().error(f"Error reading motor {motor_id}: {e}")
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    motor_reader = MotorReader()
    rclpy.spin(motor_reader)

    motor_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
