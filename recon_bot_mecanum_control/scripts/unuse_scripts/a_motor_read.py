#!/usr/bin/env python3

# --- Import Libraries ---
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from dynamixel_sdk import *
from dxl_address2 import *

# --- Motor IDs Mapping ---
DXL_IDS = {
    "J_Wheel_FL": 4,
    "J_Wheel_FR": 3,
    "J_Wheel_RL": 2,
    "J_Wheel_RR": 1
}

# --- Motor Reader Node ---
class MotorReader(Node):

    def __init__(self):
        super().__init__('motor_read')

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.group_sync_read = GroupSyncRead(
            self.port_handler, 
            self.packet_handler, 
            ADDR_MX_PRESENT_VELOCITY, 
            LEN_MX_PRESENT_VELOCITY
        )

        self.open_port_and_baud()
        self.init_sync_read()

        self.last_positions = {name: 0.0 for name in DXL_IDS.keys()}
        self.last_time = self.get_clock().now()

        self.create_timer(0.1, self.read_and_publish_joint_states)  # 10Hz

        self.get_logger().info("✅ MotorReader node initialized and running.")

    def open_port_and_baud(self):
        if not self.port_handler.openPort():
            self.get_logger().error('❌ Failed to open port')
        else:
            self.get_logger().info('✅ Port opened successfully')
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('❌ Failed to set baudrate')
        else:
            self.get_logger().info('✅ Baudrate set successfully')

    def init_sync_read(self):
        for motor_id in DXL_IDS.values():
            if not self.group_sync_read.addParam(motor_id):
                self.get_logger().error(f'❌ Failed to add motor ID {motor_id} to GroupSyncRead')

    def read_and_publish_joint_states(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.group_sync_read.txRxPacket() != COMM_SUCCESS:
            self.get_logger().error("❌ Failed GroupSyncRead txRxPacket()")
            return

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = now.to_msg()
        joint_state_msg.header.frame_id = "Mobile_Base"
        joint_state_msg.name = list(DXL_IDS.keys())
        joint_state_msg.velocity = []
        joint_state_msg.position = []

        for name, motor_id in DXL_IDS.items():
            if self.group_sync_read.isAvailable(motor_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY):
                data = self.group_sync_read.getData(motor_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)

                if data > (1 << 31):
                    data -= (1 << 32)

                wheel_rad_per_sec = self.velocity_to_rads(data)
                self.last_positions[name] += wheel_rad_per_sec * dt

                joint_state_msg.velocity.append(wheel_rad_per_sec)
                joint_state_msg.position.append(self.last_positions[name])
            else:
                self.get_logger().warn(f"⚠️ No data for motor {name}")
                joint_state_msg.velocity.append(0.0)
                joint_state_msg.position.append(self.last_positions[name])

        self.joint_state_pub.publish(joint_state_msg)

    def velocity_to_rads(self, raw_velocity):
        rpm = raw_velocity * 0.229  # Conversion from Dynamixel unit to RPM
        return rpm * 2 * math.pi / 60.0

    def destroy_node(self):
        self.group_sync_read.clearParam()
        if self.port_handler.isPortOpen():
            self.port_handler.closePort()
        super().destroy_node()

# --- Main function ---
def main(args=None):
    rclpy.init(args=args)
    try:
        node = MotorReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
