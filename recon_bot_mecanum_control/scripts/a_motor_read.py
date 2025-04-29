#!/usr/bin/env python3

# --- Import Libraries ---
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from dynamixel_sdk import *
from dxl_address import *

# --- Motor IDs Mapping ---
DXL_IDS = {
    "J_Wheel_LF": 4,
    "J_Wheel_RF": 3,
    "J_Wheel_LR": 2,
    "J_Wheel_RR": 1
}

# --- Motor Reader Node ---
class MotorReader(Node):

    def __init__(self):
        super().__init__('motor_read')
        
        # Create a publisher for /joint_states topic
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Dynamixel SDK handlers
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_MX_PRESENT_SPEED, 2)

        # Open serial port and set baudrate
        self.open_port_and_baud()

        # Initialize motors to be added into group sync read
        self.init_sync_read()

        # Initialize storage for last known positions
        self.last_positions = {name: 0.0 for name in DXL_IDS.keys()}
        self.last_time = self.get_clock().now()

        # Create a timer to read motor state at 10Hz
        self.create_timer(0.1, self.read_and_publish_joint_states)

        self.get_logger().info("✅ MotorReader node initialized and running.")

    def open_port_and_baud(self):
        # Open the port and set baudrate
        if not self.port_handler.openPort():
            self.get_logger().error('❌ Failed to open port')
        else:
            self.get_logger().info('✅ Port opened successfully')
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('❌ Failed to set baudrate')
        else:
            self.get_logger().info('✅ Baudrate set successfully')

    def init_sync_read(self):
        # Add all motors to the GroupSyncRead
        for motor_id in DXL_IDS.values():
            if not self.group_sync_read.addParam(motor_id):
                self.get_logger().error(f'❌ Failed to add motor ID {motor_id} to GroupSyncRead')

    def read_and_publish_joint_states(self):
        # Read all motor states and publish as JointState
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Perform sync read
        if self.group_sync_read.txRxPacket() != COMM_SUCCESS:
            self.get_logger().error("❌ Failed GroupSyncRead txRxPacket()")
            return

        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = now.to_msg()
        joint_state_msg.header.frame_id = "Mobile_Base"
        joint_state_msg.name = list(DXL_IDS.keys())
        joint_state_msg.velocity = []
        joint_state_msg.position = []

        # Process each motor
        for name, motor_id in DXL_IDS.items():
            if self.group_sync_read.isAvailable(motor_id, ADDR_MX_PRESENT_SPEED, 2):
                speed = self.group_sync_read.getData(motor_id, ADDR_MX_PRESENT_SPEED, 2)

                # Convert unsigned to signed value
                if speed > 1023:
                    speed = -(speed - 1024)

                # Normalize to rad/s
                speed_rad = self.dxl_to_rads(speed)

                # Integrate position
                self.last_positions[name] += speed_rad * dt

                joint_state_msg.velocity.append(speed_rad)
                joint_state_msg.position.append(self.last_positions[name])
            else:
                # No data available for this motor
                self.get_logger().warn(f"⚠️ No data for motor {name}")
                joint_state_msg.velocity.append(0.0)
                joint_state_msg.position.append(self.last_positions[name])

        # Publish the joint states
        self.joint_state_pub.publish(joint_state_msg)

    def dxl_to_rads(self, value):
        # Convert raw Dynamixel speed (range -1023~1023) to radians per second
        max_rpm = 45.0  # MX-106 default max RPM
        max_rad_per_sec = max_rpm * 2 * math.pi / 60.0
        return value / 1023.0 * max_rad_per_sec

    def destroy_node(self):
        # Clear parameters and close port before destroying the node
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
