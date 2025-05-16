#!/usr/bin/env python3

# --- Import Libraries ---
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import *
from dxl_address import *
import math

# --- Motor Configuration ---
DXL_IDS = {
    'FL': 4,  # Front Left
    'FR': 3,  # Front Right
    'RL': 2,  # Rear Left
    'RR': 1   # Rear Right
}

DIR = {
    'FL': 1,
    'FR': -1,
    'RL': 1,
    'RR': -1
}

VELOCITY_LIMIT = 1023  # Dynamixel maximum raw velocity
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Robot physical parameters
Lx = 0.245  # meters
Ly = 0.2    # meters
WHEEL_RADIUS = 0.062  # meters

# MX-106 spec constants
MAX_RPM = 45.0  # Max RPM from datasheet
MAX_RAD_PER_SEC = MAX_RPM * 2 * math.pi / 60.0

class MotorWriter(Node):

    def __init__(self):
        super().__init__('motor_writer')

        # ROS2 subscription
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Dynamixel setup
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_MX_MOVING_SPEED, 2)

        self.open_port()
        self.init_motors()

    def open_port(self):
        if not self.port_handler.openPort():
            self.get_logger().fatal('❌ Failed to open the port')
            raise RuntimeError('Failed to open port')
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().fatal('❌ Failed to set baudrate')
            raise RuntimeError('Failed to set baudrate')
        self.get_logger().info('✅ Port initialized')

    def init_motors(self):
        for name, dxl_id in DXL_IDS.items():
            self.enable_torque(dxl_id)
            self.set_wheel_mode(dxl_id)
            self.set_acceleration(dxl_id, 20)  # Set once here

    def enable_torque(self, dxl_id):
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if result != COMM_SUCCESS or error != 0:
            self.get_logger().error(f"❌ Failed to enable torque for motor {dxl_id}")

    def set_wheel_mode(self, dxl_id):
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_CW_ANGLE_LIMIT, 0)
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_CCW_ANGLE_LIMIT, 0)

    def set_acceleration(self, dxl_id, accel_value):
        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_GOAL_ACCELERATION, accel_value)

    def cmd_vel_callback(self, twist_msg):
        vx, vy, omega = twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z
        lxly = Lx + Ly

        # Kinematic calculation
        velocities = {
            'FL': (vx - vy - omega * lxly) / WHEEL_RADIUS * DIR['FL'],
            'FR': (vx + vy + omega * lxly) / WHEEL_RADIUS * DIR['FR'],
            'RL': (vx + vy - omega * lxly) / WHEEL_RADIUS * DIR['RL'],
            'RR': (vx - vy + omega * lxly) / WHEEL_RADIUS * DIR['RR']
        }

        self.sync_write.clearParam()
        for name, dxl_id in DXL_IDS.items():
            raw_velocity = self.scale_velocity(velocities[name])
            param = [DXL_LOBYTE(raw_velocity), DXL_HIBYTE(raw_velocity)]
            self.sync_write.addParam(dxl_id, param)

        result = self.sync_write.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(f"❌ SyncWrite error: {self.packet_handler.getTxRxResult(result)}")

    def scale_velocity(self, velocity_rad_per_sec):
        # Normalize from rad/s to Dynamixel range 0-1023 or 1024-2047
        normalized = velocity_rad_per_sec / MAX_RAD_PER_SEC
        normalized = max(-1.0, min(1.0, normalized))
        scaled = int(normalized * VELOCITY_LIMIT)

        if scaled >= 0:
            return scaled
        else:
            return 1024 + abs(scaled)

    def disable_torque(self):
        for dxl_id in DXL_IDS.values():
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

    def destroy(self):
        self.disable_torque()
        if self.port_handler.isPortOpen():
            self.port_handler.closePort()

# --- Main program ---
def main(args=None):
    rclpy.init(args=args)
    node = MotorWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Keyboard interrupt received, shutting down...")
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
