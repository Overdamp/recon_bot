#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from dynamixel_sdk import *
from dxl_address import *
import math

DXL_IDS = {
    'FL': 4,
    'FR': 3,
    'RL': 2,
    'RR': 1,
}

DIR = {
    'FL': 1,
    'FR': -1,
    'RL': 1,
    'RR': -1,
}

# Constants
WHEEL_RADIUS = 0.062
Lx = 0.245
Ly = 0.2
VELOCITY_LIMIT = 300
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
max_rpm = 4.714  # Dynamixel maximum RPM (if set limit)
TICKS_PER_REVOLUTION = 4096  # MX-106 encoder resolution

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_MX_MOVING_SPEED, 2)
        self.sync_read = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_MX_PRESENT_SPEED, 2)

        if not self.port_handler.openPort():
            self.get_logger().fatal('Failed to open port')
            return
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().fatal('Failed to set baudrate')
            return

        self.last_positions = {name: 0.0 for name in DXL_IDS.keys()}
        self.last_time = self.get_clock().now()

        for name, dxl_id in DXL_IDS.items():
            self.enable_torque(dxl_id)
            self.set_wheel_mode(dxl_id)
            self.sync_read.addParam(dxl_id)

        self.create_timer(0.02, self.read_publish_joint_states)  # 50Hz update

    def enable_torque(self, dxl_id):
        self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_wheel_mode(self, dxl_id):
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_CW_ANGLE_LIMIT, 0)
        self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, ADDR_MX_CCW_ANGLE_LIMIT, 0)

    def cmd_vel_callback(self, msg):
        vx, vy, omega = msg.linear.x, msg.linear.y, msg.angular.z
        lxly = Lx + Ly

        velocities = {
            'FL': (vx - vy - omega * lxly) / WHEEL_RADIUS * DIR['FL'],
            'FR': (vx + vy + omega * lxly) / WHEEL_RADIUS * DIR['FR'],
            'RL': (vx + vy - omega * lxly) / WHEEL_RADIUS * DIR['RL'],
            'RR': (vx - vy + omega * lxly) / WHEEL_RADIUS * DIR['RR'],
        }

        self.sync_write.clearParam()
        for name, dxl_id in DXL_IDS.items():
            velocity = int((velocities[name] / max_rpm) * VELOCITY_LIMIT)
            velocity = max(-VELOCITY_LIMIT, min(velocity, VELOCITY_LIMIT))
            if velocity >= 0:
                value = velocity
            else:
                value = 1024 + abs(velocity)
            param = [DXL_LOBYTE(value), DXL_HIBYTE(value)]
            self.sync_write.addParam(dxl_id, param)

        self.sync_write.txPacket()

    def read_publish_joint_states(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.sync_read.txRxPacket() != COMM_SUCCESS:
            self.get_logger().error('Failed to sync read')
            return

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'Mobile_Base'
        msg.name = ['J_Wheel_LF', 'J_Wheel_RF', 'J_Wheel_LR', 'J_Wheel_RR']
        msg.velocity = []
        msg.position = []

        for name, dxl_id in DXL_IDS.items():
            if self.sync_read.isAvailable(dxl_id, ADDR_MX_PRESENT_SPEED, 2):
                raw_speed = self.sync_read.getData(dxl_id, ADDR_MX_PRESENT_SPEED, 2)

                # Convert Dynamixel speed to rad/s
                if raw_speed > 1023:
                    raw_speed = -(raw_speed - 1024)
                wheel_rpm = raw_speed * max_rpm / VELOCITY_LIMIT
                wheel_rad_per_sec = wheel_rpm * 2 * math.pi / 60.0

                # Integrate position
                self.last_positions[name] += wheel_rad_per_sec * dt

                msg.velocity.append(wheel_rad_per_sec)
                msg.position.append(self.last_positions[name])
            else:
                msg.velocity.append(0.0)
                msg.position.append(self.last_positions[name])

        self.joint_state_pub.publish(msg)

    def destroy(self):
        for dxl_id in DXL_IDS.values():
            self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.port_handler.closePort()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down by user')
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
