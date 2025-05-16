#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from dynamixel_sdk import *
from dxl_address_2 import *
import math

# Motor constants for a Mecanum drive
DXL_ID_FL = 4  # Front Left
DXL_ID_FR = 3  # Front Right
DXL_ID_RL = 2  # Rear Left
DXL_ID_RR = 1  # Rear Right

# --- Robot Physical Constants ---
WHEEL_RADIUS = 0.062
Lx = 0.245
Ly = 0.2
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
max_rpm = 45.0  # Corrected to actual MX-106 max RPM

# Robot physical parameters
Lx = 0.245  # Distance from center to wheel in x-axis (m)
Ly = 0.2    # Distance from center to wheel in y-axis (m)
WHEEL_RADIUS = 0.062  # Radius of the wheels (m)
max_velocity = 0.1758 #max_velocity m/s
max_rpm = 4.714 #max_rpm rad/s  (5.13)

class MotorReadWrite(Node):

    def __init__(self):
        super().__init__('motor_read_write')
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_MX_GOAL_VELOCITY, LEN_MX_GOAL_VELOCITY)
        self.sync_read = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)

    def init_dynamixel(self):
        # Initialize Dynamixel motors
        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open the port')
            return
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to set the baudrate')
            return

        # # Enable torque for each motor
        # for dxl_id in [DXL_ID_FL, DXL_ID_FR, DXL_ID_RL, DXL_ID_RR]:
        #     self.enable_torque(dxl_id)

        # # Set position mode for each motor and move them to position 0
        # for dxl_id in [DXL_ID_FL, DXL_ID_FR, DXL_ID_RL, DXL_ID_RR]:
        #     self.set_position_mode(dxl_id)
        #     self.set_motor_position(dxl_id, 0)

        # # Wait for motors to reach position 0
        # self.wait_until_reach_position()

        # # After reaching position 0, set motors back to wheel mode for normal operation
        # for dxl_id in [DXL_ID_FL, DXL_ID_FR, DXL_ID_RL, DXL_ID_RR]:
        #     self.set_wheel_mode(dxl_id)

    def enable_torque(self, dxl_id):
        # Enable torque for the given motor
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to enable torque for motor {dxl_id}: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            self.get_logger().error(f"Motor {dxl_id} returned error while enabling torque: {self.packet_handler.getRxPacketError(error)}")
        else:
            self.get_logger().info(f"Torque enabled for motor {dxl_id}")

    def set_wheel_mode(self, dxl_id):
        # Explicitly disable CW/CCW angle limits to enter wheel mode
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
            # Convert velocity [rad/s] -> RPM -> Raw unit for Protocol 2.0 (0.229 rpm/unit)
            rpm = velocities[name] * 60.0 / (2 * math.pi)
            raw_value = int(rpm / 0.229)
            if raw_value < 0:
                raw_value = (1 << 32) + raw_value  # convert to unsigned 32-bit

            param = [
                DXL_LOBYTE(DXL_LOWORD(raw_value)),
                DXL_HIBYTE(DXL_LOWORD(raw_value)),
                DXL_LOBYTE(DXL_HIWORD(raw_value)),
                DXL_HIBYTE(DXL_HIWORD(raw_value))
            ]
            self.sync_write.addParam(dxl_id, param)

        result = self.sync_write.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to set position for motor {dxl_id}: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            self.get_logger().error(f"Motor {dxl_id} returned error while setting position: {self.packet_handler.getRxPacketError(error)}")

    def wait_until_reach_position(self):
        # Wait until all motors reach the target position (0)
        while True:
            all_reached = True
            for dxl_id in [DXL_ID_FL, DXL_ID_FR, DXL_ID_RL, DXL_ID_RR]:
                current_position = self.read_motor_position(dxl_id)
                if abs(current_position - 0) > 10:  # Adjust the tolerance to be smaller, e.g., 2
                    all_reached = False
                    break
            if all_reached:
                break
            time.sleep(0.1)


    def read_motor_position(self, motor_id):
        # Read current position of a motor
        position, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, ADDR_MX_PRESENT_POSITION)
        return position

    def disable_torque(self):
        # Disable torque for each motor when shutting down
        for dxl_id in [DXL_ID_FL, DXL_ID_FR, DXL_ID_RL, DXL_ID_RR]:
            result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            if result != COMM_SUCCESS:
                self.get_logger().error(f"Failed to disable torque for motor {dxl_id}: {self.packet_handler.getTxRxResult(result)}")
            elif error != 0:
                self.get_logger().error(f"Motor {dxl_id} returned error while disabling torque: {self.packet_handler.getRxPacketError(error)}")
            else:
                self.get_logger().info(f"Torque disabled for motor {dxl_id}")

    def cmd_vel_callback(self, twist_msg):
        # Convert velocity commands to motor speeds using mecanum wheel kinematics
        vx = twist_msg.linear.x
        vy = twist_msg.linear.y
        omega = twist_msg.angular.z
        
        # Calculate velocities for each wheel
        v_fl, v_fr, v_rl, v_rr = self.calculate_wheel_velocities(vx, vy, omega)
        # Write to motors
        self.set_wheel_velocity(DXL_ID_FL, self.convert_velocity(v_fl))
        self.set_wheel_velocity(DXL_ID_FR, self.convert_velocity(v_fr))
        self.set_wheel_velocity(DXL_ID_RL, self.convert_velocity(v_rl))
        self.set_wheel_velocity(DXL_ID_RR, self.convert_velocity(v_rr))

    def calculate_wheel_velocities(self, vx, vy, omega):
        # Kinematic equations for mecanum wheels (adjust for correct directions)
        vel_fl = (vx - vy - omega * (Lx + Ly) ) / WHEEL_RADIUS * DIR_FL
        vel_fr = (vx + vy + omega * (Lx + Ly) ) / WHEEL_RADIUS * DIR_FR
        vel_rl = (vx + vy - omega * (Lx + Ly) ) / WHEEL_RADIUS * DIR_RL
        vel_rr = (vx - vy + omega * (Lx + Ly) ) / WHEEL_RADIUS * DIR_RR
        return vel_fl, vel_fr, vel_rl, vel_rr

    def set_wheel_velocity(self, dxl_id, velocity):
        # Dynamixel motors need velocity to be in the form of a scaled value, so convert accordingly
        velocity = max(-VELOCITY_LIMIT, min(velocity, VELOCITY_LIMIT))  # Limit velocity to motor max
        if velocity >= 0:
            # Forward: map 0-1023 directly to Dynamixel's 0-1023
            velocity_value = velocity
        elif velocity <= 0:
            # Reverse: map -1023-0 to Dynamixel's 1024-2047
            velocity_value = 1024 + abs(velocity)
        else:
            velocity_value = 0

        # Log the velocity being commanded to the motor
        self.get_logger().info(f"Setting motor {dxl_id} to velocity: {velocity_value} (original: {velocity})")

        result = self.sync_read.txRxPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to set speed for motor {dxl_id}: {self.packet_handler.getTxRxResult(result)}")
        elif error != 0:
            self.get_logger().error(f"Motor {dxl_id} returned error while setting speed: {self.packet_handler.getRxPacketError(error)}")

    def convert_velocity(self, velocity):
        # Scale the velocity from m/s to motor value (e.g., scaling factor is based on motor specification)
        motor_velocity = (velocity/max_rpm) * 300  # Example scaling factor, adjust for your motor
        return int(motor_velocity)

        for name, dxl_id in DXL_IDS.items():
            if self.sync_read.isAvailable(dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY):
                raw_velocity = self.sync_read.getData(dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)
                if raw_velocity > (1 << 31):
                    raw_velocity -= (1 << 32)

                rpm = raw_velocity * 0.229  # unit = 0.229 rpm
                wheel_rad_per_sec = rpm * 2 * math.pi / 60.0

        # Set the velocities for each joint
        joint_state_msg.velocity = [
            float(speed_fl),
            float(speed_fr),
            float(speed_rl),
            float(speed_rr)
        ]

        # Set the positions for each joint
        joint_state_msg.position = [
            float(pos_fl),
            float(pos_fr),
            float(pos_rl),
            float(pos_rr)
        ]

        # Publish the joint state
        self.joint_state_pub.publish(joint_state_msg)

    def read_motor_state(self, motor_id):
        try:
            # Read current speed and position of a motor
            speed, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, ADDR_MX_PRESENT_SPEED)
            position, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, ADDR_MX_PRESENT_POSITION)

            if speed is None or position is None:
                self.get_logger().error(f"Failed to read state for motor {motor_id}")
                return 0.0, 0.0

            # Convert reverse speed if necessary
            if speed > 1023:
                speed = -(speed - 1024)

            return speed, position
        except Exception as e:
            self.get_logger().error(f"Error reading motor {motor_id}: {e}")
            return 0.0, 0.0


    def destroy(self):
        # Disable torque before shutting down
        self.disable_torque()
        self.port_handler.closePort()


def main(args=None):
    rclpy.init(args=args)
    motor_read_write = MotorReadWrite()
    try:
        rclpy.spin(motor_read_write)
    except KeyboardInterrupt:
        motor_read_write.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        motor_read_write.destroy()
        motor_read_write.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()