#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import tf_transformations
from tf2_ros import TransformBroadcaster
import math

class MecanumBaseController(Node):
    def __init__(self):
        super().__init__('mecanum_base_controller')

        # Declare ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.062),
                ('lx', 0.245),
                ('ly', 0.2),
                ('dxl_id_fl', 4),
                ('dxl_id_fr', 3),
                ('dxl_id_rl', 2),
                ('dxl_id_rr', 1),
                ('cmd_vel_timeout', 1.0),  # Increased to 1.0s to reduce interruptions
                ('loop_rate', 100.0),
                ('enable_axis', -1),
                ('enable_axis_threshold', 0.5),
                ('max_linear_speed', 2.0),
                ('max_angular_speed', 4.0),
                ('zero_cmd_threshold', 0.001),
                ('cmd_vel_debounce', 0.01),
                ('max_wheel_omega', 8.16),
            ]
        )

        # Robot parameters
        self.r = self.get_parameter('wheel_radius').value
        self.Lx = self.get_parameter('lx').value
        self.Ly = self.get_parameter('ly').value
        self.L = self.Lx + self.Ly
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.loop_rate = self.get_parameter('loop_rate').value
        self.enable_axis = self.get_parameter('enable_axis').value
        self.enable_axis_threshold = self.get_parameter('enable_axis_threshold').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.zero_cmd_threshold = self.get_parameter('zero_cmd_threshold').value
        self.cmd_vel_debounce = self.get_parameter('cmd_vel_debounce').value
        self.max_wheel_omega = self.get_parameter('max_wheel_omega').value

        # Motor IDs and direction signs
        self.dxl_ids = {
            'J_Wheel_RR': self.get_parameter('dxl_id_rr').value,
            'J_Wheel_LR': self.get_parameter('dxl_id_rl').value,
            'J_Wheel_RF': self.get_parameter('dxl_id_fr').value,
            'J_Wheel_LF': self.get_parameter('dxl_id_fl').value
        }
        self.dir_signs = {
            'J_Wheel_RR': -1,
            'J_Wheel_LR': 1,
            'J_Wheel_RF': -1,
            'J_Wheel_LF': 1
        }
        self.joint_names = list(self.dxl_ids.keys())

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()
        self.last_wheel_positions = {joint: 0.0 for joint in self.joint_names}

        # Command velocity state
        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_active = False
        self.enable_active = False if self.enable_axis >= 0 else True
        self.torque_enabled = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for control loop
        self.timer = self.create_timer(1.0 / self.loop_rate, self.control_loop)

        # Initialize motors
        self.initialize_motors()

        self.get_logger().info('MecanumBaseController initialized for MX-106(2.0) with dynamixel_hardware')

    def initialize_motors(self):
        for joint, dxl_id in self.dxl_ids.items():
            self.get_logger().info(f"Setting {joint} (ID: {dxl_id}) to Velocity Control Mode")
        self.torque_enabled = True

    def joint_state_callback(self, msg: JointState):
        positions = {joint: self.last_wheel_positions[joint] for joint in self.joint_names}
        valid_data = True

        for joint in self.joint_names:
            if joint in msg.name:
                idx = msg.name.index(joint)
                positions[joint] = msg.position[idx]
            else:
                self.get_logger().warn(f"Joint {joint} not found in joint_states, using last known position")
                valid_data = False

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9

        if dt <= 1e-6:
            self.get_logger().warn(f"Invalid dt: {dt}, skipping odometry update")
            return

        if valid_data:
            delta_positions = [(positions[joint] - self.last_wheel_positions[joint]) for joint in self.joint_names]
            wheel_speeds = [delta / dt * self.dir_signs[joint] for delta, joint in zip(delta_positions, self.joint_names)]
            vx, vy, wz = self.calculate_robot_velocity(wheel_speeds)
            delta_x = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
            delta_y = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
            delta_yaw = wz * dt
            self.x += delta_x
            self.y += delta_y
            self.yaw += delta_yaw
            if abs(self.yaw) > 2 * math.pi:
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            self.publish_odometry(now, vx, vy, wz)

        self.last_wheel_positions = positions
        self.last_time = now

    def cmd_vel_callback(self, msg: Twist):
        now = self.get_clock().now()
        time_since_last = (now - self.last_cmd_vel_time).nanoseconds * 1e-9

        if time_since_last < self.cmd_vel_debounce:
            return

        clipped_x = max(min(msg.linear.x, 1.0), -1.0)
        clipped_y = max(min(msg.linear.y, 1.0), -1.0)
        clipped_z = max(min(msg.angular.z, 1.0), -1.0)

        scaled_x = clipped_x * self.max_linear_speed
        scaled_y = clipped_y * self.max_linear_speed
        scaled_z = clipped_z * self.max_angular_speed

        if (abs(scaled_x) < self.zero_cmd_threshold and
            abs(scaled_y) < self.zero_cmd_threshold and
            abs(scaled_z) < self.zero_cmd_threshold):
            self.last_cmd_vel = Twist()
        else:
            self.last_cmd_vel.linear.x = scaled_x
            self.last_cmd_vel.linear.y = scaled_y
            self.last_cmd_vel.angular.z = scaled_z

        self.last_cmd_vel_time = now
        self.cmd_vel_active = True
        self.get_logger().info(
            f"cmd_vel raw: ({msg.linear.x}, {msg.linear.y}, {msg.angular.z}), "
            f"scaled: ({self.last_cmd_vel.linear.x}, {self.last_cmd_vel.linear.y}, {self.last_cmd_vel.angular.z})"
        )

    def control_loop(self):
        if not self.torque_enabled:
            self.get_logger().warn("Torque not enabled, cannot send velocity commands")
            return

        now = self.get_clock().now()
        time_since_last_cmd = (now - self.last_cmd_vel_time).nanoseconds * 1e-9

        if time_since_last_cmd > self.cmd_vel_timeout:
            self.cmd_vel_active = False
            self.get_logger().info("cmd_vel timeout, maintaining last velocities")

        if self.cmd_vel_active and self.enable_active:
            vx = self.last_cmd_vel.linear.x
            vy = self.last_cmd_vel.linear.y
            wz = self.last_cmd_vel.angular.z
        else:
            # Use last known velocities instead of resetting to zero
            vx = self.last_cmd_vel.linear.x
            vy = self.last_cmd_vel.linear.y
            wz = self.last_cmd_vel.angular.z

        wheel_speeds = self.mecanum_inverse_kinematics(vx, vy, wz)
        wheel_speeds = [max(min(w, self.max_wheel_omega), -self.max_wheel_omega) for w in wheel_speeds]

        dxl_velocities = [w * self.dir_signs[joint] for w, joint in zip(wheel_speeds, self.joint_names)]

        float_msg = Float64MultiArray()
        float_msg.data = [float(v) for v in dxl_velocities]
        self.joint_cmd_pub.publish(float_msg)
        self.get_logger().info(f"Wheel velocities (rad/s): {list(float_msg.data)}")

    def mecanum_inverse_kinematics(self, vx, vy, wz):
        r = self.r
        L = self.L
        w_rr = (1 / r) * (vx - vy + L * wz)
        w_lr = (1 / r) * (vx + vy - L * wz)
        w_rf = (1 / r) * (vx + vy + L * wz)
        w_lf = (1 / r) * (vx - vy - L * wz)
        return [w_rr, w_lr, w_rf, w_lf]

    def calculate_robot_velocity(self, wheel_speeds):
        r = self.r
        L = self.L
        w_rr, w_lr, w_rf, w_lf = wheel_speeds
        vx = r / 4 * (w_rr + w_lr + w_rf + w_lf)
        vy = r / 4 * (-w_rr + w_lr + w_rf - w_lf)  
        wz = r / (4 * L) * (w_rr - w_lr + w_rf - w_lf)  
        return vx, vy, wz


    def publish_odometry(self, now, vx, vy, wz):
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]
        odom_msg.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]
        self.odom_pub.publish(odom_msg)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

    def shutdown(self):
        if self.torque_enabled:
            for joint, dxl_id in self.dxl_ids.items():
                self.get_logger().info(f"Disabling torque for {joint} (ID: {dxl_id})")
            self.torque_enabled = False

def main(args=None):
    rclpy.init(args=args)
    node = MecanumBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()