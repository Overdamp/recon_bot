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
                ('lx', 0.245),  # front-back half length
                ('ly', 0.2),    # left-right half width
                ('velocity_limit', 285),  # Dynamixel velocity limit
                ('dxl_id_fl', 4),  # Front Left
                ('dxl_id_fr', 3),  # Front Right
                ('dxl_id_rl', 2),  # Rear Left
                ('dxl_id_rr', 1),  # Rear Right
                ('cmd_vel_timeout', 0.5),  # Timeout for cmd_vel (seconds)
                ('loop_rate', 50.0),  # Control loop rate (Hz)
            ]
        )

        # Robot parameters
        self.r = self.get_parameter('wheel_radius').value
        self.Lx = self.get_parameter('lx').value
        self.Ly = self.get_parameter('ly').value
        self.L = self.Lx + self.Ly  # Sum distances for kinematics
        self.velocity_limit = self.get_parameter('velocity_limit').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.loop_rate = self.get_parameter('loop_rate').value

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

        self.get_logger().info('MecanumBaseController initialized')

    def joint_state_callback(self, msg: JointState):
        """Process joint state messages to compute odometry."""
        positions = {joint: self.last_wheel_positions[joint] for joint in self.joint_names}
        valid_data = True

        # Collect joint positions
        for joint in self.joint_names:
            if joint in msg.name:
                idx = msg.name.index(joint)
                positions[joint] = msg.position[idx]
            else:
                self.get_logger().warn(f"Joint {joint} not found in joint_states, using last known position")
                valid_data = False

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9

        # Validate dt to prevent division by zero or negative time steps
        if dt <= 1e-6:
            self.get_logger().warn(f"Invalid dt: {dt}, skipping odometry update")
            return

        if valid_data:
            # Calculate wheel speeds
            delta_positions = [(positions[joint] - self.last_wheel_positions[joint]) for joint in self.joint_names]
            wheel_speeds = [delta / dt * self.dir_signs[joint] for delta, joint in zip(delta_positions, self.joint_names)]

            # Compute robot velocity
            vx, vy, wz = self.calculate_robot_velocity(wheel_speeds)

            # Integrate pose
            delta_x = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
            delta_y = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
            delta_yaw = wz * dt

            self.x += delta_x
            self.y += delta_y
            self.yaw += delta_yaw

            # Normalize yaw only if necessary
            if abs(self.yaw) > 2 * math.pi:
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            self.publish_odometry(now, vx, vy, wz)

        self.last_wheel_positions = positions
        self.last_time = now

    def cmd_vel_callback(self, msg: Twist):
        """Store cmd_vel and update timestamp."""
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_active = True
        self.get_logger().debug(f"Received cmd_vel: linear.x={msg.linear.x}, linear.y={msg.linear.y}, angular.z={msg.angular.z}")

    def control_loop(self):
        """Main control loop to process cmd_vel and handle timeout."""
        now = self.get_clock().now()
        time_since_last_cmd = (now - self.last_cmd_vel_time).nanoseconds * 1e-9

        # Check for cmd_vel timeout
        if time_since_last_cmd > self.cmd_vel_timeout:
            if self.cmd_vel_active:
                self.get_logger().info("cmd_vel timeout, stopping robot")
                self.cmd_vel_active = False
                self.last_cmd_vel = Twist()  # Reset to zero velocity

        # Process cmd_vel
        vx = self.last_cmd_vel.linear.x
        vy = self.last_cmd_vel.linear.y
        wz = self.last_cmd_vel.angular.z

        # Calculate wheel angular velocities (rad/s)
        wheel_omegas = self.mecanum_inverse_kinematics(vx, vy, wz)

        # Apply direction signs
        wheel_omegas = [w * self.dir_signs[joint] for w, joint in zip(wheel_omegas, self.joint_names)]

        # Convert to Dynamixel velocity units
        dxl_velocities = [self.radps_to_dxl_velocity(w) for w in wheel_omegas]

        # Publish velocities in order: RR, LR, RF, LF
        float_msg = Float64MultiArray()
        float_msg.data = [float(v) for v in dxl_velocities]
        self.get_logger().debug(f"Publishing velocities: {dxl_velocities}")
        self.joint_cmd_pub.publish(float_msg)

    def mecanum_inverse_kinematics(self, vx, vy, wz):
        """Compute wheel angular velocities from robot velocities."""
        r = self.r
        L = self.L

        w_rr = (1 / r) * (vx - vy - L * wz)  # Rear Right
        w_lr = (1 / r) * (vx + vy - L * wz)  # Rear Left
        w_rf = (1 / r) * (vx + vy + L * wz)  # Front Right
        w_lf = (1 / r) * (vx - vy + L * wz)  # Front Left

        return [w_rr, w_lr, w_rf, w_lf]

    def calculate_robot_velocity(self, wheel_speeds):
        """Compute robot velocities from wheel angular velocities."""
        r = self.r
        L = self.L
        w_rr, w_lr, w_rf, w_lf = wheel_speeds

        vx = r / 4 * (w_rr + w_lr + w_rf + w_lf)
        vy = r / 4 * (-w_rr + w_lr + w_rf - w_lf)
        wz = r / (4 * L) * (-w_rr - w_lr + w_rf + w_lf)

        return vx, vy, wz

    def radps_to_dxl_velocity(self, radps):
        """Convert wheel angular velocity (rad/s) to Dynamixel velocity units."""
        rpm = radps * 60 / (2 * math.pi)
        velocity_units = rpm / 0.229  # Approx Dynamixel units per rpm

        # Cap velocity to prevent exceeding motor limits
        if velocity_units > self.velocity_limit:
            self.get_logger().warn(f"Velocity capped at {self.velocity_limit}")
            velocity_units = self.velocity_limit
        elif velocity_units < -self.velocity_limit:
            self.get_logger().warn(f"Velocity capped at {-self.velocity_limit}")
            velocity_units = -self.velocity_limit

        return int(velocity_units)

    def publish_odometry(self, now, vx, vy, wz):
        """Publish odometry and TF transform."""
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'  # World-fixed frame
        odom_msg.child_frame_id = 'base_footprint'  # Robot base frame

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Twist
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz

        # Covariance (placeholders, adjust based on sensor accuracy)
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

        # Publish TF transform
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

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MecanumBaseController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()