#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
import math

# Robot dimensions and wheel radius
Lx = 0.245  # Distance from center to wheel in x-axis (m)
Ly = 0.2    # Distance from center to wheel in y-axis (m)
WHEEL_RADIUS = 0.062  # Radius of the wheels (m)

# Max velocity (in rad/s) for Dynamixel MX-106R at 12V (45 RPM)
MAX_MOTOR_VELOCITY_RPM = 45.0
MAX_MOTOR_VELOCITY_RAD_S = MAX_MOTOR_VELOCITY_RPM * 2 * math.pi / 60  # rad/s


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_pub')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Initialize robot's position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info("Odometry Publisher Initialized")

    def joint_state_callback(self, joint_state_msg):
        # Try to extract the wheel speeds from JointState message
        try:
            speed_fl, speed_fr, speed_rl, speed_rr = joint_state_msg.velocity
        except Exception as e:
            #self.get_logger().error(f"Error extracting wheel speeds: {e}")
            return

        # Convert the motor speeds from Dynamixel units to rad/s
        speed_fl_rad_s = self.convert_dynamixel_speed_to_radians(speed_fl)
        speed_fr_rad_s = self.convert_dynamixel_speed_to_radians(speed_fr*-1)
        speed_rl_rad_s = self.convert_dynamixel_speed_to_radians(speed_rl)
        speed_rr_rad_s = self.convert_dynamixel_speed_to_radians(speed_rr*-1)

        # Debug: Log wheel speeds after conversion
        #self.get_logger().info(f"Wheel Speeds (rad/s): FL={speed_fl_rad_s}, FR={speed_fr_rad_s}, RL={speed_rl_rad_s}, RR={speed_rr_rad_s}")

        # Compute body velocities using inverse kinematics for mecanum wheels
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Compute time delta in seconds
        self.last_time = current_time

        # Body velocities based on wheel velocities (inverse kinematics)
        vx = (speed_fl_rad_s + speed_fr_rad_s + speed_rl_rad_s + speed_rr_rad_s) * WHEEL_RADIUS / 4.0
        vy = (-speed_fl_rad_s + speed_fr_rad_s + speed_rl_rad_s - speed_rr_rad_s) * WHEEL_RADIUS / 4.0
        omega = (-speed_fl_rad_s + speed_fr_rad_s - speed_rl_rad_s + speed_rr_rad_s) * WHEEL_RADIUS / (4.0 * (Lx + Ly))

        # Update odometry using the velocity estimates and time delta
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Debug: Log position updates
       # self.get_logger().info(f"Updated Odometry: x={self.x}, y={self.y}, theta={self.theta}")

        # Publish odometry
        self.publish_odometry()

    def convert_dynamixel_speed_to_radians(self, motor_velocity):
        """Converts motor velocity from Dynamixel units to rad/s"""
        # Convert motor value to rad/s, assuming max value (1023) corresponds to max motor speed
        return motor_velocity /300 * MAX_MOTOR_VELOCITY_RAD_S

    def publish_odometry(self):
        current_time = self.get_clock().now()
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Fill in the odometry message with position and orientation data
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # Convert theta to quaternion for orientation
        odom_quat = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = odom_quat[0]
        odom_msg.pose.pose.orientation.y = odom_quat[1]
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # Publish the transform for TF
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x = odom_quat[0]
        transform.transform.rotation.y = odom_quat[1]
        transform.transform.rotation.z = odom_quat[2]
        transform.transform.rotation.w = odom_quat[3]
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)

    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
