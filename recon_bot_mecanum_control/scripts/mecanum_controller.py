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
import traceback # Added for better error reporting
import time # Added for shutdown sleep
import threading # Added for joint state lock

class MecanumBaseController(Node):
    def __init__(self):
        super().__init__('mecanum_base_controller')

        # Declare ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.062),
                ('lx', 0.245), # Half the distance between front/back wheels along x
                ('ly', 0.2),  # Half the distance between left/right wheels along y
                ('dxl_id_fl', 4),
                ('dxl_id_fr', 3),
                ('dxl_id_rl', 2),
                ('dxl_id_rr', 1),
                ('cmd_vel_timeout', 1.0),  # Safety timeout in seconds
                ('loop_rate', 100.0),      # Control loop frequency
                ('enable_axis', -1),      # Joystick axis for enabling motion (optional)
                ('enable_axis_threshold', 0.5),
                ('max_linear_speed', 0.25), # Max robot speed m/s (Limited by MX-106R ~45RPM)
                ('max_angular_speed', 0.6),# Max robot rot speed rad/s
                ('zero_cmd_threshold', 0.001), # Treat cmd_vel below this as zero
                ('cmd_vel_debounce', 0.01),   # Ignore cmd_vel faster than this
                ('max_wheel_omega', 4.8),    # Max wheel angular velocity rad/s (45RPM ~ 4.71 rad/s)
            ]
        )

        # Robot parameters
        self.r = self.get_parameter('wheel_radius').value
        self.Lx = self.get_parameter('lx').value
        self.Ly = self.get_parameter('ly').value
        # IMPORTANT: Check which definition of L your kinematics formula uses.
        # Often it's Lx + Ly for mecanum. Verify this matches your formula source.
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

        # Basic Parameter Validation
        if self.r <= 0:
            self.get_logger().fatal("Parameter 'wheel_radius' must be positive.")
            rclpy.shutdown()
            return
        if self.L <= 0: # Lx and Ly should be positive for L = Lx + Ly
             self.get_logger().fatal("Parameters 'lx' and 'ly' must be positive.")
             rclpy.shutdown()
             return


        # Motor IDs and direction signs (CRITICAL: Verify these match hardware and URDF)
        self.dxl_ids = {
            'J_Wheel_RR': self.get_parameter('dxl_id_rr').value, # ID 1
            'J_Wheel_LR': self.get_parameter('dxl_id_rl').value, # ID 2
            'J_Wheel_RF': self.get_parameter('dxl_id_fr').value, # ID 3
            'J_Wheel_LF': self.get_parameter('dxl_id_fl').value  # ID 4
        }
        self.dir_signs = { # Adjust signs if wheels spin opposite to desired direction
            'J_Wheel_RR': -1,
            'J_Wheel_LR': 1,
            'J_Wheel_RF': -1,
            'J_Wheel_LF': 1
        }
        # Define the EXACT order expected by the /velocity_controller/commands topic
        self.joint_names = ['J_Wheel_RR', 'J_Wheel_LR', 'J_Wheel_RF', 'J_Wheel_LF']

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = self.get_clock().now()
        self.last_wheel_positions = {joint: 0.0 for joint in self.joint_names}
        self.latest_joint_state = None # Store latest joint state msg
        self.joint_state_lock = threading.Lock() # Protect access

        # Command velocity state
        self.last_cmd_vel = Twist() # Stores the most recent valid command
        self.last_cmd_vel_receive_time = self.get_clock().now() # Time of last cmd_vel msg
        self.cmd_vel_active = False # Flag indicating if cmd_vel is recent

        # Enable button logic (optional)
        self.enable_active = False if self.enable_axis >= 0 else True

        # === Subscribers ===
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for control loop
        self.timer = self.create_timer(1.0 / self.loop_rate, self.control_loop)
        self.last_log_time = self.get_clock().now() # For throttling logs

        self.get_logger().info('MecanumBaseController initialized.')
        # Torque is managed by ros2_control lifecycle

    # Removed initialize_motors as torque is handled externally

    def joint_state_callback(self, msg: JointState):
        """Processes joint state messages to calculate odometry."""
        with self.joint_state_lock:
             self.latest_joint_state = msg # Store the latest message

        # Try to read current positions from message
        current_positions = dict(self.last_wheel_positions) # Start with last known values
        msg_joint_map = {name: i for i, name in enumerate(msg.name)} # Map names to indices

        valid_data_count = 0
        for joint in self.joint_names:
            if joint in msg_joint_map:
                idx = msg_joint_map[joint]
                # Check if position array is long enough
                if idx < len(msg.position):
                    current_positions[joint] = msg.position[idx]
                    valid_data_count += 1
                else:
                    self.get_logger().warn(f"Position data missing for joint {joint} in joint_states msg.", throttle_duration_sec=5)
            else:
                 self.get_logger().warn(f"Joint {joint} not found in joint_states msg.", throttle_duration_sec=5)

        # Only proceed if we have reasonably complete data
        if valid_data_count < len(self.joint_names):
             self.get_logger().warn("Incomplete joint data, skipping odometry update.", throttle_duration_sec=5)
             # Update time but keep old positions
             self.last_odom_time = self.get_clock().now()
             return


        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds * 1e-9

        # Basic check for valid time difference
        if dt <= 1e-6:
            # self.get_logger().warn(f"Very small or negative dt ({dt:.4f}s), skipping odom update.") # Can be spammy
            return

        delta_positions = []
        wheel_speeds_actual = [] # Actual speeds calculated from position difference
        try:
            for joint in self.joint_names:
                # Calculate change in position
                delta = current_positions[joint] - self.last_wheel_positions[joint]

                # --- FIX: Handle angle wrap-around ---
                # If delta is larger than pi, it means the wheel crossed the +/- pi boundary
                if abs(delta) > math.pi:
                    delta -= math.copysign(2 * math.pi, delta) # Subtract or add 2*pi
                # --- END FIX ---

                delta_positions.append(delta)
                # Calculate actual wheel speed applying direction sign for kinematics convention
                wheel_speeds_actual.append(delta / dt * self.dir_signs.get(joint, 1.0)) # Use get for safety

            # Calculate robot velocity using Forward Kinematics
            vx, vy, wz = self.calculate_robot_velocity(wheel_speeds_actual)

            # Integrate pose using calculated velocities
            # (Consider more advanced integration like Runge-Kutta if needed, but Euler is common)
            delta_x = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
            delta_y = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
            delta_yaw = wz * dt

            self.x += delta_x
            self.y += delta_y
            self.yaw += delta_yaw

            # Normalize yaw to keep it within [-pi, pi]
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            # Publish the calculated odometry
            self.publish_odometry(now, vx, vy, wz)

        except ZeroDivisionError:
             self.get_logger().warn("Division by zero in odometry calculation (dt likely zero). Skipping update.")
        except KeyError as e:
             self.get_logger().error(f"Kinematics error: Joint '{e}' not found in dir_signs. Check joint_names.")
        except Exception as e:
             self.get_logger().error(f"Unexpected error calculating odometry: {e}")
             # traceback.print_exc() # Uncomment for detailed stack trace


        # Update state for the next iteration
        self.last_wheel_positions = current_positions
        self.last_odom_time = now

    def cmd_vel_callback(self, msg: Twist):
        """Processes incoming velocity commands."""
        now = self.get_clock().now()
        time_since_last = (now - self.last_cmd_vel_receive_time).nanoseconds * 1e-9

        # Debounce check
        if time_since_last < self.cmd_vel_debounce:
            return # Ignore commands arriving too quickly

        # --- Enable Button Logic (Optional) ---
        # if self.enable_axis >= 0 and self.enable_axis < len(msg.axes): # Check axis exists
        #     if abs(msg.axes[self.enable_axis]) > self.enable_axis_threshold:
        #         self.enable_active = True
        #     else:
        #         self.enable_active = False
        #         # If enable button released, force stop immediately
        #         self.last_cmd_vel = Twist()
        #         self.cmd_vel_active = False # Mark as inactive due to enable release
        #         self.get_logger().info("Enable button released, stopping robot.")
        #         return # Don't process velocity if enable released
        # --- End Enable Button Logic ---


        # Scale velocity commands (assuming input is normalized, e.g., from joystick)
        scaled_x = msg.linear.x * self.max_linear_speed
        scaled_y = msg.linear.y * self.max_linear_speed # Mecanum uses Y
        scaled_z = msg.angular.z * self.max_angular_speed

        # Apply zero threshold AFTER scaling
        is_zero_command = (abs(scaled_x) < self.zero_cmd_threshold and
                           abs(scaled_y) < self.zero_cmd_threshold and
                           abs(scaled_z) < self.zero_cmd_threshold)

        if is_zero_command:
            # If command is effectively zero, store actual zeros
            self.last_cmd_vel.linear.x = 0.0
            self.last_cmd_vel.linear.y = 0.0
            self.last_cmd_vel.angular.z = 0.0
        else:
            # Otherwise, store the scaled values
            self.last_cmd_vel.linear.x = scaled_x
            self.last_cmd_vel.linear.y = scaled_y
            self.last_cmd_vel.angular.z = scaled_z

        # Update status for timeout check
        self.last_cmd_vel_receive_time = now
        self.cmd_vel_active = True # Mark command as active/recent

        # Optional: Log received and resulting stored command
        # self.get_logger().info(f"cmd_vel RX raw: ({msg.linear.x:.2f}, {msg.linear.y:.2f}, {msg.angular.z:.2f}) -> Stored: ({self.last_cmd_vel.linear.x:.2f}, {self.last_cmd_vel.linear.y:.2f}, {self.last_cmd_vel.angular.z:.2f})")

    def log_state_and_command(self, status, vx, vy, wz, dxl_cmds):
        """Helper to log controller state, command, and actual velocity."""
        log_now = self.get_clock().now()
        log_dt = (log_now - self.last_log_time).nanoseconds * 1e-9
        if log_dt > 0.5: # Throttle logging to ~2Hz
            # self.get_logger().info(f"Status: {status} | CmdVel(m/s,r/s): vx={vx:.2f},vy={vy:.2f},wz={wz:.2f}")
            # Format wheel commands nicely
            wheel_cmd_str = ', '.join([f"{cmd:.2f}" for cmd in dxl_cmds])
            self.get_logger().info(f"  -> WheelCmd(r/s): [{wheel_cmd_str}]")

            # Log current joint state velocity
            with self.joint_state_lock:
                 state_to_log = self.latest_joint_state

            if state_to_log:
                log_str = "  -> JS_Vel (r/s): {"
                try:
                    name_list = state_to_log.name
                    vel_list = state_to_log.velocity
                    if name_list and vel_list and len(name_list) == len(vel_list):
                        parts = []
                        for i in range(len(name_list)):
                             # Shorten name and format velocity
                             short_name = name_list[i].replace('J_Wheel_','')
                             parts.append(f"'{short_name}': {vel_list[i]:.2f}")
                        log_str += ', '.join(parts) + "}"
                    else:
                        log_str += " (Malformed msg)}"
                except Exception as e:
                    log_str += f"(Error: {e})}}"
                self.get_logger().info(log_str)
            # else: # Avoid warning spam if no joint state received yet
            #     self.get_logger().warn("  -> No JointState received yet.", throttle_duration_sec=5)

            self.last_log_time = log_now

    def control_loop(self):
        """Main control loop: checks timeout, calculates, commands motors, logs state."""
        now = self.get_clock().now()
        time_since_last_cmd = (now - self.last_cmd_vel_receive_time).nanoseconds * 1e-9
        current_status = "UNKNOWN" # For logging

        # --- Timeout Logic ---
        if time_since_last_cmd > self.cmd_vel_timeout:
            if self.cmd_vel_active: # Log only once when timeout triggers
                self.get_logger().info(f"cmd_vel timeout after {time_since_last_cmd:.2f}s, stopping robot.")
            self.cmd_vel_active = False # Mark as inactive
            # Force zero velocity on timeout
            vx, vy, wz = 0.0, 0.0, 0.0
            current_status = "TIMEOUT"
        elif self.enable_active: # If not timed out and enabled
            vx = self.last_cmd_vel.linear.x
            vy = self.last_cmd_vel.linear.y
            wz = self.last_cmd_vel.angular.z
            current_status = "ACTIVE"
        else: # If not timed out but not enabled
            if self.cmd_vel_active: # Log if active command ignored due to enable=false
                 self.get_logger().warn("Motion command ignored, enable button not pressed.", throttle_duration_sec=2)
                 self.cmd_vel_active = False # Treat as inactive if not enabled
            vx, vy, wz = 0.0, 0.0, 0.0
            current_status = "DISABLED"
        # --- End Timeout Logic ---

        # Calculate target wheel speeds using inverse kinematics
        try:
            wheel_speeds_cmd = self.mecanum_inverse_kinematics(vx, vy, wz)
        except Exception as e:
             self.get_logger().error(f"Error in inverse kinematics: {e}. Sending zero speeds.")
             wheel_speeds_cmd = [0.0] * len(self.joint_names)

        # Apply individual wheel speed limits
        wheel_speeds_cmd = [max(min(w, self.max_wheel_omega), -self.max_wheel_omega) for w in wheel_speeds_cmd]

        # Apply direction signs for hardware interface
        # Ensure the order matches self.joint_names for publishing
        dxl_velocities = [0.0] * len(self.joint_names)
        for i, joint in enumerate(self.joint_names):
             if i < len(wheel_speeds_cmd):
                  # Use get() on dir_signs for safety in case joint name is wrong
                  dxl_velocities[i] = wheel_speeds_cmd[i] * self.dir_signs.get(joint, 1.0)
             else:
                  # This should not happen if kinematics returns correct length
                  self.get_logger().error(f"Index out of bounds mapping wheel speeds for {joint}")

        # Prepare and publish the command message
        float_msg = Float64MultiArray()
        # Ensure data is float, handle potential None values if errors occurred earlier
        float_msg.data = [float(v) if v is not None else 0.0 for v in dxl_velocities]
        self.joint_cmd_pub.publish(float_msg)

        # Log state (throttled)
        self.log_state_and_command(current_status, vx, vy, wz, dxl_velocities)


    def mecanum_inverse_kinematics(self, vx, vy, wz):
        """Calculates target wheel speeds (rad/s) for each joint based on REP 103."""
        if self.r <= 0: raise ValueError("Wheel radius must be positive.")
        # L = Lx + Ly assumes Lx/Ly are half distances. Re-verify this.
        # If Lx is half-length, Ly is half-width, L = Lx + Ly is often used.
        if self.L <= 0: raise ValueError("Robot dimensions (L=Lx+Ly) must be positive.")

        # ROS REP 103 convention (X forward, Y left, Z up)
        # Front Left wheel (Counter-Clockwise is positive rotation)
        w_lf = (1 / self.r) * (vx - vy - self.L * wz)
        # Front Right wheel (Clockwise is positive rotation)
        w_rf = (1 / self.r) * (vx + vy + self.L * wz)
        # Rear Left wheel (Counter-Clockwise is positive rotation)
        w_lr = (1 / self.r) * (vx + vy - self.L * wz)
        # Rear Right wheel (Clockwise is positive rotation)
        w_rr = (1 / self.r) * (vx - vy + self.L * wz)

        # Map to the specific joint names in the required order
        wheel_speeds_map = {
             'J_Wheel_RR': w_rr,
             'J_Wheel_LR': w_lr,
             'J_Wheel_RF': w_rf,
             'J_Wheel_LF': w_lf
        }
        # Return speeds in the order defined by self.joint_names
        # Use get() with default 0.0 for safety
        return [wheel_speeds_map.get(joint, 0.0) for joint in self.joint_names]

    def calculate_robot_velocity(self, wheel_speeds_list_signed):
        """Calculates robot velocity (vx, vy, wz) from actual signed wheel speeds (rad/s)."""
        if self.r <= 0: raise ValueError("Wheel radius must be positive.")
        if self.L <= 0: raise ValueError("Robot dimensions (L=Lx+Ly) must be positive.")
        if len(wheel_speeds_list_signed) != len(self.joint_names):
             raise ValueError(f"Expected {len(self.joint_names)} speeds, got {len(wheel_speeds_list_signed)}")

        # Map speeds back from the list using self.joint_names
        # Note: wheel_speeds_list_signed already includes the dir_signs applied in callback
        speeds = {name: speed for name, speed in zip(self.joint_names, wheel_speeds_list_signed)}

        # Extract speeds using the names defined in self.joint_names
        # Use get() with default 0.0 for safety
        w_lf_signed = speeds.get('J_Wheel_LF', 0.0)
        w_rf_signed = speeds.get('J_Wheel_RF', 0.0)
        w_lr_signed = speeds.get('J_Wheel_LR', 0.0)
        w_rr_signed = speeds.get('J_Wheel_RR', 0.0)

        # Forward Kinematics (should mathematically invert the inverse kinematics)
        # This formula corresponds to the inverse kinematics above (REP 103)
        vx = self.r / 4.0 * ( w_lf_signed + w_rf_signed + w_lr_signed + w_rr_signed)
        vy = self.r / 4.0 * (-w_lf_signed + w_rf_signed + w_lr_signed - w_rr_signed)
        wz = self.r / (4.0 * self.L) * (-w_lf_signed + w_rf_signed - w_lr_signed + w_rr_signed)

        return vx, vy, wz

    def publish_odometry(self, now, vx, vy, wz):
        """Publishes odometry message and TF transform."""
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint' # Standard frame name

        # Pose
        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0 # Planar motion
        try:
            quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
            odom_msg.pose.pose.orientation.x = float(quat[0])
            odom_msg.pose.pose.orientation.y = float(quat[1])
            odom_msg.pose.pose.orientation.z = float(quat[2])
            odom_msg.pose.pose.orientation.w = float(quat[3])
        except Exception as e:
             self.get_logger().error(f"Error converting Euler to Quaternion: {e}")
             # Assign identity quaternion on error for safety
             odom_msg.pose.pose.orientation.w = 1.0; odom_msg.pose.pose.orientation.x = 0.0
             odom_msg.pose.pose.orientation.y = 0.0; odom_msg.pose.pose.orientation.z = 0.0

        # Twist
        odom_msg.twist.twist.linear.x = float(vx)
        odom_msg.twist.twist.linear.y = float(vy)
        odom_msg.twist.twist.angular.z = float(wz)

        # Covariance (Example values - IMPORTANT: Tune these based on observation/testing)
        # Using lists of 36 floats directly
        pose_cov_list = [0.0] * 36
        twist_cov_list = [0.0] * 36

        # Fill diagonal elements (variances) - adjust values as needed
        pose_cov_list[0] = 0.1   # x variance
        pose_cov_list[7] = 0.1   # y variance
        pose_cov_list[14] = 1e-9 # z variance (near zero)
        pose_cov_list[21] = 1e-9 # roll variance (near zero)
        pose_cov_list[28] = 1e-9 # pitch variance (near zero)
        pose_cov_list[35] = 0.2   # yaw variance

        twist_cov_list[0] = 0.05  # vx variance
        twist_cov_list[7] = 0.05  # vy variance
        twist_cov_list[14] = 1e-9 # vz variance (near zero)
        twist_cov_list[21] = 1e-9 # roll rate variance (near zero)
        twist_cov_list[28] = 1e-9 # pitch rate variance (near zero)
        twist_cov_list[35] = 0.1  # yaw rate variance

        odom_msg.pose.covariance = tuple(pose_cov_list) # Assign as tuple
        odom_msg.twist.covariance = tuple(twist_cov_list) # Assign as tuple


        self.odom_pub.publish(odom_msg)

        # TF broadcast
        # t = TransformStamped()
        # t.header.stamp = now.to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_footprint'
        # t.transform.translation.x = float(self.x)
        # t.transform.translation.y = float(self.y)
        # t.transform.translation.z = 0.0
        # t.transform.rotation = odom_msg.pose.pose.orientation # Use the calculated quaternion
        # try:
        #     self.tf_broadcaster.sendTransform(t)
        # except Exception as e:
        #     self.get_logger().error(f"Failed to send TF transform: {e}")


    def shutdown(self):
        """Custom shutdown logic: send zero velocity."""
        self.get_logger().info("Sending final zero velocity command...")
        stop_msg = Twist()
        try:
            # Try publishing stop command for a short duration
            end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=0.2)
            while rclpy.ok() and self.get_clock().now() < end_time:
                 self.cmd_vel_pub.publish(stop_msg)
                 time.sleep(0.05) # Small delay between publishes
        except Exception as e:
             self.get_logger().warn(f"Exception during final stop command publish: {e}")

        # Torque disable is handled by ros2_control lifecycle
        self.get_logger().info("MecanumBaseController shutting down.")


def main(args=None):
    rclpy.init(args=args)
    node = MecanumBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        # Log any unexpected exceptions
        node.get_logger().error(f"Unhandled exception in main spin: {e}")
        traceback.print_exc() # Print detailed traceback
    finally:
        # Ensure shutdown sequence runs even on errors
        node.shutdown()
        if node.is_valid(): # Check if node wasn't already destroyed
             node.destroy_node()
        # Check if RCLpy context is still valid before shutting down
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()