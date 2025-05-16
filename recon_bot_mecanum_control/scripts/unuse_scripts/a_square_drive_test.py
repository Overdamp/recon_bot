#!/usr/bin/env python3

import os
import platform
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import pandas as pd
import matplotlib.pyplot as plt

# Robot dimensions and wheel radius
Lx = 0.245  # Distance from center to wheel in x-axis (m)
Ly = 0.2    # Distance from center to wheel in y-axis (m)
WHEEL_RADIUS = 0.062  # Radius of the wheels (m)

# Max velocity (in rad/s) for Dynamixel MX-106R at 12V (45 RPM)
MAX_MOTOR_VELOCITY_RPM = 45
MAX_MOTOR_VELOCITY_RAD_S = MAX_MOTOR_VELOCITY_RPM * 2 * math.pi / 60  # rad/s
MAX_MOTOR_VELOCITY_RAD_S_T = 35.52 * 2 * math.pi / 60  # rad/s

class SquareDriveTest(Node):
    def __init__(self):
        super().__init__('square_drive_test')
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize robot's position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.position_log = []
        self.commanded_position_log = [
            {'x': 0.0, 'y': 0.0},
            {'x': 0.0, 'y': 2.0},
            {'x': 2.0, 'y': 2.0},
            {'x': 2.0, 'y': 0.0},
            {'x': 0.0, 'y': 0.0}
        ]

        # Flags to track motion state
        self.current_target_index = 0
        
        # Start the experiment
        self.create_timer(0.1, self.control_loop)

    def odom_callback(self, odom_msg):
        """
        Callback function to update odometry from the Odometry topic.
        """
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y

    def control_loop(self):
        """Control loop to adjust robot movement based on odom feedback."""
        cmd_vel = Twist()

        # Check if we have reached all targets
        if self.current_target_index >= len(self.commanded_position_log):
            self.stop_robot()
            self.get_logger().info("Completed square drive test.")
            
            # Plot the results
            self.plot_trajectory_comparison()
            return

        # Get current target position
        target = self.commanded_position_log[self.current_target_index]
        
        # Calculate distance to the target
        error_x = target['x'] - self.x
        error_y = target['y'] - self.y

        # Calculate control output for velocity
        distance = math.sqrt(error_x**2 + error_y**2)
        
        if distance > 0.05:  # Threshold to determine if the robot has reached the target
            # Set velocity towards the target
            angle_to_target = math.atan2(error_y, error_x)

            # Reduce speed as robot gets closer to target
            velocity_factor = min(1, distance / 2.0)  # Slow down when closer to target
            cmd_vel.linear.x = velocity_factor * MAX_MOTOR_VELOCITY_RAD_S_T * WHEEL_RADIUS * math.cos(angle_to_target)
            cmd_vel.linear.y = velocity_factor * MAX_MOTOR_VELOCITY_RAD_S_T * WHEEL_RADIUS * math.sin(angle_to_target)
        else:
            # If the robot is close enough to the target, stop and move to the next target
            self.current_target_index += 1

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)
        self.position_log.append({'x': self.x, 'y': self.y})

    def stop_robot(self):
        """Stop the robot."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def plot_trajectory_comparison(self):
        """Plot the commanded and actual trajectory comparison."""
        df_actual = pd.DataFrame(self.position_log)
        df_commanded = pd.DataFrame(self.commanded_position_log)

        plt.figure()
        plt.plot(df_commanded['x'], df_commanded['y'], label='Commanded Trajectory', linestyle='--', color='r', marker='o')
        plt.plot(df_actual['x'], df_actual['y'], label='Actual Trajectory', linestyle='-', color='b', marker='x')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Commanded vs Actual Robot Trajectory (Top-Down View)')
        plt.legend()
        plt.show()

        # Save data to Excel and calculate errors
        self.save_data_to_excel(df_actual, df_commanded)

    def save_data_to_excel(self, df_actual, df_commanded):
        """Save the actual positions to an Excel file and open it, including error calculation."""
        # Define the directory where you want to save the file
        directory = os.path.expanduser("~/Documents/recon_bot_result")
        if not os.path.exists(directory):
            os.makedirs(directory)  # Create the directory if it doesn't exist

        file_name = os.path.join(directory, "actual_positions1.xlsx")
        try:
            with pd.ExcelWriter(file_name, engine='openpyxl') as writer:
                # Record actual positions into the table
                df_actual = pd.DataFrame(self.position_log)
                df_actual.to_excel(writer, sheet_name='Actual Positions', index=False)
                # Adding commanded positions to the same Excel file
                df_commanded.to_excel(writer, sheet_name='Commanded Positions', index=False)

                # Calculate the percent error between commanded and actual positions
                epsilon = 1e-5  # A small value to avoid division by zero
                df_errors = pd.DataFrame()
                df_errors['Commanded X'] = df_commanded['x']
                df_errors['Commanded Y'] = df_commanded['y']
                df_errors['Actual X'] = df_actual['x']
                df_errors['Actual Y'] = df_actual['y']
                df_errors['Error X (%)'] = ((df_errors['Commanded X'] - df_errors['Actual X']).abs() / (df_errors['Commanded X'].abs() + epsilon)) * 100
                df_errors['Error Y (%)'] = ((df_errors['Commanded Y'] - df_errors['Actual Y']).abs() / (df_errors['Commanded Y'].abs() + epsilon)) * 100

                # Save error data to a new sheet
                df_errors.to_excel(writer, sheet_name='Position Errors', index=False)

            self.get_logger().info(f"Actual, commanded positions, and errors saved to {file_name}")

            # Open the file based on the operating system
            if platform.system() == "Windows":
                os.startfile(file_name)  # Windows
            elif platform.system() == "Darwin":
                os.system(f"open {file_name}")  # macOS
            else:
                os.system(f"xdg-open {file_name}")  # Linux
        except Exception as e:
            self.get_logger().error(f"Failed to save or open the Excel file: {e}")


def main(args=None):
    rclpy.init(args=args)
    square_drive_test = SquareDriveTest()
    rclpy.spin(square_drive_test)
    square_drive_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
