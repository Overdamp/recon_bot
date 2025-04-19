import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'recon_bot_bringup'

    # Include launch files for the robot state publisher and rplidar
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )])
    )

    rplidar_two = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rplidar_two.launch.py'
        )])
    )

    # Twist mux node
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/mecanum_cont/cmd_vel_unstamped')]
    )

    # SLAM Toolbox node
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",  # เลือกโหมดที่คุณต้องการใช้
        name="slam_toolbox",
        parameters=[os.path.join(get_package_share_directory(package_name), 'config', 'slam_toolbox.yaml')],
        remappings=[('/scan', '/filtered_front_scan')]  # เปลี่ยนตาม topic ของ lidar ที่คุณใช้
    )

    # Additional nodes for lidar filtering, motor control, odometry, and joystick control
    lidar_filtered_node = Node(
        package="rplidar_ros",
        executable="lidar_filtered.py"
    )

    motor_control_node = Node(
        package="recon_bot_mecanum_control",
        executable="a_motor_read_write.py"
    )

    odometry_node = Node(
        package="recon_bot_mecanum_control",
        executable="a_odometry_pub.py"
    )

    joy_cmd_vel_node = Node(
        package="recon_bot_mecanum_control",
        executable="a_joy_cmd_vel.py"
    )

    # Joystick node
    joy_node = Node(
        package="joy",
        executable="joy_node"
    )

    # Return the launch description with all the nodes
    return LaunchDescription([
        rplidar_two,
        rsp,
        twist_mux,
        slam_toolbox,  # SLAM toolbox
        lidar_filtered_node,  # Lidar filtering node
        motor_control_node,  # Motor control node
        odometry_node,  # Odometry publishing node
        joy_cmd_vel_node,  # Joystick command velocity node
    ])
