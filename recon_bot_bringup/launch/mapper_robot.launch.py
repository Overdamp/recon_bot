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

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'params_file': os.path.join(
                get_package_share_directory('recon_bot_slam'),  # <--- ใช้ path ของคุณ
                'config',
                'mapper_params_online_async.yaml'
            ),
            'use_sim_time': 'false'
        }.items()
    )

    # Additional nodes for lidar filtering, motor control, odometry, and joystick control
    lidar_filtered_node = Node(
        package="recon_bot_slam",
        executable="lidar_merge.py",
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
        slam_toolbox_launch,  # SLAM toolbox
        lidar_filtered_node,  # Lidar filtering node
        motor_control_node,  # Motor control node
        odometry_node,  # Odometry publishing node
        joy_cmd_vel_node,  # Joystick command velocity node
    ])
