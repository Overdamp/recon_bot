#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
import xacro


def generate_launch_description():
    # Define the package and file path for the URDF/Xacro file
    package_name = 'recon_bot_description'  # <--- CHANGE ME IF NECESSARY
    xacro_file_path = os.path.join(
        get_package_share_directory(package_name),
        'robot',
        'robot.urdf.xacro'
    )
    
    # RViz Node with delay to ensure all other nodes are ready
    rviz_file_name = 'recon_bot_config_monitor.rviz'
    rviz_file_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        rviz_file_name
    )
    delayed_rviz = TimerAction(
        period=10.0,  # Delay to ensure ZED and robot state publisher are fully initialized
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=['-d', rviz_file_path],
                output='screen'
            )
        ]
    )

    # Joystick node
    joy_node = Node(
        package="joy",
        executable="joy_node"
    )

    # Launch Description
    launch_description = LaunchDescription([
        delayed_rviz,
        joy_node,
    ])

    return launch_description


def main(args=None):
    generate_launch_description()


if __name__ == '__main__':
    main()
