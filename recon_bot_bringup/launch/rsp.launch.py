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

    # Process the xacro file to generate the robot description
    robot_description = xacro.process_file(xacro_file_path).toxml()

    # Include the ZED camera launch file, provided by the bringup package
    package_name_zed_wrapper = 'zed_wrapper'  # <--- CHANGE ME IF NECESSARY
    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name_zed_wrapper), 'launch', 'zed_camera.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false', 'camera_model': 'zed2i'}.items()
    )

    # Use robot state publisher to publish the robot description to the parameter server
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    # Joint State Publisher Node (for manual joint state publication) with event handler to ensure robot_state_publisher is ready
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    delayed_joint_state_publisher = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[joint_state_publisher]
        )
    )

    # RViz Node with delay to ensure all other nodes are ready
    rviz_file_name = 'recon_bot_config_rsp.rviz'
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

    # Launch Description
    launch_description = LaunchDescription([
        zed_camera_launch,
        robot_state_publisher,
        # delayed_joint_state_publisher,
        # delayed_rviz
    ])

    return launch_description


def main(args=None):
    generate_launch_description()


if __name__ == '__main__':
    main()
