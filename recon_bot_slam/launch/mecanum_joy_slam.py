import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = "recon_bot"
    package_name = f"{robot_name}_description"
    package_name_control = f"{robot_name}_mecanum_control"
    package_name_bringup = f"{robot_name}_bringup"

    # Define the paths for various files
    rviz_config = os.path.join(get_package_share_directory(package_name), "rviz", f"{robot_name}config.rviz")
    robot_description = os.path.join(get_package_share_directory(package_name), "robot", f"{robot_name}.urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    # Define controller configuration file path
    # controller_config = os.path.join(get_package_share_directory(package_name), "config", "mecanum_controllers.yaml")

    mecanum_joy_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name_control), 'launch', 'mecanum_control.launch.py'
        )])
    )

    mecanum_rplidar_merge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name_bringup), 'launch', 'rplidar_two.launch.py'
        )])
    )

    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     name="joy_node",
    #     output="screen",
    #     parameters=[{"use_sim_time": False}],
    # )

    joy_cmd_vel_node = Node(
        package="recon_bot_mecanum_control",
        executable="a_joy_cmd_vel.py",
        name="joy_cmd_vel",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    mecanum_lidar_node = Node(
        package="recon_bot_slam",
        executable="lidar_merge.py",
        name="lidar_merge",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    mecanum_control_node = Node(
        package="recon_bot_mecanum_control",
        executable="mecanum_controller.py",
        name="mecanum_controller",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output={"stdout": "screen", "stderr": "log"}
    )

    # Return the LaunchDescription with all the nodes inside a list
    return LaunchDescription([
        # joy_node,
        joy_cmd_vel_node,
        mecanum_rplidar_merge,
        mecanum_lidar_node,
        mecanum_control_node,
        mecanum_joy_control,
        # rviz_node,   
    ])
