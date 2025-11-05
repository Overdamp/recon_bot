import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = "recon_bot"
    package_name = f"{robot_name}_description"

    # Define the paths for various files
    rviz_config = os.path.join(get_package_share_directory(package_name), "rviz", f"{robot_name}config.rviz")
    robot_description = os.path.join(get_package_share_directory(package_name), "robot", f"{robot_name}.urdf2.xacro")
    robot_description_config = xacro.process_file(robot_description)

    # Define controller configuration file path
    controller_config = os.path.join(get_package_share_directory(package_name), "config", "mecanum_controllers.yaml")

    # Node configurations for robot_state_publisher and controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_config.toxml()},
            controller_config
        ],
        output={"stdout": "screen", "stderr": "screen"},
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    velocity_controller_node = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controller", "-c", "/controller_manager"],
        output="screen"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_config.toxml()}],
        remappings=[('joint_states', '/joint_states')],
        output="screen"
    )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=["-d", rviz_config],
    #     output={"stdout": "screen", "stderr": "log"}
    # )

    # Return the LaunchDescription with all the nodes inside a list
    return LaunchDescription([
        controller_manager_node,
        joint_state_broadcaster_node,
        velocity_controller_node,
        robot_state_publisher_node,
        # rviz_node,
    ])
