import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = "recon_bot"
    package_name_bringup = f"{robot_name}_bringup"
    package_name_control = f"{robot_name}_mecanum_control"
    package_name_slam = f"{robot_name}_slam"

    # 1. Mecanum Control (Hardware Interface + Robot State Publisher + URDF2)
    # This launches ros2_control_node, robot_state_publisher, and spawners
    mecanum_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name_control), 'launch', 'mecanum_control.launch.py'
        )]),
        # Pass 'zed' as default, or 'zed_mobile' if you want to match vio_slam exactly
        # launch_arguments={'camera_name': 'zed'}.items() 
    )

    # 2. RPLidar (Front + Back)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name_bringup), 'launch', 'rplidar_two.launch.py'
        )])
    )

    # 3. Lidar Merge (Merge Front + Back scans)
    lidar_merge_node = Node(
        package="recon_bot_slam",
        executable="lidar_merge.py",
        name="lidar_merge",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # 4. Mecanum Controller (Kinematics: cmd_vel -> wheel speeds)
    # Note: This listens to /cmd_vel_mux (output of twist_mux)
    mecanum_controller_node = Node(
        package="recon_bot_mecanum_control",
        executable="mecanum_controller.py",
        name="mecanum_controller",
        output="screen",
        parameters=[{"use_sim_time": False}],
        remappings=[('/cmd_vel', '/cmd_vel_mux')] 
    )

    # 5. Twist Mux (Priority: Joy > Nav > etc.)
    twist_mux_params = os.path.join(get_package_share_directory(package_name_bringup), 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/cmd_vel_mux')]
    )

    return LaunchDescription([
        mecanum_control,
        rplidar_launch,
        lidar_merge_node,
        twist_mux_node,
        mecanum_controller_node,
    ])