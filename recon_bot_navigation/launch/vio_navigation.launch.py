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
    package_name_slam = f"{robot_name}_slam"
    package_name_nav = f"{robot_name}_navigation"

    # Define the paths for various files
    ekf_config_path = os.path.join(
        get_package_share_directory(package_name_slam), 
        'config', 
        'ekf_vio.yaml'
    )

    # Hardware: Mecanum Control
    mecanum_joy_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name_control), 'launch', 'mecanum_control.launch.py'
        )]),
        launch_arguments={'camera_name': 'zed_mobile'}.items()
    )

    # Hardware: RPLidar
    mecanum_rplidar_merge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name_bringup), 'launch', 'rplidar_two.launch.py'
        )])
    )

    # Hardware: Joy Cmd Vel
    joy_cmd_vel_node = Node(
        package="recon_bot_mecanum_control",
        executable="a_joy_cmd_vel.py",
        name="joy_cmd_vel",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # Hardware: Lidar Merge
    mecanum_lidar_node = Node(
        package="recon_bot_slam",
        executable="lidar_merge.py",
        name="lidar_merge",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # Twist Mux
    twist_mux_params = os.path.join(get_package_share_directory(package_name_bringup), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/cmd_vel_mux')] # Output of mux
    )

    # Hardware: Mecanum Controller
    # IMPORTANT: Controller should listen to Twist Mux output, NOT raw /cmd_vel
    mecanum_control_node = Node(
        package="recon_bot_mecanum_control",
        executable="mecanum_controller.py",
        name="mecanum_controller",
        output="screen",
        parameters=[{"use_sim_time": False}],
        remappings=[('/cmd_vel', '/cmd_vel_mux')] # Listen to Mux output
    )

    # Odometry: EKF (fuse VIO + wheel)
    EKF_node = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[ekf_config_path],
        remappings=[('odometry/filtered', '/odom')]
    )

    # Navigation: Nav2 Stack (AMCL + Map Server + Planner + Controller)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name_nav), 'launch', 'nav2_custom.launch.py'
        )])
    )

    return LaunchDescription([
        joy_cmd_vel_node,
        mecanum_rplidar_merge,
        mecanum_lidar_node,
        twist_mux,            # Add twist_mux
        mecanum_control_node,
        mecanum_joy_control,
        EKF_node,
        nav2_launch,
    ])
