from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define package name and paths
    package_name = 'recon_bot_bringup'
    map_file = '/home/cobot/Ros2_directory/recon_ws/map/map.yaml'
    nav2_params = '/home/cobot/Ros2_directory/recon_ws/src/recon_bot_bringup/config/nav2_params.yaml'
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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

    # Start the map_server with lifecycle
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': use_sim_time}]
    )

    # Lifecycle manager for map_server
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['map_server']}]
    )

    # Lifecycle manager for navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': ['amcl', 'planner_server', 'controller_server', 
                                    'bt_navigator', 'smoother_server', 'waypoint_follower']}]
    )

    # Start the Navigation bringup with all components (planner, controller, AMCL, BT navigator)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'paragit gms_file': nav2_params
        }.items()
    )

    # Additional nodes for lidar filtering, motor control, and odometry
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

    # Combine everything into the launch description
    return LaunchDescription([
        rsp,
        rplidar_two,
        twist_mux,
        map_server,
        lifecycle_manager_map,  # Add lifecycle manager for map_server
        lifecycle_manager_navigation,  # Add lifecycle manager for navigation
        nav2_bringup,
        lidar_filtered_node,
        motor_control_node,
        odometry_node
    ])
