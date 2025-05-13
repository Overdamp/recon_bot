from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # โหลดไฟล์ XACRO/URDF
    xacro_file = os.path.join(
        get_package_share_directory('recon_bot_description'),
        'robot', 'recon_bot.urdf.xacro'
    )
    
    # โหลดไฟล์ controller YAML
    controller_yaml = os.path.join(
        get_package_share_directory('recon_bot_description'),
        'config', 'mecanum_controllers.yaml'
    )
    
    return LaunchDescription([
        # โหลด URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': 
                Command(['xacro ', xacro_file])}]
        ),
        
        # โหลด Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': 
                Command(['xacro ', xacro_file])}, 
                controller_yaml],
            output='screen'
        ),
        
        # สตาร์ท velocity controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['wheel_velocity_controller'],
            output='screen'
        ),
    ])