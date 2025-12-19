import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to map file (slam_map.yaml in config)
    default_map_yaml = os.path.join(
        get_package_share_directory('recon_bot_navigation'),
        'config',
        'slam_map.yaml'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Full path to map yaml file to load'
    )
    
    # Path to params file
    params_file = os.path.join(
        get_package_share_directory('recon_bot_navigation'),
        'config',
        'nav2_params.yaml'
    )

    # Include official Nav2 launch (from nav2_bringup)
    return LaunchDescription([
        map_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'false',
                'params_file': params_file,
                # 'base_link_frame': 'base_footprint' # Removed as it's handled in params
            }.items()
        )
    ])