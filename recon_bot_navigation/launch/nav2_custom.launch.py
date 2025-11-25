import os
from ament_index_python.packages import get_package_share_directory  # <-- เพิ่มบรรทัดนี้!
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to map file (slam_map.yaml in config)
    map_yaml = os.path.join(
        get_package_share_directory('recon_bot_navigation'),
        'config',
        'slam_map.yaml'
    )

    # Include official Nav2 launch (จาก nav2_bringup)
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),  # <-- ถูกต้อง: nav2_bringup
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': 'false',
                'params_file': PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'params',
                    'nav2_params.yaml'
                ]),
                'base_link_frame': 'base_footprint'  # remap base_link → Mobile_Base
            }.items()
        )
    ])