import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rtabmap_launch'),
                    'launch',
                    'rtabmap.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'rgb_topic': '/zed/zed_node/rgb/image_rect_color',
                'depth_topic': '/zed/zed_node/depth/depth_registered',
                'camera_info_topic': '/zed/zed_node/rgb/camera_info',
                'odom_topic': '/odom',
                'frame_id': 'base_footprint',
                'subscribe_depth': 'true',
                'subscribe_rgb': 'true',
                'approx_sync': 'true',
                'visual_odometry': 'true',
                'database_path': os.path.expanduser('~/rtabmap.db'),
                'world_frame_id': 'world',
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'publish_tf': 'true',
                'grid_map': 'true',
                'RGBD/NeighborLinkRefining': 'true',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ORBnFeatures': '500'  # ลด features (แก้ "not found word")
            }.items()
        )
    ])