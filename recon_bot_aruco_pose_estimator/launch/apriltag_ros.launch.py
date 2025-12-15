import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'recon_bot_aruco_pose_estimator'
    
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'tags_36h11.yaml'
    )

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/zed/zed_node/left/image_rect_color'),
                ('camera_info', '/zed/zed_node/left/camera_info'),
            ],
            parameters=[
                config_file,
                config_file,
                {'qos_profile': 'sensor_data'},
            ]
        )
    ])
