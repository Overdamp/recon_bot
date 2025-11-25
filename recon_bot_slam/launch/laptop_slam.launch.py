import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. หาที่อยู่ของไฟล์ Config (Format A ดั้งเดิม)
    config_file_path = os.path.join(
        get_package_share_directory('recon_bot_slam'),
        'config',
        'mapper_params_online_async.yaml' # <-- ใช้ไฟล์ดั้งเดิม (Format A)
    )

    # 2. สร้างโหนด Slam Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox', # <-- ตั้งชื่อให้ตรง (สำคัญ)
        output='screen',
        parameters=[config_file_path], # <-- โหลด Config ที่นี่
        remappings=[
            # (ถ้าจำเป็นต้อง Remap อะไรเพิ่ม)
        ]
    )

    # 3. รันโหนด
    return LaunchDescription([
        slam_toolbox_node
    ])