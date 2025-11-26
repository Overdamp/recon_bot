import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "recon_bot_description"
    
    # Use the same Rviz config as the robot to see the same things
    rviz_config = os.path.join(
        get_package_share_directory(package_name), 
        "rviz", 
        "recon_bot_config.rviz" # Ensure this matches your actual config name
    )
    
    # If the file doesn't exist, fall back to a default or check the name
    # Note: In vio_slam.launch.py it was f"{robot_name}config.rviz" -> "recon_botconfig.rviz"
    # Let's try to match what was likely intended or used.
    # Based on previous file read: "recon_botconfig.rviz" seems to be the one used in vio_slam.
    
    rviz_config_path = os.path.join(get_package_share_directory(package_name), "rviz", "recon_botconfig.rviz")

    return LaunchDescription([
        # [Perfect Architecture] Laptop runs Rviz to visualize data from the Robot
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
            output="screen"
        )
    ])
