# launch/mecanum_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='recon_bot_mecanum_control',  # Replace with your actual package name
            executable='recon_bot_joy_controller.py',  # The name of your joystick controller executable
            name='recon_bot_joy_controller',
            output='screen',
        ),

        Node(
            package='recon_bot_commnunication',  # Replace with your actual package name
            executable='joy_pyserial.py',  # The name of your odometry executable
            name='joy_pyserial',
            output='screen',
        ),
        
    ])