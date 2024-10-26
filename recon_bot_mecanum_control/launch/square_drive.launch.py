from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Launch the joystick command velocity node first
        # Node(
        #     package='recon_bot_mecanum_control',
        #     executable='a_joy_cmd_vel.py',
        #     name='joy_cmd_vel',
        #     output='screen'
        # ),


        # Launch the motor read and write node after a 10 second delay
        TimerAction(
            period=0.2,
            actions=[
                Node(
                    package='recon_bot_mecanum_control',
                    executable='a_motor_read_write.py',
                    name='a_motor_read_write',
                    output='screen'
                )
            ]
        ),

        # Launch the odometry publisher node after a 15 second delay
        TimerAction(
            period=0.2,
            actions=[
                Node(
                    package='recon_bot_mecanum_control',
                    executable='a_odometry_pub.py',
                    name='odometry_pub',
                    output='screen'
                )
            ]
        ),

        # Launch the square drive test node after a 20 second delay
        TimerAction(
            period=.2,
            actions=[
                Node(
                    package='recon_bot_mecanum_control',
                    executable='a_square_drive_test.py',
                    name='square_drive_test',
                    output='screen'
                )
            ]
        ),
    ])
