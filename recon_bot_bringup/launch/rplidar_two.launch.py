from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        

         Node(
            package='rplidar_ros',  # Name of the RPLIDAR ROS2 package
            executable='rplidar_composition',  # The main node executable
            name='rplidar_front',  # Unique name for the front RPLIDAR node
            namespace='front',  # Namespace for the front RPLIDAR
            output='screen',  # Print the output on the screen
            parameters=[{
                'serial_port': '/dev/ttyUSB_RPLIDAR2',  # Front RPLIDAR connected here
                'serial_baudrate': 115200,  # Baudrate for the RPLIDAR (could be 256000 for other models)
                'frame_id': 'lidar_front_link',  # The frame ID for the front lidar
                'inverted': False,  # Normal scanning direction
                'angle_compensate': True,  # Compensate for angle distortion
                'scan_mode': 'Standard',
                'sample_rate': 5, 
            }],
            remappings=[
                ('/scan', '/scan_front'),  # Remap the scan topic for the front RPLIDAR
            ]
        ),
        
        # Back RPLIDAR
        Node(
            package='rplidar_ros',  # Name of the RPLIDAR ROS2 package
            executable='rplidar_composition',  # The main node executable
            name='rplidar_back',  # Unique name for the back RPLIDAR node
            namespace='back',  # Namespace for the back RPLIDAR
            output='screen',  # Print the output on the screen
            parameters=[{
                'serial_port': '/dev/ttyUSB_RPLIDAR1',  # Back RPLIDAR connected here
                'serial_baudrate': 115200,  # Baudrate for the RPLIDAR
                'frame_id': 'lidar_back_link',  # The frame ID for the back lidar
                'inverted': False,  # Normal scanning direction
                'angle_compensate': True,  # Compensate for angle distortion
                'scan_mode': 'Standard',
                'sample_rate': 20, 
            }],
            remappings=[
                ('/scan', '/scan_back'),  # Remap the scan topic for the back RPLIDAR
            ]
        ),
        # # Add other nodes or configurations as needed

        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'frame_id': 'laser_frame',
        #         'angle_compensate': True,
        #         'scan_mode': 'Standard'
        #     }]
        # ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen',
        #     arguments=['-d', [ThisLaunchFileDir(), '/../config/rplidar.rviz']],
        # )
    ])
