import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robot_name = "recon_bot"
    package_name1 = robot_name + "_description"
    package_name2 = robot_name + "_bringup"

    # Define the package and file path for the URDF/Xacro file
    xacro_file_path = os.path.join(
        get_package_share_directory(package_name1),
        'robot',
        'recon_bot.urdf.xacro'
    )

    # Process the xacro file to generate the robot description
    robot_description = xacro.process_file(xacro_file_path).toxml()

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name2), 'launch', 'rsp.launch.py'
        )])
    )

    # Twist Mux
    twist_mux_params = os.path.join(get_package_share_directory(package_name2), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/velocity_controller/cmd_vel_unstamped')]
    )

    # Controller Manager
    controller_params_file = os.path.join(get_package_share_directory(package_name2), 'config', 'my_controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ],
        output="screen"
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # Joint State Broadcaster Spawner
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Velocity Controller Spawner
    velocity_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    delayed_velocity_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[velocity_spawner],
        )
    )

    # Launch Description
    return LaunchDescription([
        rsp,
        twist_mux,
        # delayed_controller_manager,
        # delayed_joint_broad_spawner,
        # delayed_velocity_spawner
    ])