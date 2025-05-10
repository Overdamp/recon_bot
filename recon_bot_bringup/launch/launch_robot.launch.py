import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():

    robot_name = "recon_bot"
    package_name1 = robot_name + "_description"
    rviz_config = os.path.join(get_package_share_directory(
        package_name1), "launch", robot_name + ".rviz")
    robot_description = os.path.join(get_package_share_directory(
        package_name1), "robot", robot_name + ".urdf.xacro")
    
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name2='recon_bot_bringup' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name2),'launch','rsp.launch.py'
                )])
    )

    rplidar_two = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name2),'launch','rplidar_two.launch.py'
                )])
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name2),'launch','joystick.launch.py'
    #             )])
    # )


    twist_mux_params = os.path.join(get_package_share_directory(package_name2),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/mecanum_cont/cmd_vel_unstamped')]
        )

    


    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name2),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
                    output={
                        "stdout": "screen",
                        "stderr": "screen",
                    }
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

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


    velocity_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["velocity_controller", "-c", "/controller_manager"],
    )

    delayed_velocity_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_broad_spawner,
            on_start=[velocity_spawner],
        )
    )

    joint_trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    delayed_joint_trajectory_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=velocity_spawner,
            on_start=[joint_trajectory_spawner],
        )
    )



    rplidar_merge = Node(
        package="recon_bot_slam",
        executable="lidar_merge.py"
    )


    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        # rplidar_two,
        # rplidar_merge,
        rsp,
        # joystick,
        # twist_mux,
        delayed_controller_manager, 
        delayed_joint_broad_spawner,
        delayed_velocity_spawner,
        delayed_joint_trajectory_spawner 
    ])