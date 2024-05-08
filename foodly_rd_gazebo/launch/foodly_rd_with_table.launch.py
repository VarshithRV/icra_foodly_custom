# Copyright 2024 RT Corporation

import os

from ament_index_python.packages import get_package_share_directory
from foodly_rd_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_head_camera = DeclareLaunchArgument(
        'use_head_camera',
        default_value='true',
        description='Use head camera.'
    )

    declare_use_chest_camera = DeclareLaunchArgument(
        'use_chest_camera',
        default_value='true',
        description='Use chest camera.'
    )

    declare_fix_base_link = DeclareLaunchArgument(
        'fix_base_link',
        default_value='false',
        description='Fix base link.'
    )

    # If You don't pass additional PATH, You can't STL file is not loaded.
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('foodly_rd_description'))}
    world_file = os.path.join(
        get_package_share_directory('foodly_rd_gazebo'), 'worlds', 'icra_world.sdf')
    gui_config = os.path.join(
        get_package_share_directory('foodly_rd_gazebo'), 'gui', 'gui.config')
    # The controller will not start unless you start the simulation with the -r option.
    ign_gazebo = ExecuteProcess(
            cmd=['ign gazebo -r', world_file, '--gui-config', gui_config],
            output='screen',
            additional_env=env,
            shell=True
        )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'foodly_rd',
                   '-x', '0.0',
                   '-z', '0.898',
                   '-allow_renaming', 'true'],
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_gazebo = 'true'
    description_loader.use_gazebo_head_camera = LaunchConfiguration('use_head_camera')
    description_loader.use_gazebo_chest_camera = LaunchConfiguration('use_chest_camera')
    description_loader.fix_base_link = LaunchConfiguration('fix_base_link')
    description_loader.gz_control_config_package = 'foodly_rd_control'
    description_loader.gz_control_config_file_path = 'config/foodly_rd_controllers.yaml'
    description = description_loader.load()

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('foodly_rd_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    spawn_joint_state_broadcaster = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
                shell=True,
                output='screen',
            )

    spawn_right_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner right_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_right_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner right_gripper_controller'],
                shell=True,
                output='screen',
            )

    spawn_left_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner left_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_left_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner left_gripper_controller'],
                shell=True,
                output='screen',
            )

    spawn_neck_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner neck_controller'],
                shell=True,
                output='screen',
            )

    spawn_waist_yaw_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner waist_yaw_controller'],
                shell=True,
                output='screen',
            )

    bridge_file = os.path.join(
        get_package_share_directory('foodly_rd_gazebo'), 'config', 'bridge.yaml')

    bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'config_file': bridge_file}],
                output='screen'
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        declare_use_head_camera,
        declare_use_chest_camera,
        declare_fix_base_link,
        ign_gazebo,
        ignition_spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_right_arm_controller,
        spawn_right_gripper_controller,
        spawn_left_arm_controller,
        spawn_left_gripper_controller,
        spawn_neck_controller,
        spawn_waist_yaw_controller,
        move_group,
        bridge
    ])
