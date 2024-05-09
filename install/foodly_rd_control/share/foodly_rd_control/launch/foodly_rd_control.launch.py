# Copyright 2024 RT Corporation

import os

from ament_index_python.packages import get_package_share_directory
from foodly_rd_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_use_mock = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Set "true" to use mock hardware.'
    )

    right_config_file_path = os.path.join(
        get_package_share_directory('foodly_rd_control'),
        'config',
        'right_manipulator_config.yaml'
    )
    left_config_file_path = os.path.join(
        get_package_share_directory('foodly_rd_control'),
        'config',
        'left_manipulator_config.yaml'
    )
    center_config_file_path = os.path.join(
        get_package_share_directory('foodly_rd_control'),
        'config',
        'center_manipulator_config.yaml'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_right_mock_hardware = LaunchConfiguration('use_mock')
    description_loader.use_left_mock_hardware = LaunchConfiguration('use_mock')
    description_loader.use_center_mock_hardware = LaunchConfiguration('use_mock')
    description_loader.right_port_name = '/dev/foodlyright'
    description_loader.right_baudrate = '4000000'
    description_loader.right_manipulator_config_file_path = right_config_file_path
    description_loader.left_port_name = '/dev/foodlyleft'
    description_loader.left_baudrate = '4000000'
    description_loader.left_manipulator_config_file_path = left_config_file_path
    description_loader.center_port_name = '/dev/foodlyrcenter'
    description_loader.center_baudrate = '4000000'
    description_loader.center_manipulator_config_file_path = center_config_file_path
    description_loader.timeout_seconds = '5.0'

    declare_loaded_description = DeclareLaunchArgument(
        'loaded_description',
        default_value=description_loader.load(),
        description='Set robot_description text.  \
                     It is recommended to use RobotDescriptionLoader() in foodly_rd_description.'
    )

    foodly_rd_controllers = os.path.join(
        get_package_share_directory('foodly_rd_control'),
        'config',
        'foodly_rd_controllers.yaml'
        )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': LaunchConfiguration('loaded_description')},
                    foodly_rd_controllers],
        output='screen',
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

    return LaunchDescription([
        declare_use_mock,
        declare_loaded_description,
        controller_manager,
        spawn_right_arm_controller,
        spawn_right_gripper_controller,
        spawn_left_arm_controller,
        spawn_left_gripper_controller,
        spawn_joint_state_broadcaster,
        spawn_neck_controller,
        spawn_waist_yaw_controller
    ])
