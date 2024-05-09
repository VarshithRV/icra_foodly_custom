# Copyright 2024 RT Corporation

import os

from ament_index_python.packages import get_package_share_directory
from foodly_rd_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_mock = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Set "true" to use mock hardware.'
    )

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
    description_loader.center_port_name = '/dev/foodlycenter'
    description_loader.center_baudrate = '4000000'
    description_loader.center_manipulator_config_file_path = center_config_file_path
    description_loader.timeout_seconds = '5.0'

    description = description_loader.load()

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('foodly_rd_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={
                'loaded_description': description
            }.items()
        )

    control_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('foodly_rd_control'),
                '/launch/foodly_rd_control.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    head_camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('foodly_rd_vision'),
                '/launch/head_camera.launch.py']),
            condition=IfCondition(LaunchConfiguration('use_head_camera')),
        )

    chest_camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('foodly_rd_vision'),
                '/launch/chest_camera.launch.py']),
            condition=IfCondition(LaunchConfiguration('use_chest_camera')),
        )

    return LaunchDescription([
        declare_use_mock,
        declare_use_head_camera,
        declare_use_chest_camera,
        move_group,
        control_node,
        head_camera_node,
        chest_camera_node
    ])
