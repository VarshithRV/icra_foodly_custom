# Copyright 2024 RT Corporation

import os

from ament_index_python.packages import get_package_share_directory
from foodly_rd_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    description_loader = RobotDescriptionLoader()

    robot_description_semantic_config = load_file(
        'foodly_rd_moveit_config', 'config/foodly_rd.srdf')
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('foodly_rd_moveit_config', 'config/kinematics.yaml')

    # declare_example_name = DeclareLaunchArgument(
    #     'example', default_value='gripper_control',
    #     description=('Set an example executable name: '
    #                  '[gripper_control, neck_control, waist_control,'
    #                  'pick_and_place_right_arm_waist, pick_and_place_left_arm]')
    # )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description=('Set true when using the gazebo simulator.')
    )

    left_arm_server_node = Node(name=['left_arm_server', '_node'],
                        package='foodly_rd_examples',
                        executable='left_arm_server',
                        output='log',
                        parameters=[{'robot_description': description_loader.load()},
                                    robot_description_semantic,
                                    kinematics_yaml])
    

    right_arm_server_node = Node(name=['right_arm_server', '_node'],
                    package='foodly_rd_examples',
                    executable='right_arm_server',
                    output='log',
                    parameters=[{'robot_description': description_loader.load()},
                                robot_description_semantic,
                                kinematics_yaml])
    
    conveyor_server = Node(name=['conveyor_server', '_node'],
                package='conveyor_control',
                executable='conveyor_server',
                output='log',)



    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        # declare_example_name,
        left_arm_server_node,
        right_arm_server_node,
        conveyor_server
    ])
