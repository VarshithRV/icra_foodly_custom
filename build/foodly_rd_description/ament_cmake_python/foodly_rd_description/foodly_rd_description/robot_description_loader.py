# Copyright 2024 RT Corporation

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


class RobotDescriptionLoader():

    def __init__(self):
        self.robot_description_path = os.path.join(
            get_package_share_directory('foodly_rd_description'),
            'urdf',
            'foodly_rd.urdf.xacro')
        self.use_right_mock_hardware = 'false'
        self.use_left_mock_hardware = 'false'
        self.use_center_mock_hardware = 'false'
        self.right_port_name = '/dev/foodlyright'
        self.right_baudrate = '4000000'
        self.right_manipulator_config_file_path = ''
        self.left_port_name = '/dev/foodlyleft'
        self.left_baudrate = '4000000'
        self.left_manipulator_config_file_path = ''
        self.center_port_name = '/dev/foodlycenter'
        self.center_baudrate = '4000000'
        self.center_manipulator_config_file_path = ''
        self.timeout_seconds = '1.0'
        self.use_gazebo = 'false'
        self.use_gazebo_head_camera = 'false'
        self.use_gazebo_chest_camera = 'false'
        self.fix_base_link = 'false'
        self.gz_control_config_package = ''
        self.gz_control_config_file_path = ''

    def load(self):
        return Command([
                'xacro ',
                self.robot_description_path,
                ' use_right_mock_hardware:=', self.use_right_mock_hardware,
                ' use_left_mock_hardware:=', self.use_left_mock_hardware,
                ' use_center_mock_hardware:=', self.use_center_mock_hardware,
                ' right_port_name:=', self.right_port_name,
                ' right_baudrate:=', self.right_baudrate,
                ' right_manipulator_config_file_path:=', self.right_manipulator_config_file_path,
                ' left_port_name:=', self.left_port_name,
                ' left_baudrate:=', self.left_baudrate,
                ' left_manipulator_config_file_path:=', self.left_manipulator_config_file_path,
                ' center_port_name:=', self.center_port_name,
                ' center_baudrate:=', self.center_baudrate,
                ' center_manipulator_config_file_path:=', self.center_manipulator_config_file_path,
                ' timeout_seconds:=', self.timeout_seconds,
                ' use_gazebo:=', self.use_gazebo,
                ' use_gazebo_head_camera:=', self.use_gazebo_head_camera,
                ' use_gazebo_chest_camera:=', self.use_gazebo_chest_camera,
                ' fix_base_link:=', self.fix_base_link,
                ' gz_control_config_package:=', self.gz_control_config_package,
                ' gz_control_config_file_path:=', self.gz_control_config_file_path
                ])
