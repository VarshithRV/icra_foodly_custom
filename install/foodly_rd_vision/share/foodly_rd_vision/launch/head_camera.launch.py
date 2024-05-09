# Copyright 2024 RT Corporation

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    head_camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py']),
            launch_arguments={
                'camera_name': 'head_camera',
                'device_type': 'd415',
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
            }.items()
        )

    return LaunchDescription([
        head_camera_node
    ])
