# Copyright 2024 RT Corporation

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    chest_camera_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('realsense2_camera'),
                '/launch/rs_launch.py']),
            launch_arguments={
                'camera_name': 'chest_camera',
                'device_type': 'd435',
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
            }.items()
        )
    
    

    return LaunchDescription([
        chest_camera_node,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            output='screen',
            arguments=[
                '0', '0', '0',  # Translation (x, y, z)
                '0', '0', '0', '1',  # Rotation (Quaternion: x, y, z, w)
                'chest_camera_color_frame',  # Frame ID
                'chest_camera_link_color_frame'  # Child Frame ID
            ],
        )
    ])
