# Copyright 2024 RT Corporation

from foodly_rd_description.robot_description_loader import RobotDescriptionLoader
from launch.launch_context import LaunchContext
import pytest


def exec_load(loader):
    # This method is based on the test of Command substitution.
    # https://github.com/ros2/launch/blob/074cd2903ddccd61bce8f40a0f58da0b7c200481/launch/test/launch/substitutions/test_command.py#L47
    context = LaunchContext()
    return loader.load().perform(context)


def test_load_description():
    # Expect xacro to load successfully
    rdl = RobotDescriptionLoader()
    assert exec_load(rdl)


def test_change_description_path():
    # Set wrong xacro file path and expect loading to fail
    rdl = RobotDescriptionLoader()
    rdl.robot_description_path = 'hoge'
    with pytest.raises(Exception) as e:
        exec_load(rdl)
    assert e.value


def test_use_gazebo():
    # Expect use_gazebo to be changed and ign_ros2_control set in xacro
    rdl = RobotDescriptionLoader()
    rdl.use_gazebo = 'true'
    rdl.gz_control_config_package = 'foodly_rd_description'
    rdl.gz_control_config_file_path = 'config/dummy_controllers.yaml'
    assert 'ign_ros2_control/IgnitionSystem' in exec_load(rdl)


def test_use_mock_hardware():
    # Expect mock_components to be included in xacro
    rdl = RobotDescriptionLoader()
    rdl.use_right_mock_hardware = 'true'
    rdl.use_left_mock_hardware = 'true'
    rdl.use_center_mock_hardware = 'true'
    assert 'mock_components/GenericSystem' in exec_load(rdl)
    assert 'foodly_rd_hardware/FoodlyRDHardware' not in exec_load(rdl)

    rdl = RobotDescriptionLoader()
    rdl.use_right_mock_hardware = 'false'
    rdl.use_left_mock_hardware = 'false'
    rdl.use_center_mock_hardware = 'false'
    assert 'mock_components/GenericSystem' not in exec_load(rdl)
    assert 'foodly_rd_hardware/FoodlyRDHardware' in exec_load(rdl)


def test_port_name():
    # Expect port_name to be changed in xacro
    rdl = RobotDescriptionLoader()
    rdl.right_port_name = '/dev/ttyUSB1'
    rdl.left_port_name = '/dev/ttyUSB2'
    rdl.center_port_name = '/dev/ttyUSB3'

    assert '"port_name">/dev/ttyUSB1' in exec_load(rdl)
    assert '"port_name">/dev/ttyUSB2' in exec_load(rdl)
    assert '"port_name">/dev/ttyUSB3' in exec_load(rdl)


def test_baudrate():
    # Expect baudrate to be changed in xacro
    rdl = RobotDescriptionLoader()
    rdl.right_baudrate = '10000'
    rdl.left_baudrate = '20000'
    rdl.center_baudrate = '30000'

    assert '"baudrate">10000' in exec_load(rdl)
    assert '"baudrate">20000' in exec_load(rdl)
    assert '"baudrate">30000' in exec_load(rdl)


def test_manipulator_config_file_path():
    # Expect manipulator_config_file_path to be changed in xacro
    rdl = RobotDescriptionLoader()
    rdl.right_manipulator_config_file_path = 'right/config/file/path'
    rdl.left_manipulator_config_file_path = 'left/config/file/path'
    rdl.center_manipulator_config_file_path = 'center/config/file/path'

    assert '"manipulator_config_file_path">right/config/file/path' in exec_load(rdl)
    assert '"manipulator_config_file_path">left/config/file/path' in exec_load(rdl)
    assert '"manipulator_config_file_path">center/config/file/path' in exec_load(rdl)


def test_timeout_seconds():
    # Expect timeout_seconds to be changed in xacro
    rdl = RobotDescriptionLoader()
    rdl.timeout_seconds = '3.0'
    assert '"timeout_seconds">3.0' in exec_load(rdl)


def test_true_fix_base_link():
    # Checking the behavior when fix_base_link is changed
    rdl = RobotDescriptionLoader()
    rdl.fix_base_link = 'true'
    assert 'base_link_to_world_joint' in exec_load(rdl)

    rdl.fix_base_link = 'false'
    assert 'base_link_to_world_joint' not in exec_load(rdl)


def test_use_gazebo_head_camera():
    # Expect use_gazebo_head_camera to be changed and head_camera_color_frame to be set in xacro
    rdl = RobotDescriptionLoader()
    rdl.use_gazebo = 'true'
    rdl.use_gazebo_head_camera = 'true'
    rdl.gz_control_config_package = 'foodly_rd_description'
    rdl.gz_control_config_file_path = 'config/dummy_controllers.yaml'
    assert 'reference="head_camera_color_frame"' in exec_load(rdl)


def test_use_gazebo_chest_camera():
    # Expect use_gazebo_chest_camera to be changed and chest_camera_color_frame to be set in xacro
    rdl = RobotDescriptionLoader()
    rdl.use_gazebo = 'true'
    rdl.use_gazebo_chest_camera = 'true'
    rdl.gz_control_config_package = 'foodly_rd_description'
    rdl.gz_control_config_file_path = 'config/dummy_controllers.yaml'
    assert 'reference="chest_camera_color_frame"' in exec_load(rdl)
