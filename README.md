# foodly_rd_ros2

## Installation

### Download package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Download foodly_rd_ros2 here
```

### Install dependencies

Other ROS 2 packages.

```bash
sudo apt install python3-vcstool
cd ~/ros2_ws/src
vcs import < foodly_rd_ros2/foodly_rd.repos

rosdep install -iry --from-paths . 
```

Dynamixel SDK Python library.

```bash
cd ~
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd ~/DynamixelSDK/python
sudo python3 setup.py install
```

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --allow-overriding controller_interface controller_manager hardware_interface hardware_interface_testing ros2_control_test_assets controller_manager_msgs
```
