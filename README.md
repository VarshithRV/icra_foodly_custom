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


### Run program for simulation 

```bash
# Terminal 1
#  These commands for Gazebo launch (run only one !! --> with camera or wihtout)
#  With camera
ros2 launch foodly_rd_gazebo foodly_rd_with_table.launch.py
#  Without camera
ros2 launch foodly_rd_gazebo foodly_rd_with_table.launch.py use_head_camera:=false use_chest_camera:=false

# Terminal 2
#  Run vision server
ros2 run icra_perception vision.py

# Terminal 3
#  This for active the server node
ros2 launch foodly_rd_examples example.launch.py use_sim_time:='true'

# Terminal 4
#  This for state machine 
ros2 run state_machine state_machine_client
```


### Run with real robot
```bash
# Terminal 1
# This with the real hardware, most probably, it will already be launched
ros2 launch foodly_rd_examples demo.launch.py

# Terminal 2
#  Run vision server
ros2 run icra_perception vision.py

# Terminal 3
#  This for active the server node
ros2 launch foodly_rd_examples example.launch.py 

# Terminal 4
#  This for state machine 
ros2 run state_machine state_machine_client
```
