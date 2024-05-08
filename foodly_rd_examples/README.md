# foodly_rd_examples

This package includes examples to control Foodly Type R.

- [Launch Foodly](#launch-foodly)
  - [Launch Foodly with real hardware.](#launch-foodly-with-real-hardware)
  - [Launch Foodly with mock hardware.](#launch-foodly-with-mock-hardware)
  - [Launch Foodly with Gazebo](#launch-foodly-with-gazebo)
- [Launch examples](#launch-examples)
  - [Launch examples with Gazebo](#launch-examples-with-gazebo)
- [Examples](#examples)
  - [gripper\_control](#gripper_control)
  - [neck\_control](#neck_control)
  - [waist\_control](#waist_control)
  - [pick\_and\_place\_right\_arm\_waist](#pick_and_place_right_arm_waist)
  - [pick\_and\_place\_left\_arm](#pick_and_place_left_arm)
  - [head\_camera\_tracking](#head_camera_tracking)
  - [chest\_camera\_tracking](#chest_camera_tracking)

## Launch Foodly

### Launch Foodly with real hardware.

```sh
ros2 launch foodly_rd_examples demo.launch.py
```

Without cameras:
```sh
ros2 launch foodly_rd_examples demo.launch.py use_head_camera:=false use_chest_camera:=false
```

### Launch Foodly with mock hardware.

```sh
ros2 launch foodly_rd_examples demo.launch.py use_mock:=true
```

Without cameras:
```sh
ros2 launch foodly_rd_examples demo.launch.py use_mock:=true use_head_camera:=false use_chest_camera:=false
```

### Launch Foodly with Gazebo

```sh
ros2 launch foodly_rd_gazebo foodly_rd_with_table.launch.py
```

Without cameras:
```sh
ros2 launch foodly_rd_gazebo foodly_rd_with_table.launch.py use_head_camera:=false use_chest_camera:=false
```

## Launch examples

You can run each examples with `demo.launch.py` running. 
For example, to open/close the grippers of the two arms with the following command.

```sh
ros2 launch foodly_rd_examples example.launch.py example:='gripper_control'
```

### Launch examples with Gazebo

To launch examples with Gazebo, use `use_sim_time` option.

```sh
ros2 launch foodly_rd_examples example.launch.py example:='gripper_control' use_sim_time:='true'
```

## Examples

- [gripper\_control](#gripper_control)
- [neck\_control](#neck_control)
- [waist\_control](#waist_control)
- [pick\_and\_place\_right\_arm\_waist](#pick_and_place_right_arm_waist)
- [pick\_and\_place\_left\_arm](#pick_and_place_left_arm)
- [head\_camera\_tracking](#head_camera_tracking)
- [chest\_camera\_tracking](#chest_camera_tracking)

---

### gripper_control

This is an example to open/close the grippers of the two arms.

```sh
ros2 launch foodly_rd_examples example.launch.py example:='gripper_control'
```

[back to example list](#examples)

---

### neck_control

This is an example to change angles of the neck.

```sh
ros2 launch foodly_rd_examples example.launch.py example:='neck_control'
```

[back to example list](#examples)

---

### waist_control

This is an example to change angles of the waist.

```sh
ros2 launch foodly_rd_examples example.launch.py example:='waist_control'
```

[back to example list](#examples)

---

### pick_and_place_right_arm_waist

This is an example to pick and place a small object with right hand while turning the waist.

```sh
ros2 launch foodly_rd_examples example.launch.py example:='pick_and_place_right_arm_waist'
```

[back to example list](#examples)

---

### pick_and_place_left_arm

This is an example to pick and place a small object with left hand.

```sh
ros2 launch foodly_rd_examples example.launch.py example:='pick_and_place_left_arm'
```

[back to example list](#examples)

---

### head_camera_tracking

This is an example to use the head camera images for orange object tracking.

```sh
ros2 launch foodly_rd_examples head_camera_tracking.launch.py
```

[back to example list](#examples)

---

### chest_camera_tracking

This is an example to use the chest camera images for orange object tracking.

```sh
ros2 launch foodly_rd_examples chest_camera_tracking.launch.py
```

[back to example list](#examples)

---
