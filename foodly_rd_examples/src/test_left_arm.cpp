// Copyright 2024 RT Corporation

#include <cmath>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "pose_presets.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_left_arm");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // Declare nodes
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);


  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_arm_node, "l_arm_group");
  move_group_arm.setMaxVelocityScalingFactor(0.5);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(0.2);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "l_gripper_group");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  auto arm_joint_values = move_group_arm.getCurrentJointValues();


  // const double PICK_X = 0.55;
  // const double PICK_Y = 0.10;
  // const double PICK_Z = 0.10;



  // ********************************** Command manipulation *********************************

  move_group_arm.setNamedTarget("l_arm_init_pose");
  move_group_arm.move();

//   gripper_joint_values[0] = GRIPPER_OPEN;
//   move_group_gripper.setJointValueTarget(gripper_joint_values);
//   move_group_gripper.move();


// --------- > Standby <-------------

  // set joint value 
  arm_joint_values[0] = angles::from_degrees(0);
  arm_joint_values[1] = angles::from_degrees(90.0);
  arm_joint_values[2] = angles::from_degrees(0);
  arm_joint_values[3] = angles::from_degrees(-109.0);
  arm_joint_values[4] = angles::from_degrees(0.0);
  arm_joint_values[5] = angles::from_degrees(104.0);
  arm_joint_values[6] = angles::from_degrees(0.0);

  move_group_arm.setJointValueTarget(arm_joint_values);
  move_group_arm.move();


// --------- > PrepPick <------------
  arm_joint_values[0] = angles::from_degrees(-43.0);
  arm_joint_values[1] = angles::from_degrees(90.0);
  arm_joint_values[2] = angles::from_degrees(0.0);
  arm_joint_values[3] = angles::from_degrees(-52.0);
  arm_joint_values[4] = angles::from_degrees(0.0);
  arm_joint_values[5] = angles::from_degrees(60.0);
  arm_joint_values[6] = angles::from_degrees(0.0);

  move_group_arm.setJointValueTarget(arm_joint_values);
  move_group_arm.move();


// =============> Pick <=============
  arm_joint_values[0] = angles::from_degrees(-48.0);
  arm_joint_values[1] = angles::from_degrees(90.0);
  arm_joint_values[2] = angles::from_degrees(0.0);
  arm_joint_values[3] = angles::from_degrees(-38.0);
  arm_joint_values[4] = angles::from_degrees(0.0);
  arm_joint_values[5] = angles::from_degrees(51.0);
  arm_joint_values[6] = angles::from_degrees(0.0);

  move_group_arm.setJointValueTarget(arm_joint_values);
  move_group_arm.move();



// --------- > PrepPick <------------

  arm_joint_values[0] = angles::from_degrees(-43.0);
  arm_joint_values[1] = angles::from_degrees(90.0);
  arm_joint_values[2] = angles::from_degrees(0.0);
  arm_joint_values[3] = angles::from_degrees(-52.0);
  arm_joint_values[4] = angles::from_degrees(0.0);
  arm_joint_values[5] = angles::from_degrees(60.0);
  arm_joint_values[6] = angles::from_degrees(0.0);

  move_group_arm.setJointValueTarget(arm_joint_values);
  move_group_arm.move();



// --------- > Standby <-------------

  // set joint value 
  arm_joint_values[0] = angles::from_degrees(0);
  arm_joint_values[1] = angles::from_degrees(90.0);
  arm_joint_values[2] = angles::from_degrees(0);
  arm_joint_values[3] = angles::from_degrees(-109.0);
  arm_joint_values[4] = angles::from_degrees(0.0);
  arm_joint_values[5] = angles::from_degrees(104.0);
  arm_joint_values[6] = angles::from_degrees(0.0);

  move_group_arm.setJointValueTarget(arm_joint_values);
  move_group_arm.move();


  // ===========> Place <=============
  // set joint value 
  arm_joint_values[0] = angles::from_degrees(0);
  arm_joint_values[1] = angles::from_degrees(90.0);
  arm_joint_values[2] = angles::from_degrees(0);
  arm_joint_values[3] = angles::from_degrees(-75.0);
  arm_joint_values[4] = angles::from_degrees(0.0);
  arm_joint_values[5] = angles::from_degrees(91.0);
  arm_joint_values[6] = angles::from_degrees(0.0);

  move_group_arm.setJointValueTarget(arm_joint_values);
  move_group_arm.move();



// --------- > Standby <-------------

  // set joint value 
  arm_joint_values[0] = angles::from_degrees(0);
  arm_joint_values[1] = angles::from_degrees(90.0);
  arm_joint_values[2] = angles::from_degrees(0);
  arm_joint_values[3] = angles::from_degrees(-109.0);
  arm_joint_values[4] = angles::from_degrees(0.0);
  arm_joint_values[5] = angles::from_degrees(104.0);
  arm_joint_values[6] = angles::from_degrees(0.0);

  move_group_arm.setJointValueTarget(arm_joint_values);
  move_group_arm.move();



  // move_group_arm.setNamedTarget("l_arm_init_pose");
  // move_group_arm.move();


  rclcpp::shutdown();
  return 0;
}
