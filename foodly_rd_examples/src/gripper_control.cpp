// Copyright 2024 RT Corporation

#include <cmath>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("gripper_control");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node =
    rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_r_gripper_node =
    rclcpp::Node::make_shared("move_group_r_gripper_node", node_options);
  auto move_group_l_gripper_node =
    rclcpp::Node::make_shared("move_group_l_gripper_node", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_r_gripper_node);
  executor.add_node(move_group_l_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_arm_node, "two_arm_group");
  move_group_arm.setMaxVelocityScalingFactor(0.1);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(0.1);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_r_gripper(move_group_r_gripper_node, "r_gripper_group");
  move_group_r_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_r_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto r_gripper_joint_values = move_group_r_gripper.getCurrentJointValues();
  const double R_GRIPPER_CLOSE = 0.0;
  const double R_GRIPPER_OPEN = angles::from_degrees(40.0);

  MoveGroupInterface move_group_l_gripper(move_group_l_gripper_node, "l_gripper_group");
  move_group_l_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_l_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto l_gripper_joint_values = move_group_l_gripper.getCurrentJointValues();
  const double L_GRIPPER_CLOSE = 0.0;
  const double L_GRIPPER_OPEN = angles::from_degrees(-40.0);

  move_group_arm.setNamedTarget("two_arm_init_pose");
  move_group_arm.move();

  // Move right gripper
  for (int i = 0; i < 2; i++) {
    r_gripper_joint_values[0] = R_GRIPPER_OPEN;
    move_group_r_gripper.setJointValueTarget(r_gripper_joint_values);
    move_group_r_gripper.move();

    r_gripper_joint_values[0] = R_GRIPPER_CLOSE;
    move_group_r_gripper.setJointValueTarget(r_gripper_joint_values);
    move_group_r_gripper.move();
  }

  // Move left gripper
  for (int i = 0; i < 2; i++) {
    l_gripper_joint_values[0] = L_GRIPPER_OPEN;
    move_group_l_gripper.setJointValueTarget(l_gripper_joint_values);
    move_group_l_gripper.move();

    l_gripper_joint_values[0] = L_GRIPPER_CLOSE;
    move_group_l_gripper.setJointValueTarget(l_gripper_joint_values);
    move_group_l_gripper.move();
  }

  rclcpp::shutdown();
  return 0;
}
