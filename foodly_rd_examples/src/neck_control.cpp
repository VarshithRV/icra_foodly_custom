// Copyright 2024 RT Corporation

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("neck_control");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("neck_control", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_neck(move_group_node, "neck_group");
  move_group_neck.setMaxVelocityScalingFactor(0.1);  // Set 0.0 ~ 1.0
  move_group_neck.setMaxAccelerationScalingFactor(0.1);  // Set 0.0 ~ 1.0

  move_group_neck.setNamedTarget("neck_init_pose");
  move_group_neck.move();

  auto joint_values = move_group_neck.getCurrentJointValues();

  // Turn the head to the left
  joint_values[0] = angles::from_degrees(45.0);
  move_group_neck.setJointValueTarget(joint_values);
  move_group_neck.move();

  // Turn the head to the right
  joint_values[0] = angles::from_degrees(-45.0);
  move_group_neck.setJointValueTarget(joint_values);
  move_group_neck.move();

  // Turn the head to the front
  joint_values[0] = angles::from_degrees(0.0);
  move_group_neck.setJointValueTarget(joint_values);
  move_group_neck.move();

  // Turn the head down
  joint_values[1] = angles::from_degrees(45.0);
  move_group_neck.setJointValueTarget(joint_values);
  move_group_neck.move();

  // Turn the head up
  joint_values[1] = angles::from_degrees(-45.0);
  move_group_neck.setJointValueTarget(joint_values);
  move_group_neck.move();

  move_group_neck.setNamedTarget("neck_init_pose");
  move_group_neck.move();

  rclcpp::shutdown();
  return 0;
}
