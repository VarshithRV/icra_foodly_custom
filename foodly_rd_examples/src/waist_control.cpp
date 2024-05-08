// Copyright 2024 RT Corporation

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("waist_control");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("waist_control", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_waist(move_group_node, "waist_group");
  move_group_waist.setMaxVelocityScalingFactor(0.1);  // Set 0.0 ~ 1.0
  move_group_waist.setMaxAccelerationScalingFactor(0.1);  // Set 0.0 ~ 1.0

  move_group_waist.setNamedTarget("waist_init_pose");
  move_group_waist.move();

  auto joint_values = move_group_waist.getCurrentJointValues();

  // Turn the waist to the right
  joint_values[0] = angles::from_degrees(45.0);
  move_group_waist.setJointValueTarget(joint_values);
  move_group_waist.move();

  // Turn the waist to the left
  joint_values[0] = angles::from_degrees(-45.0);
  move_group_waist.setJointValueTarget(joint_values);
  move_group_waist.move();

  move_group_waist.setNamedTarget("waist_init_pose");
  move_group_waist.move();

  rclcpp::shutdown();
  return 0;
}
