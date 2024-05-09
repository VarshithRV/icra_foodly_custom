// Copyright 2024 RT Corporation

#ifndef FOODLY_RD_EXAMPLES__WAIST_JT_CONTROL_HPP_
#define FOODLY_RD_EXAMPLES__WAIST_JT_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace foodly_rd_examples
{

class WaistJtControl : public rclcpp::Node
{
public:
  explicit WaistJtControl(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angles_subscription_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr jt_publisher_;

  void angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace foodly_rd_examples

#endif  // FOODLY_RD_EXAMPLES__WAIST_JT_CONTROL_HPP_
