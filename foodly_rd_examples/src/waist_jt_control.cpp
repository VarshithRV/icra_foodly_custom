// Copyright 2024 RT Corporation

#include "foodly_rd_examples/waist_jt_control.hpp"

#include "angles/angles.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace foodly_rd_examples
{

WaistJtControl::WaistJtControl(const rclcpp::NodeOptions & options)
: Node("waist_control", options)
{
  angles_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "target_angles", 10, std::bind(&WaistJtControl::angles_callback, this, _1));

  jt_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/waist_yaw_controller/joint_trajectory", 10);
}

void WaistJtControl::angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  const auto TIME_FROM_START = 1ms;
  const double MAX_YAW_ANGLE = angles::from_degrees(90);
  const double MIN_YAW_ANGLE = angles::from_degrees(-90);

  if (msg->data.size() != 2) {
    return;
  }
  auto yaw_angle = msg->data[0];

  yaw_angle = std::clamp(yaw_angle, MIN_YAW_ANGLE, MAX_YAW_ANGLE);

  trajectory_msgs::msg::JointTrajectory jt_msg;
  jt_msg.joint_names.push_back("waist_yaw_joint");

  trajectory_msgs::msg::JointTrajectoryPoint jt_point_msg;
  jt_point_msg.positions.push_back(yaw_angle);
  jt_point_msg.time_from_start = rclcpp::Duration(TIME_FROM_START);
  jt_msg.points.push_back(jt_point_msg);

  jt_publisher_->publish(jt_msg);
}

}  // namespace foodly_rd_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(foodly_rd_examples::WaistJtControl)
