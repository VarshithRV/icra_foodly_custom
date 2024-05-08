// Copyright 2024 RT Corporation

#ifndef FOODLY_RD_EXAMPLES__OBJECT_TRACKER_HPP_
#define FOODLY_RD_EXAMPLES__OBJECT_TRACKER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace foodly_rd_examples
{

class ObjectTracker : public rclcpp::Node
{
public:
  explicit ObjectTracker(const rclcpp::NodeOptions & options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_point_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angles_publisher_;
  control_msgs::msg::JointTrajectoryControllerState::SharedPtr current_angles_msg_;
  geometry_msgs::msg::PointStamped::SharedPtr object_point_msg_;
  std::vector<double> target_angles_;

  void state_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void tracking();
};

}  // namespace foodly_rd_examples

#endif  // FOODLY_RD_EXAMPLES__OBJECT_TRACKER_HPP_
