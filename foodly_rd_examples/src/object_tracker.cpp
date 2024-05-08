// Copyright 2024 RT Corporation

#include "foodly_rd_examples/object_tracker.hpp"

#include "angles/angles.h"

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace foodly_rd_examples
{

ObjectTracker::ObjectTracker(const rclcpp::NodeOptions & options)
: Node("object_tracker", options)
{
  timer_ = this->create_wall_timer(
    30ms, std::bind(&ObjectTracker::tracking, this));

  state_subscription_ =
    this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
    "/controller_state", 10, std::bind(&ObjectTracker::state_callback, this, _1));

  object_point_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "target_position", 10, std::bind(&ObjectTracker::point_callback, this, _1));

  angles_publisher_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("target_angles", 10);
}

void ObjectTracker::state_callback(
  const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
  current_angles_msg_ = msg;
}

void ObjectTracker::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  object_point_msg_ = msg;
}

void ObjectTracker::tracking()
{
  const double POSITION_THRESH = 0.1;

  const std::vector<double> INITIAL_ANGLES = {0, 0};

  const double MAX_YAW_ANGLE = angles::from_degrees(120);
  const double MIN_YAW_ANGLE = angles::from_degrees(-120);
  const double MAX_PITCH_ANGLE = angles::from_degrees(70);
  const double MIN_PITCH_ANGLE = angles::from_degrees(-33);

  const int WAIST_JOINT_NUM = 1;
  const int NECK_JOINT_NUM = 2;
  const double MAX_ANGULAR_DIFF = angles::from_degrees(1.5);
  const double RESET_ANGULAR_DIFF = angles::from_degrees(0.5);

  const std::chrono::nanoseconds DETECTION_TIMEOUT = 1s;

  const double OPERATION_GAIN = 0.02;

  bool look_object = false;

  if (!current_angles_msg_) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Waiting controller state.");
    return;
  }
  if (current_angles_msg_->feedback.positions.size() != WAIST_JOINT_NUM &&
    current_angles_msg_->feedback.positions.size() != NECK_JOINT_NUM)
  {
    return;
  }
  const auto current_angles = current_angles_msg_->feedback.positions;
  if (target_angles_.empty()) {
    target_angles_ = current_angles;
    if (target_angles_.size() == 1) {
      target_angles_.push_back(0);
    }
  }

  auto now = this->get_clock()->now().nanoseconds();

  if (object_point_msg_) {
    const auto detected_time = rclcpp::Time(object_point_msg_->header.stamp).nanoseconds();
    const auto POINT_ELAPSED_TIME = now - detected_time;
    look_object = POINT_ELAPSED_TIME < DETECTION_TIMEOUT.count();
  }

  if (look_object) {
    std::vector<double> object_position;
    object_position.push_back(object_point_msg_->point.x);
    object_position.push_back(-object_point_msg_->point.y);
    std::vector<double> diff_angles = {0, 0};

    // Calculate joint angles
    for (int i = 0; i < 2; i++) {
      if (std::abs(object_position[i]) > POSITION_THRESH) {
        diff_angles[i] = object_position[i] * OPERATION_GAIN;
        diff_angles[i] = std::clamp(diff_angles[i], -MAX_ANGULAR_DIFF, MAX_ANGULAR_DIFF);
        target_angles_[i] -= diff_angles[i];
      }
    }
  } else {
    // Return to initial angle
    std::vector<double> diff_angles = {0, 0};

    for (int i = 0; i < 2; i++) {
      diff_angles[i] = INITIAL_ANGLES[i] - target_angles_[i];
      if (std::abs(diff_angles[i]) > RESET_ANGULAR_DIFF) {
        target_angles_[i] += std::copysign(RESET_ANGULAR_DIFF, diff_angles[i]);
      } else {
        target_angles_[i] = INITIAL_ANGLES[i];
      }
    }
  }

  target_angles_[0] = std::clamp(target_angles_[0], MIN_YAW_ANGLE, MAX_YAW_ANGLE);
  target_angles_[1] = std::clamp(target_angles_[1], MIN_PITCH_ANGLE, MAX_PITCH_ANGLE);

  std_msgs::msg::Float64MultiArray target_angles_msg;
  target_angles_msg.data.push_back(target_angles_[0]);
  target_angles_msg.data.push_back(target_angles_[1]);
  angles_publisher_->publish(target_angles_msg);
}

}  // namespace foodly_rd_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(foodly_rd_examples::ObjectTracker)
