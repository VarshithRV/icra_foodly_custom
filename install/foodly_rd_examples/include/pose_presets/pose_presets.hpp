// Copyright 2024 RT Corporation

#ifndef POSE_PRESETS_HPP_
#define POSE_PRESETS_HPP_

#include "geometry_msgs/msg/pose.hpp"

namespace pose_presets
{
geometry_msgs::msg::Pose generate_pose(
  const double x, const double y, const double z,
  const double roll, const double pitch, const double yaw);
geometry_msgs::msg::Pose right_arm_downward(const double x, const double y, const double z);
geometry_msgs::msg::Pose left_arm_downward(const double x, const double y, const double z);
}  // namespace pose_presets

#endif  // POSE_PRESETS_HPP_
