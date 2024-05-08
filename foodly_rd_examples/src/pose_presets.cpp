// Copyright 2024 RT Corporation

#include "pose_presets.hpp"
#include "angles/angles.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pose_presets
{
geometry_msgs::msg::Pose generate_pose(
  const double x, const double y, const double z,
  const double roll, const double pitch, const double yaw)
{
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  q.setRPY(roll, pitch, yaw);
  target_pose.orientation = tf2::toMsg(q);
  return target_pose;
}

geometry_msgs::msg::Pose right_arm_downward(const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose = generate_pose(
    x, y, z,
    angles::from_degrees(90), angles::from_degrees(0), angles::from_degrees(0));
  return target_pose;
}

geometry_msgs::msg::Pose left_arm_downward(const double x, const double y, const double z)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose = generate_pose(
    x, y, z,
    angles::from_degrees(-90), angles::from_degrees(0), angles::from_degrees(0));
  return target_pose;
}
}  // namespace pose_presets
