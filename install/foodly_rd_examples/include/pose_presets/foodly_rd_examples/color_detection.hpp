// Copyright 2024 RT Corporation

#ifndef FOODLY_RD_EXAMPLES__COLOR_DETECTION_HPP_
#define FOODLY_RD_EXAMPLES__COLOR_DETECTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

namespace foodly_rd_examples
{

class ColorDetection : public rclcpp::Node
{
public:
  explicit ColorDetection(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_annotated_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr object_point_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  // namespace foodly_rd_examples

#endif  // FOODLY_RD_EXAMPLES__COLOR_DETECTION_HPP_
