// Copyright 2024 RT Corporation

#include "foodly_rd_examples/color_detection.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
using std::placeholders::_1;

namespace foodly_rd_examples
{

ColorDetection::ColorDetection(const rclcpp::NodeOptions & options)
: Node("color_detection", options)
{
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&ColorDetection::image_callback, this, _1));

  image_annotated_publisher_ =
    this->create_publisher<sensor_msgs::msg::Image>("image_annotated", 10);

  object_point_publisher_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>("target_position", 10);
}

void ColorDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Track a orange object
  const int LOW_H = 5, HIGH_H = 20;
  const int LOW_S = 120, HIGH_S = 255;
  const int LOW_V = 120, HIGH_V = 255;

  const auto MIN_OBJECT_SIZE = msg->width * msg->height * 0.01;

  auto cv_img = cv_bridge::toCvShare(msg, msg->encoding);

  cv::Mat img_hsv;
  cv::cvtColor(cv_img->image, img_hsv, cv::COLOR_RGB2HSV);

  cv::Mat img_thresholded;
  cv::inRange(
    img_hsv,
    cv::Scalar(LOW_H, LOW_S, LOW_V),
    cv::Scalar(HIGH_H, HIGH_S, HIGH_V),
    img_thresholded);

  cv::morphologyEx(
    img_thresholded,
    img_thresholded,
    cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

  cv::morphologyEx(
    img_thresholded,
    img_thresholded,
    cv::MORPH_CLOSE,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

  cv::Mat img_annotated;
  cv_img->image.copyTo(img_annotated, img_thresholded);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(img_thresholded, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  if (contours.size()) {
    std::vector<cv::Moments> object_moments;
    int max_area_i = -1;
    int i = 0;
    for (const auto & contour : contours) {
      object_moments.push_back(cv::moments(contour));
      if (object_moments[max_area_i].m00 < object_moments[i].m00) {
        max_area_i = i;
      }
      i++;
    }

    if (object_moments[max_area_i].m00 > MIN_OBJECT_SIZE) {
      cv::Point2d object_point;
      object_point.x = object_moments[max_area_i].m10 / object_moments[max_area_i].m00;
      object_point.y = object_moments[max_area_i].m01 / object_moments[max_area_i].m00;

      RCLCPP_DEBUG_STREAM(this->get_logger(), "Detect at" << object_point << ".");

      const cv::Scalar ANNOTATE_COLOR(256, 0, 256);
      const int ANNOTATE_THICKNESS = 4;
      const int ANNOTATE_RADIUS = 10;
      cv::drawContours(img_annotated, contours, max_area_i, ANNOTATE_COLOR, ANNOTATE_THICKNESS);
      cv::circle(img_annotated, object_point, ANNOTATE_RADIUS, ANNOTATE_COLOR, -1);

      cv::Point2d translated_object_point;
      translated_object_point.x = object_point.x - msg->width / 2.0;
      translated_object_point.y = object_point.y - msg->height / 2.0;

      // Normalize object position from -1.0 to 1.0
      cv::Point2d normalized_object_point_;
      if (msg->width != 0 && msg->height != 0) {
        normalized_object_point_.x = translated_object_point.x / (msg->width / 2.0);
        normalized_object_point_.y = translated_object_point.y / (msg->height / 2.0);
      }

      geometry_msgs::msg::PointStamped object_point_msg;
      object_point_msg.header = msg->header;
      object_point_msg.point.x = normalized_object_point_.x;
      object_point_msg.point.y = normalized_object_point_.y;
      object_point_publisher_->publish(object_point_msg);
    }
  }

  sensor_msgs::msg::Image::SharedPtr img_annotated_msg =
    cv_bridge::CvImage(msg->header, msg->encoding, img_annotated).toImageMsg();
  image_annotated_publisher_->publish(*img_annotated_msg);
}

}  // namespace foodly_rd_examples

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(foodly_rd_examples::ColorDetection)
