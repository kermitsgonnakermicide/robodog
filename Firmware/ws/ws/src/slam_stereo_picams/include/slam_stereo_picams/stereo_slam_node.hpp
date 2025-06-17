#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.h>

class StereoSLAMNode : public rclcpp::Node {
public:
  StereoSLAMNode();
  ~StereoSLAMNode();

private:
  void leftCB(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void rightCB(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
  void processStereo();

  bool got_left_{false}, got_right_{false};
  cv::Mat left_cv_, right_cv_;

  image_transport::CameraPublisher left_pub_, right_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  image_transport::SubscriberFilter left_sub_, right_sub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  double baseline_, angle_rad_;
  cv::Mat Q_;
};
