#include "slam_stereo_picams/stereo_slam_node.hpp"

StereoSLAMNode::StereoSLAMNode()
: Node("stereo_slam_node"),
  left_pub_(this, "left/image_rect"),
  right_pub_(this, "right/image_rect"),
  left_sub_(image_transport::ImageTransport(this).subscribeCamera("left/image_raw", 10,
            std::bind(&StereoSLAMNode::leftCB, this, std::placeholders::_1), this)),
  right_sub_(image_transport::ImageTransport(this).subscribeCamera("right/image_raw", 10,
            std::bind(&StereoSLAMNode::rightCB, this, std::placeholders::_1), this)),
  tf_broadcaster_(this)
{
  auto cfg = this->declare_parameter("baseline", 0.05);
  baseline_ = this->get_parameter("baseline").as_double();

  auto angle = this->declare_parameter("angle", 5.0) * M_PI / 180.0;
  angle_rad_ = angle;

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("stereo_pose", 10);
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("stereo_map", 1);

  Q_ = cv::Mat::eye(4,4,CV_64F);
  Q_.at<double>(0,3) = -cx();
  Q_.at<double>(1,3) = -cy();
  Q_.at<double>(0,0) = 1/fx();
  Q_.at<double>(1,1) = 1/fy();
  Q_.at<double>(0,3) = (cx()-cx())/baseline_;
}

void StereoSLAMNode::leftCB(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
  left_cv_ = cv_bridge::toCvShare(msg, "mono8")->image;
  got_left_ = true;
  if(got_right_) processStereo();
}

void StereoSLAMNode::rightCB(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
  right_cv_ = cv_bridge::toCvShare(msg, "mono8")->image;
  got_right_ = true;
  if(got_left_) processStereo();
}

void StereoSLAMNode::processStereo(){
  cv::Mat disp, disp8;
  cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(0,16,3);
  stereo->compute(left_cv_, right_cv_, disp);
  disp.convertTo(disp8, CV_8U, 255/(16*16.));

  // Publish rectified images (for debugging/visualization)
  sensor_msgs::msg::Image::SharedPtr disp_msg = cv_bridge::CvImage(
    std_msgs::msg::Header(), "mono8", disp8
  ).toImageMsg();
  right_pub_.publish(disp_msg);

  // Dummy pose and map publish to complete the interface
  auto pose = geometry_msgs::msg::PoseStamped();
  pose.header.stamp = now();
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1;
  pose_pub_->publish(pose);

  auto grid = nav_msgs::msg::OccupancyGrid();
  grid.header = pose.header;
  grid.info.resolution = 0.05;
  grid.info.width = disp8.cols;
  grid.info.height = disp8.rows;
  grid.data.assign(disp8.total(), -1);
  map_pub_->publish(grid);

  got_left_ = got_right_ = false;
}

StereoSLAMNode::~StereoSLAMNode() {}
