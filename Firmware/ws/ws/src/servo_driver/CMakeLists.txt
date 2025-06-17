cmake_minimum_required(VERSION 3.5)
project(slam_stereo_picams)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(stereo_slam_node src/stereo_slam_node.cpp)
ament_target_dependencies(stereo_slam_node
  rclcpp sensor_msgs image_transport cv_bridge OpenCV geometry_msgs nav_msgs tf2_ros
)

install(TARGETS stereo_slam_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch include DESTINATION share/${PROJECT_NAME})

ament_package()
