cmake_minimum_required(VERSION 3.5)
project(slam_yplidar_x2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
