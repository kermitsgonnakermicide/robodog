cmake_minimum_required(VERSION 3.5)
project(leg_walker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(ik_solver_lib src/ik_solver.cpp)
ament_target_dependencies(ik_solver_lib rclcpp geometry_msgs sensor_msgs Eigen3)

add_executable(walk_node src/walk_node.cpp)
target_link_libraries(walk_node ik_solver_lib)
ament_target_dependencies(walk_node rclcpp geometry_msgs sensor_msgs Eigen3)

install(TARGETS walk_node ik_solver_lib DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch include DESTINATION share/${PROJECT_NAME})

ament_package()
