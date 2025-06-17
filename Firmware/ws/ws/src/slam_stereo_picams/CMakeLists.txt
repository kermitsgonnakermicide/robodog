cmake_minimum_required(VERSION 3.5)
project(servo_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(servo_driver_node
  src/servo_driver_node.cpp
  src/pigpio_pwm_controller.cpp
)
target_link_libraries(servo_driver_node pigpio)

install(TARGETS servo_driver_node DESTINATION lib/${PROJECT_NAME})


include_directories(include)

add_executable(servo_driver_node src/servo_driver_node.cpp src/pwm_controller.cpp)
ament_target_dependencies(servo_driver_node rclcpp sensor_msgs)

install(TARGETS servo_driver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch include DESTINATION share/${PROJECT_NAME})

ament_package()
