cmake_minimum_required(VERSION 3.5)
project(livox_relay)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(relay_node src/relay_node.cpp)
ament_target_dependencies(relay_node rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS relay_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
