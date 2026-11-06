cmake_minimum_required(VERSION 3.8)
project(obstacle_veto_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

# 否决层节点（生产名 obstacle_veto；影子模式用 -r 换名换输出话题）
add_executable(obstacle_veto_node src/obstacle_veto_node.cpp)
ament_target_dependencies(obstacle_veto_node
  rclcpp sensor_msgs geometry_msgs std_msgs)

# 纯逻辑回归（R1-R3 38 断言，仅 Eigen，狗不上电可跑）
add_executable(veto_core_test test/veto_core_test.cpp)

install(TARGETS obstacle_veto_node veto_core_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
