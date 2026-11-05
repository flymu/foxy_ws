cmake_minimum_required(VERSION 3.8)
project(point_lio_ros2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point-LIO de-ROS fork 根目录（环境变量或 cmake 变量指定，Orin 上默认 ~/Point-LIO）
set(POINT_LIO_ROOT "$ENV{POINT_LIO_ROOT}" CACHE PATH "Point-LIO de-ROS fork root")
if("${POINT_LIO_ROOT}" STREQUAL "")
  set(POINT_LIO_ROOT "$ENV{HOME}/Point-LIO")
endif()
# 核心库 build 目录可覆盖：宿主机(GCC11+PCL1.15)与容器(GCC9+PCL1.10)的 ABI 不兼容
# （boost/std shared_ptr 之差），容器内统一用 build_foxy，不与宿主机 build/ 混用
set(POINT_LIO_BUILD "$ENV{POINT_LIO_BUILD}" CACHE PATH "Point-LIO core build dir")
if("${POINT_LIO_BUILD}" STREQUAL "")
  if(EXISTS "${POINT_LIO_ROOT}/build_foxy/libpoint_lio_core.a")
    set(POINT_LIO_BUILD "${POINT_LIO_ROOT}/build_foxy")  # 容器编的优先
  else()
    set(POINT_LIO_BUILD "${POINT_LIO_ROOT}/build")
  endif()
endif()
if(NOT EXISTS "${POINT_LIO_BUILD}/libpoint_lio_core.a")
  message(FATAL_ERROR "找不到 ${POINT_LIO_BUILD}/libpoint_lio_core.a — 容器内: cmake -B build_foxy -S ${POINT_LIO_ROOT} && cmake --build build_foxy")
endif()
message(STATUS "POINT_LIO_ROOT = ${POINT_LIO_ROOT}, BUILD = ${POINT_LIO_BUILD}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

# Foxy 容器里 libyaml-cpp 只装在 yaml_cpp_vendor 的非标准路径（系统目录没有），
# 裸 -lyaml-cpp 会报 "cannot find"。两个节点实际用自研 ConfigReader，不引用
# yaml-cpp 符号（--as-needed 会把 NEEDED 丢掉），但 ld 仍要求链接期能找到文件。
if(EXISTS "/opt/ros/foxy/opt/yaml_cpp_vendor/lib/libyaml-cpp.so")
  link_directories("/opt/ros/foxy/opt/yaml_cpp_vendor/lib")
endif()

add_executable(map_localization_node src/map_localization_node.cpp)
target_include_directories(map_localization_node PRIVATE
  ${POINT_LIO_ROOT}/src
  ${POINT_LIO_ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(map_localization_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros pcl_conversions)
target_link_libraries(map_localization_node
  ${PCL_LIBRARIES} yaml-cpp pthread stdc++fs)
install(TARGETS map_localization_node DESTINATION lib/${PROJECT_NAME})

add_executable(point_lio_node src/point_lio_node.cpp)
target_include_directories(point_lio_node PRIVATE
  ${POINT_LIO_ROOT}/src
  ${POINT_LIO_ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(point_lio_node
  rclcpp livox_ros_driver2 sensor_msgs nav_msgs geometry_msgs tf2_ros pcl_conversions)
target_link_libraries(point_lio_node
  ${POINT_LIO_BUILD}/libpoint_lio_core.a
  ${PCL_LIBRARIES}
  yaml-cpp pthread stdc++fs
)

install(TARGETS point_lio_node DESTINATION lib/${PROJECT_NAME})
ament_package()
