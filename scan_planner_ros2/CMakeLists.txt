cmake_minimum_required(VERSION 3.8)
project(scan_planner_ros2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SCAN-Planner standalone 根目录（含 include/ 与 build_foxy/libscan_planner_core.a）
set(SCAN_PLANNER_ROOT "$ENV{SCAN_PLANNER_ROOT}" CACHE PATH "SCAN-Planner standalone root")
if("${SCAN_PLANNER_ROOT}" STREQUAL "")
  set(SCAN_PLANNER_ROOT "$ENV{HOME}/workspace/SCAN-Planner/standalone")
endif()
# 独立 build 目录：宿主机 GCC11 与容器 GCC9 的缓存不能混用
set(SCAN_PLANNER_BUILD "$ENV{SCAN_PLANNER_BUILD}" CACHE PATH "standalone build dir")
if("${SCAN_PLANNER_BUILD}" STREQUAL "")
  set(SCAN_PLANNER_BUILD "${SCAN_PLANNER_ROOT}/build_foxy")
endif()
if(NOT EXISTS "${SCAN_PLANNER_BUILD}/libscan_planner_core.a")
  message(FATAL_ERROR "找不到 ${SCAN_PLANNER_BUILD}/libscan_planner_core.a — 先 cmake -B build_foxy -S <standalone> && cmake --build build_foxy --target scan_planner_core")
endif()
message(STATUS "SCAN_PLANNER_ROOT = ${SCAN_PLANNER_ROOT}, BUILD = ${SCAN_PLANNER_BUILD}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_executable(scan_planner_node src/scan_planner_node.cpp)
target_include_directories(scan_planner_node PRIVATE
  ${SCAN_PLANNER_ROOT}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(scan_planner_node
  rclcpp nav_msgs sensor_msgs geometry_msgs pcl_conversions)
target_link_libraries(scan_planner_node
  ${SCAN_PLANNER_BUILD}/libscan_planner_core.a
  ${PCL_LIBRARIES} pthread
)
install(TARGETS scan_planner_node DESTINATION lib/${PROJECT_NAME})
ament_package()
