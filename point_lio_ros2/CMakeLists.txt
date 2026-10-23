cmake_minimum_required(VERSION 3.8)
project(point_lio_ros2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Point-LIO de-ROS fork 根目录（环境变量或 cmake 变量指定，Orin 上默认 ~/Point-LIO）
set(POINT_LIO_ROOT "$ENV{POINT_LIO_ROOT}" CACHE PATH "Point-LIO de-ROS fork root")
if("${POINT_LIO_ROOT}" STREQUAL "")
  set(POINT_LIO_ROOT "$ENV{HOME}/Point-LIO")
endif()
if(NOT EXISTS "${POINT_LIO_ROOT}/build/libpoint_lio_core.a")
  message(FATAL_ERROR "找不到 ${POINT_LIO_ROOT}/build/libpoint_lio_core.a — 先在 fork 里 cmake -B build -S . && cmake --build build")
endif()
message(STATUS "POINT_LIO_ROOT = ${POINT_LIO_ROOT}")

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
  ${POINT_LIO_ROOT}/build/libpoint_lio_core.a
  ${PCL_LIBRARIES}
  yaml-cpp pthread stdc++fs
)

install(TARGETS point_lio_node DESTINATION lib/${PROJECT_NAME})
ament_package()
