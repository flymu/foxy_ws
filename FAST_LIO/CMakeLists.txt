cmake_minimum_required(VERSION 3.10)
project(fast_lio LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# The multithreaded residual computation is tuned to the core count, matching
# the upstream behaviour.
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message(STATUS "Processor number: ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN -DMP_PROC_NUM=3)
    message(STATUS "core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN -DMP_PROC_NUM=2)
    message(STATUS "core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

message(STATUS "Eigen: ${EIGEN3_INCLUDE_DIR}")

# ---------------------------------------------------------------------------
# Core library: no ROS, no I/O.
# ---------------------------------------------------------------------------
add_library(fastlio
  src/fast_lio.cpp
  src/config.cpp
  src/preprocess.cpp
  include/ikd-Tree/ikd_Tree.cpp
)

target_include_directories(fastlio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(fastlio SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(fastlio PUBLIC ${PCL_LIBRARIES} Threads::Threads)
target_link_directories(fastlio PUBLIC ${PCL_LIBRARY_DIRS})
target_compile_definitions(fastlio PUBLIC ${PCL_DEFINITIONS})

# yaml-cpp exports an imported target in newer releases and only variables in
# older ones.
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(fastlio PRIVATE yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  target_link_libraries(fastlio PRIVATE yaml-cpp)
else()
  target_include_directories(fastlio PRIVATE ${YAML_CPP_INCLUDE_DIR})
  target_link_libraries(fastlio PRIVATE ${YAML_CPP_LIBRARIES})
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlio PUBLIC OpenMP::OpenMP_CXX)
endif()

# ---------------------------------------------------------------------------
# Offline runner
# ---------------------------------------------------------------------------
add_executable(fastlio_offline examples/offline_player.cpp)
target_link_libraries(fastlio_offline PRIVATE fastlio)

# Exports per-scan LiDAR-frame clouds + pose.json for the HBA mapper.
add_executable(fastlio_export_hba examples/export_hba_scans.cpp)
target_link_libraries(fastlio_export_hba PRIVATE fastlio)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
install(TARGETS fastlio fastlio_offline
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES
  include/fast_lio.h
  include/lio_types.h
  include/lio_log.h
  include/voxel_grid.h
  include/common_lib.h
  include/so3_math.h
  include/use-ikfom.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(DIRECTORY include/IKFoM_toolkit DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
