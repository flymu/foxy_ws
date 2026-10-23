cmake_minimum_required(VERSION 3.10)
project(hba LANGUAGES CXX)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fexceptions -pthread")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# gtsam is vendored under 3rdparty/gtsam (tag 4.1.1, the version HBA was
# developed against). Built as a static library, no TBB, no examples/tests.
# GTSAM_USE_SYSTEM_EIGEN: gtsam's bundled Eigen is 3.3 while this machine (and
# PCL/HBA) uses Eigen 3.4; mixing the two trips gtsam's version static_assert
# and would be an ODR hazard anyway.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(GTSAM_USE_TBB OFF CACHE BOOL "" FORCE)
set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "" FORCE)
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_UNOFFICIAL_TOOLS OFF CACHE BOOL "" FORCE)
set(GTSAM_THROW_ON_INVALID_POSE3_CONSTRUCTOR ON CACHE BOOL "" FORCE)
add_subdirectory(3rdparty/gtsam EXCLUDE_FROM_ALL)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Hierarchical Bundle Adjustment
add_executable(hba source/hba.cpp)
target_link_libraries(hba ${PCL_LIBRARIES} Threads::Threads gtsam)

# Assemble the per-scan pcds into one map using a pose file
# (build both the pre- and post-optimization map for comparison).
add_executable(assemble_map source/assemble_map.cpp)
target_link_libraries(assemble_map ${PCL_LIBRARIES} Threads::Threads)

# visualize.cpp (rviz-only) and calculate_MME.cpp (needs PCL kdtree +
# segmentation, not available in this PCL install) are not built here;
# use tools/mme.py for the MME metric instead.
