cmake_minimum_required(VERSION 3.0.2)
project(pmf_ground_filter)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(PCL 1.8 REQUIRED)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(pmf_ground_filter_node src/pmf_ground_filter_core.cpp src/pmf_ground_filter_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pmf_ground_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Add dependencies to the executable
add_dependencies(pmf_ground_filter_node ${catkin_EXPORTED_TARGETS})
