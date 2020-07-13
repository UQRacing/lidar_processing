cmake_minimum_required(VERSION 3.0.2)
project(lidar_cones_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf2
)

catkin_package( DEPENDS
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf2
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloudFilterSample src/cloudFilter.cpp)
target_link_libraries(cloudFilterSample ${catkin_LIBRARIES})