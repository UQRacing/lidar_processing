cmake_minimum_required(VERSION 3.0.2)
project(lidar_cones_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf2
)

catkin_package(DEPENDS
  roscpp
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf2
  INCLUDE_DIRS include
  LIBRARIES PCLWrapper
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(PCLWrapper STATIC src/PCLWrapper.cpp)

add_executable(cloudFilterSample src/cloudFilter.cpp)
target_link_libraries(cloudFilterSample ${catkin_LIBRARIES})

# FIXME: delete me; this is for testing purposed
add_executable(testing src/test.cpp)
target_link_libraries(testing ${catkin_LIBRARIES} PCLWrapper)