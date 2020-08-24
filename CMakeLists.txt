cmake_minimum_required(VERSION 3.0.2)
project(lidar_cones_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  cv_bridge
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(DEPENDS
  roscpp
  pcl_ros
  sensor_msgs
  tf2
  cv_bridge
  INCLUDE_DIRS include
  LIBRARIES PCLWrapper IOUtility ProjectionParameters Projector
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

###############
## Libraries ##
###############

add_library(PCLWrapper STATIC src/PCLWrapper.cpp)
target_link_libraries(PCLWrapper ${pcl_ros_LIBRARIES})

add_library(IOUtility STATIC src/IOUtility.cpp)
target_link_libraries(IOUtility ${pcl_ros_LIBRARIES} PCLWrapper)

add_library(ProjectionParameters STATIC src/ProjectionParameters.cpp)

add_library(Projector STATIC src/Projector.cpp)
target_link_libraries(Projector ${OpenCV_LIBRARIES} PCLWrapper ProjectionParameters)

################
## Executable ##
################

# FIXME: delete me; this is for testing purposed
add_executable(testing src/test.cpp)
target_link_libraries(testing ${catkin_LIBRARIES} ${PCL_LIBRARIES} PCLWrapper IOUtility)

add_executable(segmentTesting src/segmentTest.cpp)
target_link_libraries(segmentTesting ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} Projector ProjectionParameters PCLWrapper IOUtility)