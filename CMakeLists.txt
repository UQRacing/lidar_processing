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

# i commented these out because I don't know why they are needed? Please explain sir!?
#link_directories(${pcl_ros_LIBRARY_DIRS})
#add_definitions(${pcl_ros_DEFINITIONS})

# gotta link those pcl_ros_LIBRARIES or the whole thing breaks
# also the wrapper is being compiled as a library not an executable for obvious reasons
add_library(PCLWrapper STATIC src/PCLWrapper.cpp)
target_link_libraries(PCLWrapper ${pcl_ros_LIBRARIES})

add_library(IOUtility STATIC src/IOUtility.cpp)
target_link_libraries(IOUtility ${pcl_ros_LIBRARIES} PCLWrapper)

add_executable(cloudFilterSample src/cloudFilter.cpp)
target_link_libraries(cloudFilterSample ${catkin_LIBRARIES})

# FIXME: delete me; this is for testing purposed
add_executable(testing src/test.cpp)
target_link_libraries(testing ${catkin_LIBRARIES} ${PCL_LIBRARIES} PCLWrapper IOUtility)
