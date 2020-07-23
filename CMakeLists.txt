cmake_minimum_required(VERSION 3.0.2)
project(lidar_cones_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  tf2
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(DEPENDS
  roscpp
  pcl_ros
  sensor_msgs
  tf2
  INCLUDE_DIRS include
  LIBRARIES PCLWrapper IOUtility OnlineSegment
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# i commented these out because I don't know why they are needed? Please explain sir!?
# They're generalised variants of the ones below. It's better practice to do what we have in target_link_libraries.
#link_directories(${pcl_ros_LIBRARY_DIRS})
#add_definitions(${pcl_ros_DEFINITIONS})

# gotta link those pcl_ros_LIBRARIES or the whole thing breaks
# also the wrapper is being compiled as a library not an executable for obvious reasons
add_library(PCLWrapper STATIC src/PCLWrapper.cpp)
target_link_libraries(PCLWrapper ${pcl_ros_LIBRARIES})

add_library(IOUtility STATIC src/IOUtility.cpp)
target_link_libraries(IOUtility ${pcl_ros_LIBRARIES} PCLWrapper)

add_library(OnlineSegment STATIC src/OnlineSegmentation.cpp)
target_link_libraries(OnlineSegment ${pcl_ros_LIBRARIES} ${Eigen3_LIBRARIES} PCLWrapper IOUtility)

# FIXME: delete me; this is for testing purposed
add_executable(testing src/test.cpp)
target_link_libraries(testing ${catkin_LIBRARIES} ${PCL_LIBRARIES} PCLWrapper IOUtility)

add_executable(segmentTesting src/segmentTest.cpp)
target_link_libraries(segmentTesting ${catkin_LIBRARIES} ${PCL_LIBRARIES} OnlineSegment PCLWrapper IOUtility)