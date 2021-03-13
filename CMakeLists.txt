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

add_library(UQRPointCloud STATIC 
            src/UQRPointCloud.cpp)
target_link_libraries(UQRPointCloud 
                      ${pcl_ros_LIBRARIES})


add_library(IOUtility STATIC 
            src/IOUtility.cpp)
          
target_link_libraries(IOUtility 
                      ${pcl_ros_LIBRARIES} 
                      UQRPointCloud)


add_executable(planeDetection
               src/planeDetectionNode.cpp
               src/planeDetector.cpp)

target_link_libraries(planeDetection 
                      ${pcl_ros_LIBRARIES}
                      UQRPointCloud)

add_executable(binDetection
               src/binDetectionNode.cpp
               src/binDetector.cpp)
       
target_link_libraries(binDetection 
                      ${pcl_ros_LIBRARIES}
                      UQRPointCloud)

#add_executable(cloudFilterSample src/cloudFilter.cpp)
#target_link_libraries(cloudFilterSample ${catkin_LIBRARIES})