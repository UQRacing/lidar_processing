cmake_minimum_required(VERSION 3.16.0)
project(lidar_cone_detection)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -g3)
#add_link_options(-fuse-ld=lld)

# Optimisation
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Release build, enabling performance")
    add_compile_options(-O3 -march=native -mtune=native -flto)
    add_link_options(-flto)
else()
    message(STATUS "Debug build, enabling sanitizers")
    add_compile_options(-Og -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address -fsanitize=undefined)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    uqr_msgs
    fssim_messages
    geometry_msgs
    sensor_msgs
    visualization_msgs
    ddynamic_reconfigure
    cv_bridge
    image_geometry
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS ${OMPL_INCLUDE_DIRS} ${GEOS_INCLUDE_DIRS}
#  LIBRARIES lidar_cone_detection
  CATKIN_DEPENDS roscpp rospy std_msgs uqr_msgs fssim_messages geometry_msgs sensor_msgs
    visualization_msgs ddynamic_reconfigure cv_bridge image_geometry
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(lidar_cone_detection src/lidar_cone.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_cone_detection
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${Boost_LIBRARY_DIR}
    ${OpenCV_LIBS}
    OpenMP::OpenMP_CXX
)