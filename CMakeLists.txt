cmake_minimum_required(VERSION 3.16.0)
project(lidar_processing)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(uqr_msgs REQUIRED)
#find_package(fssim_messages REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(ddynamic_reconfigure REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)

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

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
ament_export_include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(lidar_processing src/lidar_processing.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_processing
    ${rclcpp_LIBRARIES}
    Eigen3::Eigen
    ${Boost_LIBRARY_DIR}
    ${OpenCV_LIBS}
    OpenMP::OpenMP_CXX
)

#Removed fssim_messages and ddynamic_reconfigure from dependencies list because I don't have them so I can test whether this will build - you can add it back if it's needed and works in ros2
ament_export_dependencies(roscpp rospy std_msgs uqr_msgs geometry_msgs sensor_msgs
    visualization_msgs cv_bridge image_geometry)
    
ament_package()
