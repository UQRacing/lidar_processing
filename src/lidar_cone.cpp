// LiDAR cone detector, main file
// Matt Young (with much prior work done by Riley Bowyer & Caleb Aitken), 2022, UQRacing
#include "lidar_cone_detection/lidar_cone.hpp"
#include <ros/ros.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_cone");
    ROS_INFO("Hello, world!");
    ros::spin();
}