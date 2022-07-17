// LiDAR cone detector, main header
// Matt Young (with much prior work done by Riley Bowyer & Caleb Aitken), 2022, UQRacing
#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace uqr {
    class LidarConeDetector {
    public:
        explicit LidarConeDetector(ros::NodeHandle &handle);

    private:
        ros::Subscriber lidarSub; // subscribe to cones
        ros::Publisher conePub, lidarDebugPub; // publish detected cones

        std::string lidarTopicName;
        std::string lidarDebugTopicName;
        bool enableDebugUI{};

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud);
    };
}