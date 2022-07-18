// LiDAR cone detector (new version), main header
// Matt Young, 2022, UQRacing
#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
// FIXME CLion doesn't detect this stupid ass fucking header
//#include <lidar_cone_detection/LidarConeConfig.h>

#ifdef __JETBRAINS_IDE__
// hack to load the full path of the header file here?
#endif

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