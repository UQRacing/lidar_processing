// LiDAR cone detector, main header
// Matt Young (with much prior work done by Riley Bowyer & Caleb Aitken), 2022, UQRacing
#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cilantro/cilantro.hpp>

namespace uqr {
    class LidarConeDetector {
    public:
        explicit LidarConeDetector(ros::NodeHandle &handle);

    private:
        ros::Subscriber lidarSub; // subscribe to cones
        ros::Publisher conePub; // publish detected cones

        std::string lidarTopicName;
        bool enableDebugUI{};

        // FIXME this is absolutely atrocious, find a better way to fake-init "viz" in case debug is disabled
        // or, since the visualiser doesn't appear to work AT ALL, maybe disable it entirely?
        std::optional<cilantro::Visualizer> viz;

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud);
    };
}