// LiDAR cone detector (new version), main header
// Matt Young, 2022, UQRacing
#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace uqr {
    class LidarConeDetector {
    public:
        explicit LidarConeDetector(ros::NodeHandle &handle);

    private:
        ros::Subscriber lidarSub; // subscribe to cones
        ros::Publisher conePub, lidarDebugPub, detectDebugPub; // publish detected cones

        // YAML parameters
        std::string lidarTopicName{};
        std::string lidarDebugTopicName{}, detectDebugTopicName{};
        bool enableDebugUI{};

        // RANSAC ground plane fit parameters
        double planeDistThresh{};
        int32_t planeRansacN{}, planeNumIters{};

        // Crop bounding bbox
        double bboxMinX{}, bboxMinY{}, bboxMinZ{}, bboxMaxX{}, bboxMaxY{}, bboxMaxZ{};

        // Voxeliser parameters
        double voxeliserResolution{};

        // DBSCAN clustering parameters
        double dbscanEps{};
        int32_t dbscanMinPts{};

        ddynamic_reconfigure::DDynamicReconfigure ddr{};

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud);
    };
}