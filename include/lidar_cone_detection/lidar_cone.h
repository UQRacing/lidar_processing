// LiDAR cone detector (new version), main header
// Matt Young, 2022, UQRacing
#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_geometry/pinhole_camera_model.h>

namespace uqr {
    class LidarProcessing {
    public:
        explicit LidarProcessing(ros::NodeHandle &handle);

    private:
        ros::Subscriber lidarSub, cameraInfoSub;
        ros::Publisher lidarDebugPub, lidarDepthPub;

        // YAML parameters
        std::string lidarTopicName{};
        std::string lidarDebugTopicName{};
        std::string cameraInfoTopicName{};
        std::string lidarDepthTopicName{};

        ddynamic_reconfigure::DDynamicReconfigure ddr{};

        std::optional<image_geometry::PinholeCameraModel> camera{};
        std::optional<sensor_msgs::CameraInfo> cameraInfo{};

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud);
        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    };
}