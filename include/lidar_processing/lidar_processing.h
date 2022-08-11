// LiDAR processing (new version), main header
// Matt Young, 2022, UQRacing
#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_geometry/pinhole_camera_model.h>

#define DEG_RAD 0.017453293
#define RAD_DEG 57.29578

namespace uqr {
    class LidarProcessing {
    public:
        explicit LidarProcessing(ros::NodeHandle &handle);

    private:
        ros::Subscriber lidarSub, cameraInfoSub, cameraFrameSub;
        ros::Publisher lidarDepthPub;
        std::optional<ros::Publisher> inpaintingDebugPub;

        // YAML parameters
        std::string lidarTopicName{};
        std::string lidarDebugTopicName{};
        std::string cameraInfoTopicName{};
        std::string lidarDepthTopicName{};

        // pipeline configuration
        bool inpainting = false, morphological = false, publishColour = false;

        ddynamic_reconfigure::DDynamicReconfigure ddr{};

        std::optional<image_geometry::PinholeCameraModel> camera{};
        std::optional<sensor_msgs::CameraInfo> cameraInfo{};
        std::optional<cv::Mat> lastCameraFrame{}; // only used for debug

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud);
        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
        void cameraFrameCallback(const sensor_msgs::CompressedImageConstPtr &image);
    };
}