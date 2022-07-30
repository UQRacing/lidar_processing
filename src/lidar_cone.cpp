// LiDAR cone detector (new version), main file
// Matt Young, 2022, UQRacing
#include "lidar_cone_detection/lidar_cone.h"
#include "lidar_cone_detection/defines.h"
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "lidar_cone_detection/open3d_conversions.h"
#include <open3d/Open3D.h>
#include <visualization_msgs/MarkerArray.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace uqr;

// very lazily borrowed and reformatted from https://www.codespeedy.com/hsv-to-rgb-in-cpp/
static Eigen::Matrix<double, 3, 1> hsvToRgb(double H, double S, double V) {
    if (H > 360 || H < 0 || S > 100 || S < 0 || V > 100 || V < 0) {
        throw std::invalid_argument("Invalid HSV");
    }
    double s = S / 100;
    double v = V / 100;
    double C = s * v;
    double X = C * (1.0 - std::abs(std::fmod(H / 60.0, 2.0) - 1.0));
    double m = v - C;
    double r, g, b;
    if (H >= 0 && H < 60) {
        r = C, g = X, b = 0;
    } else if (H >= 60 && H < 120) {
        r = X, g = C, b = 0;
    } else if (H >= 120 && H < 180) {
        r = 0, g = C, b = X;
    } else if (H >= 180 && H < 240) {
        r = 0, g = X, b = C;
    } else if (H >= 240 && H < 300) {
        r = X, g = 0, b = C;
    } else {
        r = C, g = 0, b = X;
    }
    Eigen::Matrix<double, 3, 1> out;
    out << (r + m), (g + m), (b + m);
    return out;
}

LidarConeDetector::LidarConeDetector(ros::NodeHandle &handle) {
    if (!handle.getParam("/lidar_cone_detector/lidar_topic", lidarTopicName)) {
        ROS_ERROR("Failed to load lidar_topic from lidar cone config YAML");
    }
    handle.getParam("/lidar_cone_detector/debug_ui", enableDebugUI);
    handle.getParam("/lidar_cone_detector/lidar_debug_pub", lidarDebugTopicName);
    handle.getParam("/lidar_cone_detector/detect_debug_pub", detectDebugTopicName);

    // TODO load camera parameters from YAML

    lidarSub = handle.subscribe(lidarTopicName, 1, &LidarConeDetector::lidarCallback, this);
    // TODO conePub (figure out what message type we publish first, a point cloud?? bounding box?)

    if (!lidarDebugTopicName.empty()) {
        ROS_INFO("Publishing lidar debug to topic %s", lidarDebugTopicName.c_str());
        lidarDebugPub = handle.advertise<sensor_msgs::PointCloud2>(lidarDebugTopicName, 1);
    } else {
        ROS_INFO("Lidar debug publishing disabled by config");
    }

    if (!detectDebugTopicName.empty()) {
        ROS_INFO("Publishing detect debug to topic %s", detectDebugTopicName.c_str());
        detectDebugPub = handle.advertise<visualization_msgs::MarkerArray>(detectDebugTopicName, 1);
    } else {
        ROS_INFO("Lidar detect debug publishing disabled by config");
    }

    if (enableDebugUI) {
        // in case we want to do Open3D based debugging
    }
}

void LidarConeDetector::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud) {
    auto start = ros::WallTime::now();

    open3d::t::geometry::PointCloud cloud;
    open3d_conversions::rosToOpen3d(rosCloud, cloud);

    // FIXME load camera parameters (what is a good way to do this?)
    //  subscribe to topic /camera/color/camera_info? (then what about when we don't use the RealSense)
    //  use this: https://wiki.ros.org/camera_calibration_parsers ?
    auto intrinsic = open3d::core::Tensor();
    auto extrinsic = open3d::core::Tensor();

    // project lidar points onto camera plane to generate depth image
    // TODO tune the last two parameters: depth scale and max depth
    // TODO we could use opencv for this, cv::projectPoints, but the arguments are more confusing
    auto depthImage = cloud.ProjectToDepthImage(1280, 720,
                                                intrinsic, extrinsic, 1000.0f, 30.0f);

    // convert Open3D depth image to OpenCV

    // interpolate the depth image (inpaint?)

    // publish the depth image over ROS

    double time = (ros::WallTime::now() - start).toSec() * 1000.0;
    ROS_INFO("Lidar callback time: %.2f ms", time);
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_process");
    ROS_INFO("LiDAR Processing v" LIDAR_CONE_VERSION ": Matt Young, Fahed Alhanaee, Riley Bowyer, Caleb Aitken, 2021-2022, UQRacing");
    open3d::PrintOpen3DVersion();
    auto cvCpuFeatures = cv::getCPUFeaturesLine();
    auto cvNumThreads = cv::getNumThreads();
    auto cvVersion = cv::getVersionString();
    ROS_INFO("Using OpenCV v%s with %d threads, features: %s", cvVersion.c_str(), cvNumThreads, cvCpuFeatures.c_str());

    ros::NodeHandle handle{};
    LidarConeDetector detector = LidarConeDetector(handle);
    ROS_INFO("Initialised ROS node");

    ros::spin();
}