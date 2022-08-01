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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace uqr;
using namespace open3d;

typedef std::pair<cv::Point2d, double> CamPointPair_t;

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

LidarProcessing::LidarProcessing(ros::NodeHandle &handle) {
    if (!handle.getParam("/lidar_processing/lidar_topic", lidarTopicName)) {
        ROS_ERROR("Failed to load lidar_topic from lidar cone config YAML");
    }
    handle.getParam("/lidar_processing/lidar_debug_pub", lidarDebugTopicName);
    handle.getParam("/lidar_processing/camera_info_topic", cameraInfoTopicName);
    handle.getParam("/lidar_processing/lidar_depth_pub", lidarDepthTopicName);

    lidarSub = handle.subscribe(lidarTopicName, 1, &LidarProcessing::lidarCallback, this);
    cameraInfoSub = handle.subscribe(cameraInfoTopicName, 1, &LidarProcessing::cameraInfoCallback, this);
    lidarDepthPub = handle.advertise<sensor_msgs::Image>(lidarDepthTopicName, 1);

    if (!lidarDebugTopicName.empty()) {
        ROS_INFO("Publishing lidar debug to topic %s", lidarDebugTopicName.c_str());
        lidarDebugPub = handle.advertise<sensor_msgs::Image>(lidarDebugTopicName, 1);
    } else {
        ROS_INFO("Lidar debug publishing disabled by config");
    }
}

// source: https://github.com/libgdx/libgdx/blob/master/gdx/src/com/badlogic/gdx/math/MathUtils.java#L385
static inline constexpr double mapRange(double inRangeStart, double inRangeEnd, double outRangeStart,
                                        double outRangeEnd, double value) {
    return outRangeStart + (value - inRangeStart) * (outRangeEnd - outRangeStart) / (inRangeEnd - inRangeStart);
}

void LidarProcessing::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud) {
    auto start = ros::WallTime::now();

    // convert camera info into Open3D intrinsic/extrinsic tensor
    if (!camera.has_value() || !cameraInfo.has_value()) {
        ROS_WARN("Cannot generate depth buffer, still waiting for camera info!");
        return;
    }
    int width = static_cast<int>(cameraInfo->width);
    int height = static_cast<int>(cameraInfo->height);

    // convert ROS point cloud to Open3D
    t::geometry::PointCloud cloud;
    open3d_conversions::rosToOpen3d(rosCloud, cloud);

    // correct point cloud height
    // offset height by about +1.2 (TODO configure in YAML)
    auto correction = core::Tensor::Init({0.0, 0.0, 1.2});
    cloud = cloud.Translate(correction);

    // TODO also do occlusion culling to save compute time

    // setup camera matrix in Open3D
    // TODO currently intrinsics come from the ros topic but would make sense to load it from disk instead
    auto intrinsics = core::Tensor::Init({
                                                 { camera->fx(), 0.0, camera->cx() },
                                                 { 0.0, camera->fy(), camera->cy() },
                                                 { 0.0, 0.0,          1.0 }
                                         }, core::Device("CPU:0"));
    ROS_INFO("Camera intrinsics:\n%s", intrinsics.ToString().c_str());
    // TODO set extrinsics?
    auto extrinsics = core::Tensor::Eye(4, core::Float32, core::Device("CPU:0"));

    // project points to 3D image using Open3D
    auto o3dDepth = cloud.ProjectToDepthImage(width, height, intrinsics, extrinsics, 1000.0f, 25.0f);
    auto colorised = o3dDepth.ColorizeDepth(2.0f, 0.0f, 25.0f);

    // create OpenCV depth mat (no copy! wow!) https://stackoverflow.com/a/44453382/5007892
    // put in the OpenCV data directly
    cv::Mat depthImage(height, width, CV_32FC1, o3dDepth.GetDataPtr());
    cv::Mat colorisedDepthImage(height, width, CV_8UC3, colorised.GetDataPtr());

    // interpolate the depth image (inpaint? or use one of the papers in my firefox tabs)

    // publish the depth image over ROS
    auto cvImage = cv_bridge::CvImage();
    cvImage.image = depthImage;
    cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
//    cvImage.image = colorisedDepthImage;
//    cvImage.encoding = sensor_msgs::image_encodings::RGB8;
    lidarDepthPub.publish(cvImage.toImageMsg());
    // TODO also published compressed?

    double time = (ros::WallTime::now() - start).toSec() * 1000.0;
    ROS_INFO("Lidar callback time: %.2f ms", time);
}

void LidarProcessing::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
    if (camera.has_value()) {
        // already got camera model
        return;
    }
    camera = image_geometry::PinholeCameraModel();
    camera->fromCameraInfo(msg);
    cameraInfo = *msg;
    ROS_INFO("Received camera model");
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
    LidarProcessing detector = LidarProcessing(handle);
    ROS_INFO("Initialised ROS node");

    ros::spin();
}