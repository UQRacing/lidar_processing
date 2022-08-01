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

    // project points to 3D image using image_geometry
    // reference: https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*rosCloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*rosCloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*rosCloud, "z");

    // depth image is greyscale, single channel
    int width = static_cast<int>(cameraInfo->width);
    int height = static_cast<int>(cameraInfo->height);

    // since the depthImage will be a UMat, be on the GPU, so we want to do one "bulk upload" rather
    // than set each pixel individually
    // TODO check this is the right format (row major)
    uint8_t pixels[height][width];
    std::memset(pixels, 0, width * height * sizeof(uint8_t));

    // first argument in the pair is the point, second argument is the distance from the lidar to
    // this point (before it was projected into 2D)
    std::vector<CamPointPair_t> camPoints{};
    double missedPoints = 0.0;
    double totalPoints = 0.0;
    // TODO use omp parallel for here (it causes a segfault rn)
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // thank you krok whiteboard!!! the bizarre camera frame is INDEED y,z,x
        float lx = *iter_y;
        float ly = *iter_z;
        float lz = *iter_x;
        cv::Point3d lidarPoint(lx, ly, lz);
        totalPoints++;

        // project the point, and store it in the depth image as a pixel
        auto camPoint = camera->project3dToPixel(lidarPoint);
        if (camPoint.x < 0 || camPoint.y < 0 || camPoint.x > width || camPoint.y > height) {
            // point out of bounds
            // TODO why does this still happen??
            missedPoints++;
            continue;
        }
        // distance between lidar (origin) and point
        auto dist = cv::norm(lidarPoint);
        camPoints.emplace_back(std::make_pair(camPoint, dist));
    }

    ROS_INFO("Missed points: %.2f%%", (missedPoints / totalPoints) * 100.0);

    // first find the max and min distance, so we can lerp
    // std::minmax_element is giving me grief, so we do this manually
    double minDist = 999999, maxDist = -999999;
#pragma omp parallel for default(none) shared(camPoints, minDist, maxDist)
    for (const auto &element : camPoints) {
        // compare by distance
        auto dist = element.second;
        if (dist < minDist) {
            minDist = dist;
        } else if (dist > maxDist) {
            maxDist = dist;
        }
    }

    // write pixels to pix buf with correct intensities based on depth
#pragma omp parallel for default(none) shared(camPoints, pixels, minDist, maxDist)
    for (const auto &pair : camPoints) {
        auto [camPoint, dist] = pair;
        auto camX = static_cast<size_t>(camPoint.x);
        auto camY = static_cast<size_t>(camPoint.y);
        // map lidar distance range to 0-255 for depth image
        double depth = mapRange(minDist, maxDist, 0, 255, pair.second);
        pixels[camY][camX] = static_cast<uint8_t>(floor(depth));
    }

    // create OpenCV depth mat (no copy! wow!) https://stackoverflow.com/a/44453382/5007892
    cv::Mat depthImage(height, width, CV_8UC1, pixels);

#if 0
    // old method
    // load camera parameters (what is a good way to do this?)
    //  use this: https://wiki.ros.org/camera_calibration_parsers ?
    auto intrinsic = open3d::core::Tensor();
    auto extrinsic = open3d::core::Tensor();

    // project lidar points onto camera plane to generate depth image
    // - tune the last two parameters: depth scale and max depth
    // - we could use opencv for this, cv::projectPoints, but the arguments are more confusing
    auto depthImage = cloud.ProjectToDepthImage(1280, 720,
                                                intrinsic, extrinsic, 1000.0f, 30.0f);

    // convert Open3D depth image to OpenCV
#endif

    // interpolate the depth image (inpaint? or use one of the papers in my firefox tabs)

    // publish the depth image over ROS
    auto cvImage = cv_bridge::CvImage();
    cvImage.image = depthImage;
    cvImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
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