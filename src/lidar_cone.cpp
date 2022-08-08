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

// colour maps
#include "lidar_cone_detection/turbo_colourmap.inc"

using namespace uqr;
using namespace open3d;

// Borrowed from the Python implementation:
// https://gist.github.com/mikhailov-work/ee72ba4191942acecc03fe6da94fc73f
// Modified by me to return BGR not RGB
//static cv::Scalar turboInterpolate(double value) {
//    double x = fmax(0.0, fmin(1.0, value));
//    double a = static_cast<int>(x*255.0);
//    double b = fmin(255.0, a + 1.0);
//    double f = x*255.0 - a;
//
//    double rC = turbo_srgb_floats[a][0] + (turbo_srgb_floats[b][0] - turbo_srgb_floats[a][0]) * f;
//    double gC = turbo_srgb_floats[a][1] + (turbo_srgb_floats[b][1] - turbo_srgb_floats[a][1]) * f;
//    double bC = turbo_srgb_floats[a][2] + (turbo_srgb_floats[b][2] - turbo_srgb_floats[a][2]) * f;
//}

LidarProcessing::LidarProcessing(ros::NodeHandle &handle) {
    if (!handle.getParam("/lidar_processing/lidar_topic", lidarTopicName)) {
        ROS_ERROR("Failed to load lidar_topic from lidar cone config YAML");
    }
    handle.getParam("/lidar_processing/lidar_debug_pub", lidarDebugTopicName);
    handle.getParam("/lidar_processing/camera_info_topic", cameraInfoTopicName);
    handle.getParam("/lidar_processing/lidar_depth_pub", lidarDepthTopicName);

    lidarSub = handle.subscribe(lidarTopicName, 1, &LidarProcessing::lidarCallback, this);
    cameraInfoSub = handle.subscribe(cameraInfoTopicName, 1, &LidarProcessing::cameraInfoCallback, this);
    lidarDepthPub = handle.advertise<sensor_msgs::CompressedImage>(lidarDepthTopicName, 1);
    cameraFrameSub = handle.subscribe("/camera/color/image_raw/compressed", 1, &LidarProcessing::cameraFrameCallback, this);
}

// This is a straightforward port of Tom's VAPE code, with some minor improvements.
// Original code: https://github.com/UQRacing/VAPE/blob/master/src/main.py
// TODO automate lidar camera calibration

/**
 * Constructs the rotation matrix of the lidar relative to the camera. All roations are in camera
 * space, not lidar.
 * Source: Tom's VAPE code.
 * @param xtheta The rotation along the x axis (in degrees)
 * @param ytheta The rotation along the y axis (in degrees)
 * @param ztheta The rotation along the z axis (in degrees)
 * @return 3x3 matrix representing the 3 rotation vectors of the camera
 */
static cv::Mat constructRotationMatrix(double xtheta, double ytheta, double ztheta) {
    // references:
    // - https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
    // - https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula (I believe this is what we are doing)
    // - cv::Rodrigues

    double xthetaRad = xtheta * DEG_RAD;
    double ythetaRad = ytheta * DEG_RAD;
    double zthetaRad = ztheta * DEG_RAD;

    // construct rotation vector
    auto rotationVector = cv::Mat(3, 1, CV_64F);
    rotationVector.at<double>(0, 0) = xthetaRad;
    rotationVector.at<double>(1, 0) = ythetaRad;
    rotationVector.at<double>(0, 0) = zthetaRad;
    auto rotationMatrix = cv::Mat(3, 3, CV_64F);

    // apply Rodrigues rotation formula to generate the rotation matrix
    cv::Rodrigues(rotationVector, rotationMatrix);

    return rotationMatrix;

    /*
     the rotation matrix we want is this:
     [[ 0.99978068  0.         -0.02094242]
     [ 0.          1.          0.        ]
     [ 0.02094242  0.          0.99978068]]
     which we now have
     */
}

void LidarProcessing::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud) {
    auto start = ros::WallTime::now();

    if (!camera.has_value() || !cameraInfo.has_value() || !lastCameraFrame.has_value()) {
        ROS_WARN("Cannot generate depth buffer, still waiting for camera info!");
        return;
    }
    int width = static_cast<int>(cameraInfo->width);
    int height = static_cast<int>(cameraInfo->height);

    // iterate over point cloud and generate a list of points
    std::vector<cv::Point3d> lidarPoints{};
    // reference: https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*rosCloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*rosCloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*rosCloud, "z");

    // TODO use omp parallel for here (it causes a segfault rn)
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // I have no idea how Tom figured this out, but the transform is actually x, -z, y
        // The Krok whiteboard said y, z, x which is wrong!
        // Even Tom's inline code comment says y, z, x but that actual code is what we have here
        // Absolutely insane and completely bizarre. Who knows.
        float lx = *iter_x;
        float ly = -(*iter_z);
        float lz = *iter_y;

        cv::Point3d lidarPoint(lx, ly, lz);
        lidarPoints.emplace_back(lidarPoint);
    }

    // calculate camera intrinsic matrix from received camera model
    cv::Mat intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    // at(row, column) = at(y,x)
    intrinsic.at<double>(0, 0) = camera->fx();
    intrinsic.at<double>(0, 2) = camera->cx();
    // next row
    intrinsic.at<double>(1, 1) = camera->fy();
    intrinsic.at<double>(1, 2) = camera->cy();
    // next row
    intrinsic.at<double>(2, 2) = 1.0;
    /*
     intrinsic matrix:
     { camera->fx(), 0.0, camera->cx() },
     { 0.0, camera->fy(), camera->cy() },
     { 0.0, 0.0,          1.0 }
     */
    ROS_INFO_STREAM("Camera intrinsics:\n" << intrinsic);

    // distortion coefficients
    // these are from VAPE code
    // TODO make these configurable in YAML
    double distortionValues[] = {-0.05496145784854889, 0.06309773772954941,
                                 -0.00040654116310179234, -0.0003133322752546519,
                                 -0.0198691263794899};
    auto distortion = cv::Mat(1, 5, CV_64F, distortionValues);

    // setup the extrinsic parameters
    // TODO make these configurable in YAML
    // camera rotation parameters
    double Rx = 0.0;
    double Ry = -1.2;
    double Rz = 0.0;
    auto rotationMatrix = constructRotationMatrix(Rx, Ry, Rz);

    // camera translation parameters
    double Tx = 0.0;
    double Ty = 0.04;
    double Tz = 0.03;
    double translationValues[] = {Tx, Ty, Tz};
    auto translation = cv::Mat(1, 3, CV_64F, translationValues);

    // get projected points
    std::vector<cv::Point2d> imagePoints{};
    ROS_INFO_STREAM("OpenCV matrices:\nobjectPoints:\nomitted\nrvec:\n" << rotationMatrix
        << "\ntvec:\n" << translation << "\ncameraMatrix:\n" << intrinsic << "\ndistCoeffs:\n" << distortion);
    cv::projectPoints(lidarPoints, rotationMatrix, translation, intrinsic, distortion, imagePoints);
    ROS_INFO("Have %zu points", imagePoints.size());

    // construct depth image
    cv::Mat depthImage(height, width, CV_8UC3);
    for (size_t i = 0; i < imagePoints.size(); i++) {
        auto dist = norm(lidarPoints[i]);
        auto point = imagePoints[i];
        // TODO make this configurable in YAML as well
        // TODO should these be || or && ? I thought || but tom's code has &&
        if ((point.x < 0 || point.x >= width) || (point.y < 0 || point.y >= height) || (dist < 0 || dist > 28)) {
            // invalid point
            continue;
        }

        // lookup intensity in Google's Turbo colour map
        // TODO interpolate colours properly
        auto colour = turbo_srgb_bytes[static_cast<size_t>(fmin(dist * 8, 255))];
        // Turbo is RGB, but our Mat is BGR

        // sometimes causes heap buffer overflow according to ASan
        /*auto x = static_cast<int>(point.x);
        auto y = static_cast<int>(point.y);
        depthImage.at<cv::Vec3b>(y, x)[0] = colour[2];
        depthImage.at<cv::Vec3b>(y, x)[1] = colour[1];
        depthImage.at<cv::Vec3b>(y, x)[2] = colour[0];*/

        cv::circle(depthImage, point, 1, cv::Scalar(colour[2], colour[1], colour[0]));
    }

    // (debug only) overlay images on top of each other
    cv::Mat publishImage(height, width, CV_8UC3);
    cv::addWeighted(depthImage, 0.75, *lastCameraFrame, 0.4, 0.0, publishImage);

    // interpolate the depth image (inpaint? or use one of the papers in my firefox tabs)

    // publish the depth image over ROS (compressed using PNG, with JPEG we may get too much inaccuracy
    // looking it up later)
    auto cvImage = cv_bridge::CvImage();
    cvImage.image = publishImage;
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    lidarDepthPub.publish(cvImage.toCompressedImageMsg(cv_bridge::Format::PNG));

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

void LidarProcessing::cameraFrameCallback(const sensor_msgs::CompressedImageConstPtr &image) {
    lastCameraFrame = cv_bridge::toCvCopy(*image, "bgr8")->image;
    ROS_INFO("Received camera frame");
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