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
#include <opencv2/xphoto.hpp>

// colour maps
#include "lidar_cone_detection/turbo_colourmap.inc"

using namespace uqr;
using namespace open3d;

LidarProcessing::LidarProcessing(ros::NodeHandle &handle) {
    if (!handle.getParam("/lidar_processing/lidar_topic", lidarTopicName)) {
        ROS_ERROR("Failed to load lidar_topic from lidar cone config YAML");
    }
    handle.getParam("/lidar_processing/lidar_debug_pub", lidarDebugTopicName);
    handle.getParam("/lidar_processing/camera_info_topic", cameraInfoTopicName);
    handle.getParam("/lidar_processing/lidar_depth_pub", lidarDepthTopicName);
    auto inpaintingDebugTopicName = handle.param<std::string>("/lidar_processing/inpainting_debug_pub", "");

    // pipeline features from YAML
    handle.getParam("/lidar_processing/inpainting", inpainting);
    handle.getParam("/lidar_processing/dilate", dilate);

    // pub/sub topics
    lidarSub = handle.subscribe(lidarTopicName, 1, &LidarProcessing::lidarCallback, this);
    cameraInfoSub = handle.subscribe(cameraInfoTopicName, 1, &LidarProcessing::cameraInfoCallback, this);
    lidarDepthPub = handle.advertise<sensor_msgs::CompressedImage>(lidarDepthTopicName, 1);
    cameraFrameSub = handle.subscribe("/camera/color/image_raw/compressed", 1, &LidarProcessing::cameraFrameCallback, this);
    if (!inpaintingDebugTopicName.empty()) {
        ROS_INFO("Publishing inpainting debug to topic %s", inpaintingDebugTopicName.c_str());
        inpaintingDebugPub = handle.advertise<sensor_msgs::CompressedImage>(inpaintingDebugTopicName, 1);
    }

    ROS_INFO("Pipeline features: inpainting: %s, dilation: %s", inpainting ? "yes" : "no", dilate ? "yes" : "no");
}

// This is a straightforward port of Tom's VAPE code, with some minor improvements.
// Original code: https://github.com/UQRacing/VAPE/blob/master/src/main.py
// TODO automate lidar camera calibration

// source: https://github.com/libgdx/libgdx/blob/master/gdx/src/com/badlogic/gdx/math/MathUtils.java#L385
static inline constexpr double mapRange(double inRangeStart, double inRangeEnd, double outRangeStart,
                                        double outRangeEnd, double value) {
    return outRangeStart + (value - inRangeStart) * (outRangeEnd - outRangeStart) / (inRangeEnd - inRangeStart);
}

/**
 * Uses Rodrigues' formula to turn a 3D rotation vector into a 3D rotation matrix.
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
    double rotationValues[] = {xthetaRad, ythetaRad, zthetaRad};
    auto rotationVector = cv::Mat(1, 3, CV_64F, rotationValues);

    auto rotationMatrix = cv::Mat(3, 3, CV_64F);
    // apply Rodrigues rotation formula to generate the rotation matrix
    cv::Rodrigues(rotationVector, rotationMatrix);

    return rotationMatrix;
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
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // I have no idea how Tom figured this out, but the transform is actually x, -z, y
        // The Krok whiteboard said y, z, x which is wrong!
        // Even Tom's inline code comment says y, z, x but that actual code is what we have here.
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

    // find min and max depth (for interpolation)
    // save computing depth twice, store it in a hash map
    std::unordered_map<size_t, double> depthMappings{};
    double minDist = 999999, maxDist = -999999;
//#pragma omp parallel for default(none) shared(minDist, maxDist, depthMappings) firstprivate(imagePoints, lidarPoints, width, height)
    for (size_t i = 0; i < imagePoints.size(); i++) {
        auto dist = norm(lidarPoints[i]);
        auto point = imagePoints[i];
        if (point.x < 0 || point.x >= width || point.y < 0 || point.y >= height) {
            // invalid point (out of bounds of 2D image)
            continue;
        }
        depthMappings[i] = dist;
        if (dist < minDist) {
            minDist = dist;
        } else if (dist > maxDist) {
            maxDist = dist;
        }
    }

    // construct depth image
    cv::Mat depthImage(height, width, CV_8UC3);
    depthImage.setTo(cv::Scalar(0, 0, 0));
    // mask of pixels not drawn in the depth image (white = not drawn, black = drawn over)
    // used for inpainting later
    cv::Mat pixelsNotDrawn = cv::Mat::zeros(height, width, CV_8UC1);
    if (inpainting) pixelsNotDrawn.setTo(255);

    // record min and max y written to the image, so we know in what range to interpolate
    int minY = 9999, maxY = -9999;
    for (size_t i = 0; i < imagePoints.size(); i++) {
        auto point = imagePoints[i];
        if (point.x < 0 || point.x >= width || point.y < 0 || point.y >= height) {
            // invalid point (out of bounds of 2D image)
            continue;
        }
        auto dist = depthMappings[i];

        // lookup intensity in Google's Turbo colour map
        auto colourIndex = floor(mapRange(minDist, maxDist, 0.0, 255.0, dist));
        auto colour = turbo_srgb_bytes[static_cast<int>(colourIndex)];
        auto x = static_cast<int>(point.x);
        auto y = static_cast<int>(point.y);
        // Turbo is RGB, but our Mat is BGR, hence the indexing order
        depthImage.at<cv::Vec3b>(y, x)[0] = colour[2];
        depthImage.at<cv::Vec3b>(y, x)[1] = colour[1];
        depthImage.at<cv::Vec3b>(y, x)[2] = colour[0];

        if (y < minY) {
            minY = y;
        } else if (y > maxY) {
            maxY = y;
        }
        // unset this area of the mask (we've filled it, so it shouldn't be inpainted)
        if (inpainting) pixelsNotDrawn.at<uint8_t>(y, x) = 0;
        //cv::circle(depthImage, point, 1, cv::Scalar(colour[2], colour[1], colour[0]));
    }

    // dilate the image
    // TODO change dilation factor
    if (dilate) {
        cv::dilate(depthImage, depthImage, cv::Mat());
    }

    // now constrain the mask to only have the certain height band in the image that contains lidar pixels
    // FIXME this over-estimates the y bounds leading to long inpaint times, reduce this!
    //  - calculate minimum bounding polygon for the set of points
    // TODO maybe use opencv's photo denoising algorithm instead of inpainting??
    //  - i.e. fastNlMeansDenoisingColoredMulti
    cv::Mat inpaintMask;
    if (inpainting) {
        cv::Mat yMask = cv::Mat::zeros(height, width, CV_8UC1);
        cv::rectangle(yMask, cv::Point2i(0, minY), cv::Point2i(width, maxY), 255,
                      cv::LineTypes::FILLED);
        cv::bitwise_and(pixelsNotDrawn, yMask, inpaintMask);

        // interpolate the depth image, currently using inpainting
        cv::inpaint(depthImage, inpaintMask, depthImage, 4.0, cv::INPAINT_TELEA);
    }

    // (debug only) overlay images on top of each other
    auto publishImage = depthImage;
//    cv::Mat publishImage(height, width, CV_8UC3);
//    cv::addWeighted(inpainted, 0.95, *lastCameraFrame, 0.6, 0.0, publishImage);

    // publish the depth image over ROS (compressed using PNG, with JPEG we may get too much inaccuracy
    // looking it up later)
    auto cvImage = cv_bridge::CvImage();
    cvImage.image = publishImage;
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    lidarDepthPub.publish(cvImage.toCompressedImageMsg(cv_bridge::Format::PNG));

    // publish inpainting debug if required
    if (inpaintingDebugPub.has_value() && inpainting) {
        auto debugImage = cv_bridge::CvImage();
        debugImage.image = inpaintMask;
        debugImage.encoding = sensor_msgs::image_encodings::MONO8;
        inpaintingDebugPub->publish(debugImage.toCompressedImageMsg(cv_bridge::Format::PNG));
    }

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