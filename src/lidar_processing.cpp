// LiDAR processing, main source file
// Copyright (c) 2022 Matt Young (UQ Racing Formula SAE Team)
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// SPDX-License-Identifier: MPL-2.0

#include "lidar_processing/lidar_processing.h"
#include "lidar_processing/defines.h"
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "lidar_processing/tinycolormap.hpp"

using namespace uqr;

// This part (up until the constructor) is a straightforward port of Tom's VAPE code, with some minor improvements.
// Original code: https://github.com/UQRacing/VAPE/blob/master/src/main.py
// The legacy VAPE code has been renamed into vape-legacy, and the new VAPE, "vape2", will just be
// focused on vehicle position estimation and cone position estimation

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

// Source: https://github.com/libgdx/libgdx/blob/master/gdx/src/com/badlogic/gdx/math/MathUtils.java#L385
// Apache 2.0
static inline constexpr double mapRange(double inRangeStart, double inRangeEnd, double outRangeStart,
                                        double outRangeEnd, double value) {
    return outRangeStart + (value - inRangeStart) * (outRangeEnd - outRangeStart) / (inRangeEnd - inRangeStart);
}

/**
 * Generates LiDAR points in correct coordinate space from a PointCloud2 message.
 * @param rosCloud the point cloud 2 message
 * @return list of points, for sending to OpenCV
 */
static std::vector<cv::Point3d> generateLidarPoints(const sensor_msgs::PointCloud2ConstPtr &rosCloud) {
    // iterate over point cloud and generate a list of points
    std::vector<cv::Point3d> lidarPoints{};
    // reference: https://github.com/mikeferguson/ros2_cookbook/blob/main/rclcpp/pcl.md
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*rosCloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*rosCloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*rosCloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        // I have no idea how Tom figured this out, but the transform is actually x,-z,y - even the
        // magic Krok whiteboard said y,z,x which is wrong! In fact, even Tom's inline code comment
        // says y,z,x, but the actual code implements what we have here: x,-z-y.
        // Update from the man himself:
        // It appears to be because the previous LiVox LiDAR used y,z,x but the LeiShen uses x,-z,y.
        // Explains why both Krok whiteboard and Tom's code were wrong, because neither got updated
        // when we moved to LeiShen. So there you go.
        float lx = *iter_x;
        float ly = -(*iter_z);
        float lz = *iter_y;
        cv::Point3d lidarPoint(lx, ly, lz);
        lidarPoints.emplace_back(lidarPoint);
    }
    return lidarPoints;
}

LidarProcessing::LidarProcessing(ros::NodeHandle &handle) {
    handle.getParam("/lidar_processing/lidar_topic", lidarTopicName);
    handle.getParam("/lidar_processing/camera_info_topic", cameraInfoTopicName);
    handle.getParam("/lidar_processing/lidar_depth_image_pub", lidarDepthImgTopicName);
    handle.getParam("/lidar_processing/lidar_depth_pub", lidarDepthTopicName);
    handle.getParam("/lidar_processing/rvec", rvecYaml);
    handle.getParam("/lidar_processing/tvec", tvecYaml);
    handle.getParam("/lidar_processing/morphKernelSize", morphKernelSize);
    auto cameraFrameTopicName = handle.param<std::string>("/lidar_processing/camera_frame_topic", "");
    auto inpaintingDebugTopicName = handle.param<std::string>("/lidar_processing/inpainting_debug_pub", "");

    // pipeline features from YAML
    handle.getParam("/lidar_processing/inpainting", inpainting);
    handle.getParam("/lidar_processing/morphological", morphological);
    handle.getParam("/lidar_processing/publishColour", publishColour);

    // camera rotation parameters (extrinsic)
    double Rx = rvecYaml[0];
    double Ry = rvecYaml[1];
    double Rz = rvecYaml[2];
    rotationMatrix = constructRotationMatrix(Rx, Ry, Rz);

    // pub/sub topics
    lidarSub = handle.subscribe(lidarTopicName, 1, &LidarProcessing::lidarCallback, this);
    cameraInfoSub = handle.subscribe(cameraInfoTopicName, 1, &LidarProcessing::cameraInfoCallback, this);
    // this one publishes just the image
    lidarDepthImgPub = handle.advertise<sensor_msgs::CompressedImage>(lidarDepthImgTopicName, 1);
    // this one publishes the data which will actually be used in other nodes
    lidarDepthPub = handle.advertise<uqr_msgs::DepthImage>(lidarDepthTopicName, 1);
    ROS_INFO("Subscribing to LiDAR topic on %s", lidarTopicName.c_str());
    ROS_INFO("Subscribing to camera info topic on %s", cameraInfoTopicName.c_str());
    ROS_INFO("Publishing LiDAR depth data to topic %s", lidarDepthTopicName.c_str());

    // debug topics
    if (!inpaintingDebugTopicName.empty()) {
        ROS_INFO("(Optional, enabled) Publishing inpainting debug to topic %s", inpaintingDebugTopicName.c_str());
        inpaintingDebugPub = handle.advertise<sensor_msgs::CompressedImage>(inpaintingDebugTopicName, 1);
    }
    if (!cameraFrameTopicName.empty()) {
        ROS_INFO("(Optional, enabled) Receiving camera frames for debug on topic %s", cameraFrameTopicName.c_str());
        cameraFrameSub = handle.subscribe(cameraFrameTopicName, 1,&LidarProcessing::cameraFrameCallback, this);
    }

    ROS_INFO("Pipeline features: inpainting: %s, morphological: %s, publish colour: %s",
             inpainting ? "yes" : "no", morphological ? "yes" : "no", publishColour ? "yes" : "no");
}

void LidarProcessing::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud) {
    auto start = ros::WallTime::now();

    if (!camera.has_value() || !cameraInfo.has_value()) {
        ROS_WARN("Cannot generate depth buffer, still waiting for camera info!");
        return;
    }
    int width = static_cast<int>(cameraInfo->width);
    int height = static_cast<int>(cameraInfo->height);

    // generate lidar points
    auto lidarPoints = generateLidarPoints(rosCloud);

    // camera translation parameters (extrinsic)
    // for some absolutely godforsaken stupid awful reason, we have to recalculate this every callback,
    // otherwise the results are incorrect and glitchy
    double Tx = tvecYaml[0];
    double Ty = tvecYaml[1];
    double Tz = tvecYaml[2];
    double translationValues[] = {Tx, Ty, Tz};
    translation = cv::Mat(1, 3, CV_64F, translationValues);

    // the main operation: using cv::projectPoints to project the LiDAR points into camera space using
    // the previously calculated camera matrices
    std::vector<cv::Point2d> imagePoints{};
    cv::projectPoints(lidarPoints, rotationMatrix, translation, intrinsic, distortion, imagePoints);

    // find min and max depth (for interpolation later on)
    double minDist = 999999, maxDist = -999999;
    for (size_t i = 0; i < imagePoints.size(); i++) {
        auto dist = norm(lidarPoints[i]);
        auto point = imagePoints[i];
        if (point.x < 0 || point.x >= width || point.y < 0 || point.y >= height) {
            // invalid point (out of bounds of 2D image)
            continue;
        }
        if (dist < minDist) {
            minDist = dist;
        } else if (dist > maxDist) {
            maxDist = dist;
        }
    }

    // big step: construct the depth image
    cv::Mat depthImage(height, width, CV_8UC3);
    depthImage.setTo(cv::Scalar(0, 0, 0));
    for (size_t i = 0; i < imagePoints.size(); i++) {
        auto point = imagePoints[i];
        if (point.x < 0 || point.x >= width || point.y < 0 || point.y >= height) {
            // invalid point (out of bounds of 2D image)
            continue;
        }
        auto dist = norm(lidarPoints[i]);
        auto x = static_cast<int>(point.x);
        auto y = static_cast<int>(point.y);

        if (publishColour) {
            // lookup intensity in colour map
            auto colourIndex = mapRange(minDist, maxDist,
                                        0.0, 1.0, dist);
            auto colour = tinycolormap::GetColor(colourIndex, tinycolormap::ColormapType::Turbo);

            // colour map is RGB, but our Mat is BGR, hence the indexing order
            // access pixels directly for speed https://stackoverflow.com/a/8933111/5007892
            depthImage.data[depthImage.channels() * (depthImage.cols * y + x) + 0] = colour.bi();
            depthImage.data[depthImage.channels() * (depthImage.cols * y + x) + 1] = colour.gi();
            depthImage.data[depthImage.channels() * (depthImage.cols * y + x) + 2] = colour.ri();
        } else {
            // transmit greyscale only
            auto colour = static_cast<int>(floor(mapRange(minDist, maxDist,
                                                          0.0, 255.0, dist)));
            // although the image is greyscale, it still uses 3 channels for compatibility with
            // the colour image
            depthImage.data[depthImage.channels() * (depthImage.cols * y + x) + 0] = colour;
            depthImage.data[depthImage.channels() * (depthImage.cols * y + x) + 1] = colour;
            depthImage.data[depthImage.channels() * (depthImage.cols * y + x) + 2] = colour;
        }
    }

    // apply morphological operations to the image if enabled
    if (morphological) {
        // instead of dilation, we actually use MORPH_CLOSE because it is better
        auto kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_RECT,
                                                cv::Size(morphKernelSize, morphKernelSize));
        cv::morphologyEx(depthImage, depthImage, cv::MORPH_CLOSE, kernel);
    }

    // run inpainting: like photoshop's content aware fill, slow, but yields to good results as it
    // fills in missing depth pixels leading to a much denser image
    // TODO: remove inpainting, it is no longer useful
    cv::Mat inpaintMask = cv::Mat::zeros(height, width, CV_8UC1);
    if (inpainting) {
        // mask of pixels not drawn in the depth image (white = not drawn, black = drawn over)
        // remains on CPU
        cv::Mat pixelsNotDrawn = cv::Mat::zeros(height, width, CV_8UC1);
        for (int y = 0; y < depthImage.rows; y++) {
            for (int x = 0; x < depthImage.cols; x++) {
                auto colour = depthImage.at<cv::Vec3b>(y, x);
                // if the pixel is black, it has not been written: so it should be white in the
                // inpaint masking
                if (colour[0] == 0 && colour[1] == 0 && colour[2] == 0) {
                    pixelsNotDrawn.at<uint8_t>(y, x) = 255;
                }
            }
        }

        // constrain region to inpaint into a manually calculated bounding box
        // you can open the images in rqt and export them and use the rectangle select tool in GIMP
        // to calculate these
        // TODO crop rects should be conenet_ros predictions
        cv::Rect crop(337, 368, 755, 188);
        // ideally we would use like pixelsNotDrawn(crop) but for god knows why it's not working
        cv::Mat cropMask = cv::Mat::zeros(height, width, CV_8UC1);
        cv::rectangle(cropMask, crop, 255,cv::LineTypes::FILLED);
        // bitwise_and is faster than copyTo it seems
        cv::bitwise_and(pixelsNotDrawn, cropMask, inpaintMask);

        // interpolate the depth image, currently using inpainting
        cv::inpaint(depthImage, inpaintMask, depthImage, 1.0, cv::INPAINT_TELEA);
    }

    auto publishImage = depthImage;

    // (debug only) overlay images on top of each other
    // cv::Mat publishImage(height, width, CV_8UC3);
    // cv::addWeighted(inpainted, 0.95, *lastCameraFrame, 0.6, 0.0, publishImage);

    // publish the depth image over ROS (compressed using PNG, with JPEG due to colour banding we
    // would get too much inaccuracy for all the work we just did!)
    auto cvImage = cv_bridge::CvImage();
    cvImage.header.stamp = ros::Time::now();
    cvImage.image = publishImage;
    cvImage.encoding = sensor_msgs::image_encodings::BGR8;
    auto compressed = cvImage.toCompressedImageMsg(cv_bridge::Format::PNG);
    lidarDepthImgPub.publish(compressed);

    // publish the actual LiDAR-camera fusion data that other nodes will use
    auto depthImageMsg = uqr_msgs::DepthImage();
    depthImageMsg.header.stamp = ros::Time::now();
    depthImageMsg.depthImage = *compressed;
    depthImageMsg.min = minDist;
    depthImageMsg.max = maxDist;
    lidarDepthPub.publish(depthImageMsg);

    // publish inpainting debug if requested, and inpainting is enabled
    if (inpaintingDebugPub.has_value() && inpainting) {
        auto debugImage = cv_bridge::CvImage();
        debugImage.header.stamp = ros::Time::now();
        debugImage.image = inpaintMask;
        debugImage.encoding = sensor_msgs::image_encodings::MONO8;
        inpaintingDebugPub->publish(debugImage.toCompressedImageMsg(cv_bridge::Format::PNG));
    }

    double time = (ros::WallTime::now() - start).toSec() * 1000.0;
    ROS_INFO("Lidar callback time: %.2f ms (at least %.2f FPS)", time, (1000.0 / time));
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

    // get intrinsics and distortion coefficients from camera model
    intrinsic = camera->fullIntrinsicMatrix();
    distortion = camera->distortionCoeffs();
}

void LidarProcessing::cameraFrameCallback(const sensor_msgs::CompressedImageConstPtr &image) {
    lastCameraFrame = cv_bridge::toCvCopy(*image, "bgr8")->image;
//    ROS_INFO("Received camera frame");
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_process");
    ROS_INFO("LiDAR Processing Next v" LIDAR_PROCESSING_VERSION);
    ROS_INFO("LiDAR processing (c) 2022 Matt Young, Tom Day, Fahed Alhanaee. Licenced under the Mozilla Public License v2.0.");
    auto cvCpuFeatures = cv::getCPUFeaturesLine();
    auto cvNumThreads = cv::getNumThreads();
    auto cvVersion = cv::getVersionString();
//    ROS_INFO("OpenCV build info:\n%s", cv::getBuildInformation().c_str());
    ROS_INFO("Using OpenCV v%s with %d threads, features: %s", cvVersion.c_str(), cvNumThreads, cvCpuFeatures.c_str());

    ros::NodeHandle handle{};
    LidarProcessing detector = LidarProcessing(handle);
    ROS_INFO("Initialised ROS node");

    ros::spin();
}