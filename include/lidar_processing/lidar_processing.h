// LiDAR processing (new version), main header
// Copyright (c) 2022 Matt Young (UQ Racing Formula SAE Team)
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// SPDX-License-Identifier: MPL-2.0

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <uqr_msgs/DepthImage.h>

#define DEG_RAD 0.017453293
#define RAD_DEG 57.29578

namespace uqr {
    class LidarProcessing {
    public:
        explicit LidarProcessing(ros::NodeHandle &handle);

    private:
        ros::Subscriber lidarSub, cameraInfoSub, cameraFrameSub;
        ros::Publisher lidarDepthImgPub, lidarDepthPub;

        // YAML parameters
        std::string lidarTopicName{};
        std::string cameraInfoTopicName{};
        std::string lidarDepthImgTopicName{};
        std::string lidarDepthTopicName{};
        std::vector<double> rvecYaml{}, tvecYaml{};
        int morphKernelSize{};

        // Camera matrices
        cv::Mat rotationMatrix, translation, distortion;
        cv::Matx33d intrinsic;

        // pipeline configuration
        bool morphological = false, publishColour = false;

        ddynamic_reconfigure::DDynamicReconfigure ddr{};

        std::optional<image_geometry::PinholeCameraModel> camera{};
        std::optional<sensor_msgs::CameraInfo> cameraInfo{};
        std::optional<cv::Mat> lastCameraFrame{}; // only used for debug

        void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud);
        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
        void cameraFrameCallback(const sensor_msgs::CompressedImageConstPtr &image);
    };
}