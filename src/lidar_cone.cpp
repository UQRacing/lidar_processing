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
    lidarDepthPub = handle.advertise<sensor_msgs::CompressedImage>(lidarDepthTopicName, 1);
}

// This is a straightforward port of Tom's VAPE code, with some minor improvements.
// Original code: https://github.com/UQRacing/VAPE/blob/master/src/main.py
// TODO find out the maths behind this. I suspect we could clean up this function by doing it using
//  a quaternion. I also think we could automate lidar-camera calibration but I need to read about that.

/**
 * Constructs the rotation matrix of the lidar relative to the camera. All roations are in camera
 * space, not lidar.
 * Source: Tom's VAPE code.
 * @param xtheta The rotation along the x axis (in degrees)
 * @param ytheta The rotation along the y axis (in degrees)
 * @param ztheta The rotation along the z axis (in degrees)
 * @return 3x3 matrix representing the 3 rotation vectors of the camera
 */
static core::Tensor constructRotationMatrix(double xtheta, double ytheta, double ztheta) {
    // references:
    // - https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
    // - https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula (I believe this is what we are doing)
    // - cv::Rodrigues

    double xthetaRad = xtheta * DEG_RAD;
    double ythetaRad = ytheta * DEG_RAD;
    double zthetaRad = ztheta * DEG_RAD;

    auto xRotMat = core::Tensor::Eye(3, core::Dtype::Float64, core::Device("CPU:0"));
    auto yRotMat = core::Tensor::Eye(3, core::Dtype::Float64, core::Device("CPU:0"));
    auto zRotMat = core::Tensor::Eye(3, core::Dtype::Float64, core::Device("CPU:0"));

    // there is no way to make this cleaner, don't even try
    xRotMat[1][1] = cos(xthetaRad);
    xRotMat[1][2] = -sin(xthetaRad);
    xRotMat[2][1] = sin(xthetaRad);
    xRotMat[2][2] = cos(xthetaRad);

    yRotMat[0][0] = cos(ythetaRad);
    yRotMat[0][2] = sin(ythetaRad);
    yRotMat[2][0] = -sin(ythetaRad);
    yRotMat[2][2] = cos(ythetaRad);

    zRotMat[0][0] = cos(zthetaRad);
    zRotMat[0][1] = -sin(zthetaRad);
    zRotMat[1][0] = sin(zthetaRad);
    zRotMat[1][1] = cos(zthetaRad);

    /*
     the rotation matrix we want is this:
     [[ 0.99978068  0.         -0.02094242]
     [ 0.          1.          0.        ]
     [ 0.02094242  0.          0.99978068]]
     which we now have
     */

    return (xRotMat.Matmul(yRotMat)).Matmul(zRotMat);
}

void LidarProcessing::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud) {
    auto start = ros::WallTime::now();

    if (!camera.has_value() || !cameraInfo.has_value()) {
        ROS_WARN("Cannot generate depth buffer, still waiting for camera info!");
        return;
    }
    int width = static_cast<int>(cameraInfo->width);
    int height = static_cast<int>(cameraInfo->height);

    // convert ROS point cloud to Open3D for processing
    t::geometry::PointCloud cloud;
    open3d_conversions::rosToOpen3d(rosCloud, cloud);

    // calculate camera intrinsic matrix from received camera model
    // TODO make intrinsics/extrinsic calibration its own function??
    auto intrinsics = core::Tensor::Init({
                                                 { camera->fx(), 0.0, camera->cx() },
                                                 { 0.0, camera->fy(), camera->cy() },
                                                 { 0.0, 0.0,          1.0 }
                                         }, core::Device("CPU:0"));
    ROS_INFO("Camera intrinsics:\n%s", intrinsics.ToString().c_str());

    // calculate extrinsic matrix (the hard part)
    // extrinsic matrix references:
    // - https://i.stack.imgur.com/AGwu9.jpg
    // - https://developer.apple.com/documentation/avfoundation/avcameracalibrationdata/2881130-extrinsicmatrix
    // - Tom's VAPE code (citation above)
    // TODO make these configurable in YAML
    // camera rotation parameters
    double Rx = 0.0;
    double Ry = -1.2;
    double Rz = 0.0;
    auto rotation = constructRotationMatrix(Rx, Ry, Rz);

    // camera translation parameters
    double Tx = 0.0;
    double Ty = 0.04;
    double Tz = 0.03;
    auto translation = core::Tensor::Init({{Tx}, {Ty}, {Tz}});
    ROS_INFO("\n\nRotation:\n%s\n\nTranslation:%s", rotation.ToString().c_str(),
             translation.ToString().c_str());

    // construct the extrinsic matrix, which is the rotation matrix augmented with the camera
    // transform vector (plus some identity row thing).
    // random source on identity row: https://answers.unity.com/storage/temp/111020-matrix4x4-basic-info.png
    // [ r0 r1 r2 | t0 ]
    // [ r3 r4 r5 | t1 ]
    // [ r6 r7 r8 | t2 ]
    // [ 0  0  0  | 1  ]
    auto identityRow = core::Tensor::Init({{0.0, 0.0, 0.0, 1.0}});
    auto extrinsics = rotation.Append(translation, 1).Append(identityRow, 0);
    ROS_INFO("\n\nExtrinsics:\n%s", extrinsics.ToString().c_str());

    // TODO do frustum culling here to save time (omit points not in camera frustum)

    // project points to 3D image using Open3D
    auto o3dDepth = cloud.ProjectToDepthImage(width, height, intrinsics, extrinsics, 1000.0f, 20.0f);

    // create OpenCV depth mat (no copy! wow!) https://stackoverflow.com/a/44453382/5007892
    cv::Mat depthImage(height, width, CV_32FC1, o3dDepth.GetDataPtr());
    // ROS is outrageously stupid (as usual), we can't publish compressed as CV_32FC1, so we have
    // to do an expensive copy here
    // TODO force ROS to be less fucking stupid and skip this copy
    cv::Mat publishImage(height, width, CV_8UC1);
    depthImage.copyTo(publishImage);

    // interpolate the depth image (inpaint? or use one of the papers in my firefox tabs)

    // publish the depth image over ROS (compressed using PNG, with JPEG we may get too much inaccuracy
    // looking it up later)
    auto cvImage = cv_bridge::CvImage();
    cvImage.image = publishImage;
    cvImage.encoding = sensor_msgs::image_encodings::MONO8;
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