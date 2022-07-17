// LiDAR cone detector, main file
// Matt Young (with much prior work done by Riley Bowyer & Caleb Aitken), 2022, UQRacing
#include "lidar_cone_detection/lidar_cone.h"
#include "lidar_cone_detection/defines.h"
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "lidar_cone_detection/open3d_conversions.h"
#include <open3d/Open3D.h>

using namespace uqr;

LidarConeDetector::LidarConeDetector(ros::NodeHandle &handle) {
    if (!handle.getParam("/lidar_cone_detector/lidar_topic", lidarTopicName)) {
        ROS_ERROR("Failed to load lidar_topic from lidar cone config YAML");
    }
    handle.getParam("/lidar_cone_detector/debug_ui", enableDebugUI);
    handle.getParam("/lidar_cone_detector/lidar_debug_pub", lidarDebugTopicName);

    lidarSub = handle.subscribe(lidarTopicName, 1, &LidarConeDetector::lidarCallback, this);
    // TODO conePub (figure out what message type we publish)

    if (!lidarDebugTopicName.empty()) {
        ROS_INFO("Publishing lidar debug to topic %s", lidarDebugTopicName.c_str());
        lidarDebugPub = handle.advertise<sensor_msgs::PointCloud2>(lidarDebugTopicName, 1);
    }

    if (enableDebugUI) {
        // TODO
    }
}

// inspiration for this method comes from:
// https://github.com/ros-perception/perception_open3d/blob/main/open3d_conversions/src/open3d_conversions.cpp

void LidarConeDetector::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud) {
    auto start = ros::WallTime::now();

    open3d::geometry::PointCloud cloud;
    open3d_conversions::rosToOpen3d(rosCloud, cloud, true);

    // voxel downsample, reduce the resolution of the point cloud to speed up the following algorithms
    // not doing this yet because I think we'll need the resolution
    // auto downsampledCloud = *cloud.VoxelDownSample(0.05);

    // perform RANSAC plane segmentation
    // reference: http://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html#Plane-segmentation
    // TODO make these parameters configurable in the YAML
    auto [planeModel, inliers] =
            cloud.SegmentPlane(0.01, 3, 100, 0xDEE5);

    // everything except the ground plane
    auto notGroundCloud = *cloud.SelectByIndex(inliers, true);

    // publish debug
    if (!lidarDebugTopicName.empty()) {
        sensor_msgs::PointCloud2 rosDebugCloud{};

        // publish the inliers of the plane model as a separate cloud
        open3d_conversions::open3dToRos(notGroundCloud, rosDebugCloud, "laser_link");
        lidarDebugPub.publish(rosDebugCloud);
    }

    double time = (ros::WallTime::now() - start).toSec() * 1000.0;
    ROS_INFO("Lidar callback time: %.2f ms", time);
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_cone");
    ROS_INFO("LiDAR Cone Detection v" LIDAR_CONE_VERSION ": Matt Young, Riley Bowyer, Caleb Aitken, 2021-2022, UQRacing");

    ros::NodeHandle handle{};
    LidarConeDetector detector = LidarConeDetector(handle);
    ROS_INFO("Initialised ROS node");

    ros::spin();
}