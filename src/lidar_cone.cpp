// LiDAR cone detector (new version), main file
// Matt Young, 2022, UQRacing
#include "lidar_cone_detection/lidar_cone.h"
#include "lidar_cone_detection/defines.h"
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "lidar_cone_detection/open3d_conversions.h"
#include <open3d/Open3D.h>
#include <visualization_msgs/MarkerArray.h>

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
    handle.getParam("/lidar_cone_detector/plane_dist_thresh", planeDistThresh);
    handle.getParam("/lidar_cone_detector/plane_ransac_n", planeRansacN);
    handle.getParam("/lidar_cone_detector/plane_num_iters", planeNumIters);
    handle.getParam("/lidar_cone_detector/voxeliser_resolution", voxeliserResolution);
    handle.getParam("/lidar_cone_detector/dbscan_eps", dbscanEps);
    handle.getParam("/lidar_cone_detector/dbscan_min_pts", dbscanMinPts);

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

    open3d::geometry::PointCloud cloud;
    open3d_conversions::rosToOpen3d(rosCloud, cloud, true);

    // perform RANSAC plane segmentation
    // reference: http://www.open3d.org/docs/latest/tutorial/Basic/pointcloud.html#Plane-segmentation
    // use a constant seed 0xDEE5 (lmao) to produce more predictable and hopefully more temporally stable
    // results?
    auto [planeModel, inliers] =
            cloud.SegmentPlane(planeDistThresh, planeRansacN,
                               planeNumIters, 0xDEE5);

    // grab everything except the ground plane
    auto notGroundCloud = *cloud.SelectByIndex(inliers, true);

    // DBSCAN is incredibly slow on large point clouds, so voxel downscale after segmentation. we don't
    // downscale _before_ segmentation (yet), because I think RANSAC will be to inaccurate then and destroy
    // potential cones in the process.
    // statistical/radius outlier removal would also help, but it appears to be even slower
    auto reduced = *notGroundCloud.VoxelDownSample(voxeliserResolution);

    // cluster with DBSCAN
    auto clusteredReduced = reduced.ClusterDBSCAN(dbscanEps, dbscanMinPts);

    // publish lidar debug
    if (!lidarDebugTopicName.empty()) {
        sensor_msgs::PointCloud2 rosDebugCloud{};

        // map DBSCAN clusters to colour
        int numClusters = *std::max_element(clusteredReduced.begin(), clusteredReduced.end());
        ROS_INFO("max clusters %d", numClusters);

        for (int i : clusteredReduced) {
            Eigen::Matrix<double, 3, 1> colour;
            if (i == -1) {
                // noise point
                colour << 1, 1, 1;
            } else {
                // assign colour based on topic, use HSV and set hue based on which cluster it is
                double h = ((double) i / (double) numClusters) * 360.0;
                colour = hsvToRgb(h, 100.0, 100.0);
            }

            reduced.colors_.emplace_back(colour);
        }

        // publish the inliers of the plane model as a separate cloud
        open3d_conversions::open3dToRos(reduced, rosDebugCloud, "laser_link");
        lidarDebugPub.publish(rosDebugCloud);
    }

    // publish detect debug
    if (!lidarDebugTopicName.empty()) {
        // TODO
    }

    double time = (ros::WallTime::now() - start).toSec() * 1000.0;
    ROS_INFO("Lidar callback time: %.2f ms", time);
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lidar_cone");
    ROS_INFO("LiDAR Cone Detection v" LIDAR_CONE_VERSION ": Matt Young, Fahed Alhanaee, Riley Bowyer, Caleb Aitken, 2021-2022, UQRacing");

    ros::NodeHandle handle{};
    LidarConeDetector detector = LidarConeDetector(handle);
    ROS_INFO("Initialised ROS node");

    ros::spin();
}