// LiDAR cone detector, main file
// Matt Young (with much prior work done by Riley Bowyer & Caleb Aitken), 2022, UQRacing
#include "lidar_cone_detection/lidar_cone.h"
#include "lidar_cone_detection/defines.h"
#include <ros/ros.h>
#include <pangolin/display/display.h>
#include <cilantro/cilantro.hpp>
#include <cilantro/core/data_containers.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace uqr;

LidarConeDetector::LidarConeDetector(ros::NodeHandle &handle) {
    if (!handle.getParam("/lidar_cone_detector/lidar_topic", lidarTopicName)) {
        ROS_ERROR("Failed to load lidar_topic from lidar cone config YAML");
    }
    if (!handle.getParam("/lidar_cone_detector/debug_ui", enableDebugUI)) {
        ROS_ERROR("bruh cringe");
    }

    lidarSub = handle.subscribe(lidarTopicName, 1, &LidarConeDetector::lidarCallback, this);
    // TODO conePub (figure out what message type we publish)

    if (enableDebugUI) {
        ROS_INFO("Enabling Pangolin debug UI");
        std::string windowName = "LiDAR Cone Detection";
        pangolin::CreateWindowAndBind(windowName, 1600, 900);
        viz = cilantro::Visualizer(windowName, "disp");
    }
}

void LidarConeDetector::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &rosCloud) {
    auto start = ros::WallTime::now();

    // inspiration for this method comes from:
    // https://github.com/ros-perception/perception_open3d/blob/main/open3d_conversions/src/open3d_conversions.cpp

    // hack(?) to convert the const pointer to a non-const pointer, which, for some stupid reason,
    // the iterator requires (it appears the PCL developers in their infinite wisdom chose to classify
    // the mutable and immutable iterators as the same thing)
    sensor_msgs::PointCloud2 rosCloudData = *rosCloud;

    // iterate over ROS point cloud fields
    auto iterX = sensor_msgs::PointCloud2Iterator<float>(rosCloudData, "x");
    auto iterY = sensor_msgs::PointCloud2Iterator<float>(rosCloudData, "y");
    auto iterZ = sensor_msgs::PointCloud2Iterator<float>(rosCloudData, "z");

    cilantro::PointCloud3f cloud;
    std::vector<float> points{};
    // it appears, somehow, _not_ pre-allocating the buffer is actually faster
    //points.reserve(rosCloud->width * rosCloud->height); // pre-allocate point cloud to save growing
    for (size_t i = 0; i < rosCloud->width * rosCloud->height; ++i, ++iterX, ++iterY, ++iterZ) {
        points.push_back(*iterX);
        points.push_back(*iterY);
        points.push_back(*iterZ);
    }
    cilantro::DataMatrixMap3f data(points);
    cloud.points = data;

    ROS_INFO("have %zu points", cloud.size());

    if (enableDebugUI) {
        viz->clear();
        viz->addObject<cilantro::PointCloudRenderable>("cloud", cloud.points,
                                                       cilantro::RenderingProperties().setPointColor(1, 0, 0).setPointSize(3.0));
    }
    viz->spinOnce(); // segfaults, probably because it's not on main thread (or data is corrupt somehow?)

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