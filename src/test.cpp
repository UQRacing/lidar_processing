/**
 * @author Caleb Aitken
 * @brief this file is for testing purposes
 */


#include "lidar_cones_detection/PCLWrapper.hpp"
#include "pcl/io/pcd_io.h"

// sorry riley i didn't save your test loop somewhere before chucking all these tests in

class NodeWrapper {
public:
    /// constructor
    explicit NodeWrapper();

    /// default destructor
    ~NodeWrapper() = default;

    /// deleted copy constructor
    NodeWrapper(const NodeWrapper&) = delete;

    /// deleted copy operator
    NodeWrapper& operator=(NodeWrapper&) = delete;

    /**
     * Handle data published by lidar
     *
     * @param scan sensor_msgs::PointCloud2 published by the lidar
     */
    void lidarReadCallback(const sensor_msgs::PointCloud2::ConstPtr& scan);

private:
    // ros node handle
    ros::NodeHandle node;

    // ros subscriber; subs to lidar data output
    ros::Subscriber lidarReadSub;
    ros::Publisher out;
};

/**
 * Set up subscriber
 */
NodeWrapper::NodeWrapper() {
    this->lidarReadSub = this->node.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 100, &NodeWrapper::lidarReadCallback, this);
    this->out = node.advertise<sensor_msgs::PointCloud2>("output_cloud", 100);
}

/**
 * Handle data published by lidar
 *
 * @param scan sensor_msgs::PointCloud2 published by the lidar
 */
void NodeWrapper::lidarReadCallback(const sensor_msgs::PointCloud2::ConstPtr& scan) {
    uqr::PointCloud::Ptr pointCloud(new uqr::PointCloud(*scan));
    uqr::PointCloud::Ptr pointCloudOut(new uqr::PointCloud);
    uqr::voxelise(pointCloud, *pointCloudOut, 0.5);

    sensor_msgs::PointCloud2 inputCloud = *pointCloud;
    sensor_msgs::PointCloud2 outputCloud = *pointCloudOut;

    inputCloud.header.frame_id = "map";
    inputCloud.header.stamp = ros::Time::now();
    outputCloud.header.frame_id = "map";
    outputCloud.header.stamp = ros::Time::now();

    out.publish(outputCloud);

    // simplest check for size change
    // std::cout << inputCloud.height * inputCloud.width << std::endl;
    // std::cout << outputCloud.height * outputCloud.width << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_cone");
    NodeWrapper nodeWarpper;
    ros::spin();
    return 0;
}