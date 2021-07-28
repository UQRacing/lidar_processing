/*******************************************************************************************************************//**
 *  @file    binDetectionNode.cpp
 *  @brief   Provides a ROS interface for the bin filtering detection node.
 *
 *  @author  Riley Bowyer (riley.d.bowyer@gmail.com)
 *  @date    March 2021
 *
***********************************************************************************************************************/

// Main Include
#include "lidar_cones_detection/binDetectionNode.hpp"


/**
 * @brief Default Constructor.
 *
 */
BinDetectionNode::BinDetectionNode() {
  this->pointcloud_sub  = this->node.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 1, &BinDetectionNode::pointcloud_cb, this);
  this->observation_pub = node.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
}


/**
 * @brief Callback Function. Processes an incoming pointcloud
 *        and publishes the resultign feature set.
 * @param scan The incoming pointcloud
 *
 */
void BinDetectionNode::pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr& scan) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*scan, *inputCloud);
  
  auto coneBins = this->detector.update(inputCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto &cone: coneBins){
    processedCloud->points.push_back(cone->maxPoint);
  }
  
  sensor_msgs::PointCloud2 outputCloud;
  pcl::toROSMsg(*processedCloud, outputCloud);
  outputCloud.header = scan->header;
  
  this->observation_pub.publish(outputCloud);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "LidarConeDetector");
    BinDetectionNode detectionNode;
    ros::spin();
    return 0;
}