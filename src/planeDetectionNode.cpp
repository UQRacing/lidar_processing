/*******************************************************************************************************************//**
 *  @file    planeDetectionNode.cpp
 *  @brief   Provides a ROS interface for the plane detection node.
 *
 *  @author  Riley Bowyer (riley.d.bowyer@gmail.com)
 *  @date    March 2021
 *
***********************************************************************************************************************/

// Main Include
#include "lidar_cones_detection/planeDetectionNode.hpp"


/**
 * @brief Default Constructor.
 *
 */
PlaneDetectionNode::PlaneDetectionNode() {
    this->pointcloud_sub  = this->node.subscribe<sensor_msgs::PointCloud2>("/cloud", 1, &PlaneDetectionNode::pointcloud_cb, this);
    this->observation_pub = node.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
}


/**
 * @brief Callback Function. Processes an incoming pointcloud
 *        and publishes the resultign feature set.
 * @param scan The incoming pointcloud
 *
 */
void PlaneDetectionNode::pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr& scan) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*scan, *inputCloud);
  
  auto coneBins = this->detector.update(inputCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto &cone: coneBins){
    for (auto &point : cone->points){
      processedCloud->points.push_back(point);
    }
  }
  
  sensor_msgs::PointCloud2 outputCloud;
  pcl::toROSMsg(*processedCloud, outputCloud);
  outputCloud.header = scan->header;
  
  this->observation_pub.publish(outputCloud);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "LidarConeDetector");
    PlaneDetectionNode detectionNode;
    ros::spin();
    return 0;
}