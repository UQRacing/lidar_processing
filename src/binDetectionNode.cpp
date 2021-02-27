
// Main Include
#include "lidar_cones_detection/binDetectionNode.hpp"

BinDetectionNode::BinDetectionNode() {
    this->pointcloud_sub  = this->node.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 100, &BinDetectionNode::pointcloud_cb, this);
    this->observation_pub = node.advertise<sensor_msgs::PointCloud2>("output_cloud", 100);
    // this->detector = BinDetector();
}

void BinDetectionNode::pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr& scan) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*scan, *inputCloud);
  
  auto coneBins = this->detector.update(inputCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto &cone: coneBins){
    processedCloud->points.push_back(cone.maxPoint);
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