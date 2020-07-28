#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include <pcl/segmentation/progressive_morphological_filter.h>

int main (int argc, char** argv){
  ros::init(argc, argv, "lidar_cone");
  ros::NodeHandle nh;

  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setMaxWindowSize(1);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance(0.1f);
  pmf.setMaxDistance(0.3f);

  uqr::PointCloud::Ptr incloud(new uqr::PointCloud);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  uqr::PointCloud outcloud;

  std::string filePath;
  float voxel;
  double execution_time;

  nh.param<std::string>("/segmenter/file_path", filePath, "table.pcd");
  nh.param<float>("/segmenter/voxel_size", voxel, 0.01);
  
  uqr::load_cloud(filePath, *incloud);
  uqr::pass_through_filter(incloud, *incloud, "z", 0, 1.2, false);

  for(int i=0;i<1;i++){
    ros::Time start = ros::Time::now();

    uqr::voxelise(incloud, *incloud, voxel);
    *cloud = *incloud;
    pmf.setInputCloud(cloud);
    pmf.extract(inliers->indices);
    *incloud = *cloud;
    uqr::subtract_indices(incloud,outcloud,inliers, false);

    ros::Time end = ros::Time::now();
    execution_time += (end - start).toNSec() * 1e-6;
  }

  ROS_INFO("Cloud Size:%d Exectution time (ms): %0.3f",cloud->size(), execution_time/1);
  uqr::view_cloud(outcloud);
  // uqr::write_cloud("/home/ubuntu/UQR-DV/dv_ws/src/data/segmented.pcd",outcloud);
  return (0);
}