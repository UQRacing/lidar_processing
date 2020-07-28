#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"

int main (int argc, char** argv){
  ros::init(argc, argv, "lidar_cone");
  ros::NodeHandle nh;
  uqr::PointCloud::Ptr incloud(new uqr::PointCloud);
  uqr::PointCloud::Ptr filterCloud(new uqr::PointCloud);
  std::string filePath;
  float voxel;
  nh.param<std::string>("/segmenter/file_path", filePath, "table.pcd");
  nh.param<float>("/segmenter/voxel_size", voxel, 0.01);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  uqr::PointCloud outcloud;
  
  uqr::load_cloud(filePath, *incloud);
  uqr::pass_through_filter(incloud, *incloud, "z", 0, 1.2, false);
  pcl::PointCloud<pcl::PointXYZ> cloud = *incloud;
  double execution_time;

  for(int i=0;i<1000;i++){
    ros::Time start = ros::Time::now();

    uqr::voxelise(incloud, *filterCloud, voxel);
    uqr::sac_segmentation(filterCloud, *coefficients, *inliers, pcl::SACMODEL_PLANE, pcl::SAC_RANSAC, 0.01, true);
    uqr::subtract_indices(filterCloud,outcloud,inliers, true);

    ros::Time end = ros::Time::now();
    execution_time += (end - start).toNSec() * 1e-6;
  }
  ROS_INFO("Cloud Size:%d Exectution time (ms): %0.3f",cloud.size(), execution_time/1000);
  uqr::write_cloud("/home/ubuntu/UQR-DV/dv_ws/src/data/segmented.pcd",outcloud);
  return (0);
}