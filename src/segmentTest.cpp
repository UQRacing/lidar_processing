#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include "lidar_cones_detection/OnlineSegmentation.hpp"


int main (int argc, char** argv){
  ros::init(argc, argv, "lidar_cone");
  ros::NodeHandle nh;
  uqr::OnlineSegmenter segmenter;

  uqr::PointCloud cloud;
  std::string filePath;
  nh.param<std::string>("/segmenter/file_path", filePath, "table.pcd");
  uqr::load_cloud(filePath, cloud);

  segmenter.to_range_image(cloud);

  ros::spin();
  return 0;
}