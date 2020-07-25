#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include "lidar_cones_detection/OnlineSegmentation.hpp"


int main (int argc, char** argv){
  ros::init(argc, argv, "lidar_cone");
  ros::NodeHandle nh;
  uqr::PointCloud incloud;
  std::string filePath;
  nh.param<std::string>("/segmenter/file_path", filePath, "table.pcd");
  
  uqr::load_cloud(filePath, incloud);
  uqr::OnlineSegmenter segmenter;
  while(ros::ok()){
    ros::Time start = ros::Time::now();
    segmenter.segment(incloud);
    ros::Time end = ros::Time::now();
    double execution_time = (end - start).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
  }
}