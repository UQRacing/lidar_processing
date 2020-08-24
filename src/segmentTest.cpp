#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include "lidar_cones_detection/OnlineSegmentation.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main (int argc, char** argv){
  ros::init(argc, argv, "lidar_cone");
  ros::NodeHandle nh;
  uqr::PointCloud incloud;
  std::string filePath;
  nh.param<std::string>("/segmenter/file_path", filePath, "table.pcd");
  
  uqr::load_cloud(filePath, incloud);
  pcl::PointCloud<pcl::PointXYZ> cloud = (pcl::PointCloud<pcl::PointXYZ>)incloud;

  uqr::ProjectionParams rowAngles((float)-0.262,(float)0.262,16);
  uqr::ProjectionParams colAngles((float)-3.1415,(float)3.1415,870);

  uqr::Projector projector(rowAngles,colAngles);
  projector.convert(cloud);
  
  cv_bridge::CvImage out_msg;
  cv::Mat outImg;
  projector.get_depth().convertTo(outImg, 0, 255/projector.get_max_depth());
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = "map";
  out_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
  out_msg.image    = outImg;
  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("depth", 1000);
  while(ros::ok()){
    img_pub.publish(out_msg.toImageMsg());
    ros::spinOnce();
  }
}