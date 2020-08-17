#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"

class sacImplementation{
  public:
    sacImplementation();
    void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud);

  private:
    // ros node handle
    ros::NodeHandle nh;
    float voxel;
    std::string topic;

    // ros subscriber; subs to lidar data output
    ros::Subscriber cloudSub;
    uqr::cloudPublisher groundPub;
    uqr::cloudPublisher obstaclePub;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
};

sacImplementation::sacImplementation(){
  groundPub = uqr::cloudPublisher("/ground");
  obstaclePub = uqr::cloudPublisher("/obstacle");

  coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
  inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);

  nh.param<float>("/segmenter/voxel_size", voxel, 0.01);
  nh.param<std::string>("/segmenter/topic", topic, "/cloud");
  
  cloudSub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 10, &sacImplementation::cloud_cb, this);
}

void sacImplementation::cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  uqr::PointCloud::Ptr incloud;
  *incloud = uqr::PointCloud(*cloud);
  uqr::PointCloud cloudObstacles;
  uqr::PointCloud cloudGround;

  uqr::voxelise(incloud, *incloud, voxel);
  uqr::sac_segmentation(incloud, *coefficients, *inliers, pcl::SACMODEL_PLANE, pcl::SAC_RANSAC, 0.01, true);
  uqr::subtract_indices(incloud,cloudObstacles,inliers, true);
  uqr::subtract_indices(incloud,cloudGround,inliers, false);
}

int main (int argc, char** argv){
  ros::init(argc, argv, "lidar_cone");
  ros::NodeHandle nh;
  sacImplementation implementation;
  ros::spin();    
  return (0);
}