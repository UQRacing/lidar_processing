#include <cloudFilter.hpp>

cloud_filter::cloud_filter(std::string topic, ros::NodeHandle nh, double voxelSize){
    // Assign class variables
    cloudSub_ = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, &cloud_filter::cloud_cb, this);
    cloud_filteredPub_ = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_filtered", 1);
    voxelSize_ = voxelSize;
}

void cloud_filter::cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setLeafSize(this->voxelSize_, this->voxelSize_, this->voxelSize_);
  sor.setInputCloud(cloudPtr);
  sor.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  this->cloud_filteredPub_.publish(output);
}

int main (int argc, char** argv)
{
// Initialize ROS
  ros::init (argc, argv, "samplePCLNode");
  ros::NodeHandle nh;

  // Initialise Class
  cloud_filter cloudFilter("/cloud", nh, 0.05); 

  // Spin
  while(nh.ok())
  {
    ros::spin();
  }
}