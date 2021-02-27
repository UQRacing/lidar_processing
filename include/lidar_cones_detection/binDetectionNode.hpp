
#ifndef LIDAR_CONES_DETECTION_BINDETECTORNODE_H
#define LIDAR_CONES_DETECTION_BINDETECTORNODE_H
// Local Files
#include "lidar_cones_detection/UQRPointCloud.hpp"
#include "lidar_cones_detection/binDetector.hpp"


class BinDetectionNode {
  public:
      // constructor
      BinDetectionNode();

      // default destructor
      ~BinDetectionNode() = default;

      // deleted copy constructor
      BinDetectionNode(const BinDetectionNode&) = delete;

      // deleted copy operator
      BinDetectionNode& operator=(BinDetectionNode&) = delete;

      void pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr& scan);

  private:
      // ROS Node Handle
      ros::NodeHandle node;

      // ROS Interface
      ros::Subscriber pointcloud_sub;
      ros::Publisher observation_pub;
      
      BinDetector detector;
};

#endif // LIDAR_CONES_DETECTION_BINDETECTORNODE_H