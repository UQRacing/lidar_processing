
#ifndef LIDAR_CONES_DETECTION_PlaneDetectorNODE_H
#define LIDAR_CONES_DETECTION_PlaneDetectorNODE_H
// Local Files
#include "lidar_cones_detection/UQRPointCloud.hpp"
#include "lidar_cones_detection/planeDetector.hpp"


class PlaneDetectionNode {
  public:
      // constructor
      PlaneDetectionNode();

      // default destructor
      ~PlaneDetectionNode() = default;

      // deleted copy constructor
      PlaneDetectionNode(const PlaneDetectionNode&) = delete;

      // deleted copy operator
      PlaneDetectionNode& operator=(PlaneDetectionNode&) = delete;

      void pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr& scan);

  private:
      // ROS Node Handle
      ros::NodeHandle node;

      // ROS Interface
      ros::Subscriber pointcloud_sub;
      ros::Publisher observation_pub;
      
      PlaneDetector detector;
};

#endif // LIDAR_CONES_DETECTION_PlaneDetectorNODE_H