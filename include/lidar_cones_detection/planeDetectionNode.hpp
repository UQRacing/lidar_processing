/*******************************************************************************************************************//**
 *  @file    planeDetectionNode.hpp
 *  @brief   Provides a ROS interface for the plane detection node.
 *
 *  @author  Riley Bowyer (riley.d.bowyer@gmail.com)
 *  @date    March 2021
 *
***********************************************************************************************************************/


#ifndef LIDAR_CONES_DETECTION_PlaneDetectorNODE_H
#define LIDAR_CONES_DETECTION_PlaneDetectorNODE_H


// Local Files
#include "lidar_cones_detection/UQRPointCloud.hpp"
#include "lidar_cones_detection/planeDetector.hpp"


class PlaneDetectionNode {
  public:
      
      /**
       * @brief Default Constructor.
       *
       */
      PlaneDetectionNode();

      
      /**
       * @brief Default Deconstructor.
       *
       */
      ~PlaneDetectionNode() = default;
      
      
      /**
       * @brief Callback Function. Processes an incoming pointcloud
       *        and publishes the resultign feature set.
       * @param scan The incoming pointcloud
       *
       */
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