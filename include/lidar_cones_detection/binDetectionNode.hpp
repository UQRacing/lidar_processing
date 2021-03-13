/*******************************************************************************************************************//**
 *  @file    binDetectionNode.hpp
 *  @brief   Provides a ROS interface for the bin filtering detection node.
 *
 *  @author  Riley Bowyer (riley.d.bowyer@gmail.com)
 *  @date    March 2021
 *
***********************************************************************************************************************/


#ifndef LIDAR_CONES_DETECTION_BINDetectorNODE_H
#define LIDAR_CONES_DETECTION_BINDetectorNODE_H


// Local Files
#include "lidar_cones_detection/UQRPointCloud.hpp"
#include "lidar_cones_detection/binDetector.hpp"


class BinDetectionNode {
  public:
      
      /**
       * @brief Default Constructor.
       *
       */
      BinDetectionNode();

      
      /**
       * @brief Default Deconstructor.
       *
       */
      ~BinDetectionNode() = default;
      
      
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
      
      BinDetector detector;
};

#endif // LIDAR_CONES_DETECTION_BINDetectorNODE_H