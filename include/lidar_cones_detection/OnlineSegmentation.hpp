/**
 * @author Riley Bowyer
 * @date 23-07-2020
 *
 * @brief Online Cloud Segmenter library
 *  This library introduces:
 *    + Segmenter to identify cones
 *
 * @namespace uqr
 */

#ifndef ONLINE_SEGMENTATION_H
#define ONLINE_SEGMENTATION_H

#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>

namespace uqr {
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

    /**
     * @brief Online Cloud Segmenter
     *
     * Segments clouds to identifdy cones.
     */
    class OnlineSegmenter {
        public:
            /// Constructor 
            OnlineSegmenter();

            /// Functions
            void segment(uqr::PointCloud input_cloud);

            void angle_column();

            /// Destructor
            ~OnlineSegmenter() = default;

        private:
            double focal_x;
            double focal_y;

            int height;
            int width;
            int centre_x;
            int centre_y;
            int x_offset;
            int y_offset;

            bool view_image;
            ros::NodeHandle nh;

            /// Debug
            ros::Publisher imagePub;
  
    };

};
#endif //ONLINE_SEGMENTATION_H
