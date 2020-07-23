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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>

namespace uqr {
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
            void to_range_image(uqr::PointCloud& input_cloud);

            void calibrate_cam(uqr::PointCloud& input_cloud);
            void set_viewer_pose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);
            
            /// Destructor
            ~OnlineSegmenter() = default;

        private:
            pcl::RangeImage::CoordinateFrame coordinate_frame;
            pcl::RangeImage::Ptr rangeImage;

            Eigen::Affine3f scene_sensor_pose;
            float x_trans;
            float y_trans;
            float z_trans;

            float w_rot;
            float x_rot;
            float y_rot;
            float z_rot;

            double vertical_fov; 
            double horizontal_fov;

            double vertical_angle_res;
            double horizontal_angle_res;

            double noise_level;
            double min_range;
            int border_size;

            bool calibrate_pose;
            ros::NodeHandle nh;
    };

};
#endif //ONLINE_SEGMENTATION_H
