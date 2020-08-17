/**
 * @author Riley Bowyer
 * @date 18-07-2020
 *
 * @brief IO Utility function library
 *  This library introduces:
 *    + Simplified ROS publication and subscription
 *    + Simplified pointcloud file  saving and reading.
 *
 * @namespace uqr
 */

#ifndef LIDAR_CONES_DETECTION_IOUTILITY_H
#define LIDAR_CONES_DETECTION_IOUTILITY_H

#include "lidar_cones_detection/PCLWrapper.hpp"
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

namespace uqr {

    /**
     * Load a pointcloud from a specified file.
     *
     * A wrapper for the PCL voxelisation functions to
     * produce a uniformally voxelised pointcloud. 
     *
     * @param file_path The path to the cloud, including file extension.
     * @param outputCloud Pointer to the desired output cloud. The loaded cloud will be saved here.
     */
    void load_cloud(const std::string& file_path, uqr::PointCloud& outputCloud);

    /**
     * Load a pointcloud from a specified file.
     *
     * A wrapper for the PCL voxelisation functions to
     * produce a uniformally voxelised pointcloud. 
     *
     * @param file_path The path to the cloud, including file extension.
     * @param input_cloud Pointer to the desired input cloud. This cloud will be saved.
     */
    void write_cloud(const std::string file_path, uqr::PointCloud& input_cloud);

    /**
     * @brief Simple PointCloud Publisher
     *
     * Allows easy visualisation of a variety of pointcloud types
     */
    class cloudPublisher {
        public:
            /// Constructor with Topic
            cloudPublisher(std::string topic);
            cloudPublisher(){};
            
            // Publish Methods
            /**
             * Publish a uqr::PointCloud
             *
             * @param cloud The cloud to be published.
             */
            void publish(PointCloud& cloud, std::string frame="map" , ros::Time timeStamp=ros::Time::now());

            /**
             * Publish a sensor_msgs::PointCloud2
             *
             * @param cloud The cloud to be published.
             * @param frame The frame in which the cloud is to be published.
             * @param timeStamp The time-stamp with which the cloud is to be published.
             */
            void publish(sensor_msgs::PointCloud2& cloud, std::string frame="map" , ros::Time timeStamp=ros::Time::now());

            /**
             * Publish a pcl::PCLPointCloud2
             *
             * @param cloud The cloud to be published.
             * @param frame The frame in which the cloud is to be published.
             * @param timeStamp The time-stamp with which the cloud is to be published.
             */
            void publish(pcl::PCLPointCloud2& cloud, std::string frame="map" , ros::Time timeStamp=ros::Time::now());

            /**
             * Publish a pcl::PointCloud<pcl::PointXYZ>
             *
             * @param cloud The cloud to be published.
             * @param frame The frame in which the cloud is to be published.
             * @param timeStamp The time-stamp with which the cloud is to be published.
             */
            void publish(pcl::PointCloud<pcl::PointXYZ>& cloud, std::string frame="map" , ros::Time timeStamp=ros::Time::now());

            /// Destructor
            ~cloudPublisher() = default;

        private:
            ros::NodeHandle nh;
            ros::Publisher cloudPub;

    };

};
#endif //LIDAR_CONES_DETECTION_IOUTILITY_H
