/**
 * @author Caleb Aitken
 * @date 14-07-2020
 *
 * @brief pcl wrapper library header
 *  This library introduces:
 *    + A custom data type that has implicit conversions between all PointCloud data types used in this project
 *      from other libraries
 *    + Simplified pcl functions
 *
 * @namespace uqr
 *  @class PointCloud cross-library PointCloud adaptor
 */

// TODO:
//  - documentation

#ifndef LIDAR_CONES_DETECTION_PCLWRAPPER_H
#define LIDAR_CONES_DETECTION_PCLWRAPPER_H

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
     * @breif cross-library PointCloud adaptor
     *
     * Allows easy conversions between pcl::PCLPointCloud2, pcl::PointCloud<pcl::PointXYZ>, & sensor_msgs::PointCloud2
     */
    class PointCloud {
    public:
        /// empty PointCLoud constructor
        PointCloud() = default;

        /// copy constructor
        PointCloud(const PointCloud& otherPointCloud) = default;

        /// copy operator
        PointCloud& operator=(const PointCloud&) = default;

        /// move constructor
        PointCloud(PointCloud&&) = default;

        /// move operator
        PointCloud& operator=(PointCloud&&) = default;

        /// sensor_msgs::PointCloud2 constructor
        PointCloud(const sensor_msgs::PointCloud2& otherPointCloud);

        /// pcl::PCLPointCLoud2 constructor
        PointCloud(const pcl::PCLPointCloud2& otherPointCloud);

        /// pcl::PointCloud<pcl::PointXYZ>
        PointCloud(const pcl::PointCloud<pcl::PointXYZ>& otherPointCloud);

        /// implicit conversion to sensor_msgs::PointCloud2
        operator sensor_msgs::PointCloud2();

        /// implicit conversion to pcl::PCLPointCloud2
        operator pcl::PCLPointCloud2();

        /// implicit conversion to pcl::PointCloud<pcl::PointXYZ>
        operator pcl::PointCloud<pcl::PointXYZ>();

        /// Destructor
        ~PointCloud() = default;

    private:
        int type = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = nullptr;
        pcl::PCLPointCloud2::Ptr pclPointCloud2 = nullptr;
        sensor_msgs::PointCloud2::Ptr sensorPointCloud2 = nullptr;
    };

    // TODO: these static functions
    //      also figure out if they're meant to be static or inline??
    //      and decide if they return a new cloud, or modify the existing cloud
    //          OR take a pointer to a second cloud as another parameter and load the new cloud into that??
    static void voxelise();
    static void pass_through_filter();
    static void radius_outlier_removal();
    static void sac_segmentation();
    static void extract_indices();
};

#endif //LIDAR_CONES_DETECTION_PCLWRAPPER_H
