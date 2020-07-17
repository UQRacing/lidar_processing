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

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>

namespace uqr {

    /**
     * @breif cross-library PointCloud adaptor
     *
     * Allows easy conversions between pcl::PCLPointCloud2, pcl::PointCloud<pcl::PointXYZ>, & sensor_msgs::PointCloud2
     */
    class PointCloud {
    public:
        /// empty PointCLoud constructor
        PointCloud();

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
        operator sensor_msgs::PointCloud2() const;

        /// implicit conversion to pcl::PCLPointCloud2
        operator pcl::PCLPointCloud2() const;

        /// implicit conversion to pcl::PointCloud<pcl::PointXYZ>
        operator pcl::PointCloud<pcl::PointXYZ>() const;

        /// Destructor
        ~PointCloud() = default;

        using Ptr = boost::shared_ptr<::uqr::PointCloud>;
        using ConstPtr = boost::shared_ptr<const ::uqr::PointCloud>;

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

    //      For input/output stuff, I'm a fan of leaving the input as be and modiyfing an output pointer.

    void voxelise(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud, const float voxel_size);

    void pass_through_filter(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud,
                             const std::string&  field_name, const double lower_limit,
                             const double upper_limit, const bool invert_filter);
    
    void radius_outlier_removal(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud,
                                const double radius, const double neighbours);

    void sac_segmentation(const uqr::PointCloud::ConstPtr& inputCloud, pcl::ModelCoefficients &outputCoeff, pcl::PointIndices &outputIndex,
                          const int model, const int method, const double threshold, const bool optimize_coeff);

    void subtract_indices(const uqr::PointCLoud::ConstPtr& inputCloud, uqr::PointCLoud& outputCloud, const pcl::PointIndices::Ptr& subtraction);

    void conditional_filter(const uqr::PointCloud::ConstPtr &inputCloud, uqr::PointCloud &outputCloud,
                            pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition);
};

#endif //LIDAR_CONES_DETECTION_PCLWRAPPER_H
