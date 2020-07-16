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

    //      For input/output stuff, I'm a fan of leaving the input as be and modiyfing an output pointer.

    static void voxelise(PointCloud &inputCloud, PointCloud &outputCloud, const double voxel_size){
        // Convert to PCLPointCloud2 Format
        pcl::PCLPointCloud2 pc2InputCloud(inputCloud);

        // Define input and output clouds for filtering
        pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2);
        pcl::PCLPointCloud2 cloud_filtered;
        
        // Copy across data
        pcl::copyPointCloud(pc2InputCloud, *preFilter);

        // Define Input
        pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
        filter.setInputCloud(preFilter);

        // Configure Parameters
        filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        
        // Filter
        filter.filter(cloud_filtered);

        // Assign to output pointer
        outputCloud = cloud_filtered;
    }

    static void pass_through_filter(PointCloud &inputCloud, PointCloud &outputCloud,
                                    const std::string  field_name, const double lower_limit,
                                    const double upper_limit, const bool invert_filter){
        // Convert to PCLPointCloud2 Format
        pcl::PCLPointCloud2 pc2InputCloud(inputCloud);

        // Define input and output clouds for filtering
        pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2);
        pcl::PCLPointCloud2 cloud_filtered;
        
        // Copy across data
        pcl::copyPointCloud(pc2InputCloud, *preFilter);
        
        // Define Input
        pcl::PassThrough<pcl::PCLPointCloud2> filter;
        filter.setInputCloud(preFilter);

        // Configure Parameters
        filter.setFilterFieldName(field_name);
        filter.setFilterLimits(lower_limit, upper_limit);
        filter.setFilterLimitsNegative(invert_filter);

        // Filter
        filter.filter(cloud_filtered);

        outputCloud = cloud_filtered;
    }
    
    static void radius_outlier_removal(PointCloud &inputCloud, PointCloud &outputCloud, 
                                       const double radius, const double neighbours){
        // Convert to PCLPointCloud2 Format
        pcl::PCLPointCloud2 pc2InputCloud(inputCloud);

        // Define input and output clouds for filtering
        pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2);
        pcl::PCLPointCloud2 cloud_filtered;
        
        // Copy across data
        pcl::copyPointCloud(pc2InputCloud, *preFilter);
        
        // Define Input
        pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> filter;
        filter.setInputCloud(preFilter);

        // Configure Parameters
        filter.setRadiusSearch(radius);
        filter.setMinNeighborsInRadius(neighbours);

        // Filter
        filter.filter(cloud_filtered);

        outputCloud = cloud_filtered;
    }

    static void sac_segmentation(PointCloud &inputCloud, pcl::ModelCoefficients &outputCoeff, pcl::PointIndices &outputIndex,
                                 const int model, const int method, const double threshold, const bool optimize_coeff){

        // Method Types can be found at pcl/sample_consensus/method_types.h
        // Model Types can be found at pcl/sample_consensus/model_types.h

        // Convert to PCLPointCloud2 Format
        pcl::PointCloud<pcl::PointXYZ> pclInputCloud(inputCloud);

        // Define input clouds for filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr preSegment(new pcl::PointCloud<pcl::PointXYZ>);

        // Copy across data
        pcl::copyPointCloud(pclInputCloud, *preSegment);
        
        // Define Input
        pcl::SACSegmentation<pcl::PointXYZ> segment;
        segment.setInputCloud(preSegment);

        // Configure Parameters        
        segment.setOptimizeCoefficients(optimize_coeff);
        segment.setModelType(model);
        segment.setMethodType(method);
        segment.setDistanceThreshold(threshold);

        // Segment
        segment.segment(outputIndex, outputCoeff);
    }

    static void extract_indices();

    static void conditional_filter( PointCloud &inputCloud, PointCloud &outputCloud, 
                                    pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition){
        // Sample condition: 
        // pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
        // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
        // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));

        // Convert to PCL::PointCloud Format
        pcl::PointCloud<pcl::PointXYZ> pclInputCloud(inputCloud);

        // Define input and output clouds for filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr preFilter(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        
        // Copy across data
        pcl::copyPointCloud(pclInputCloud, *preFilter);
        
        // Define Input
        pcl::ConditionalRemoval<pcl::PointXYZ> filter;
        filter.setInputCloud(preFilter);

        // Configure Parameters
        filter.setCondition(condition);

        // Filter
        filter.filter(cloud_filtered);

        outputCloud = cloud_filtered;
    }
};

#endif //LIDAR_CONES_DETECTION_PCLWRAPPER_H
