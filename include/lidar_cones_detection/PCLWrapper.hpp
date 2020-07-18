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
     * @brief cross-library PointCloud adaptor
     *
     * Allows easy conversions between pcl::PCLPointCloud2, pcl::PointCloud<pcl::PointXYZ>, & sensor_msgs::PointCloud2
     */
    class PointCloud {
    public:
        /// Empty PointCLoud Constructor
        PointCloud();

        /// Copy Constructor
        PointCloud(const PointCloud& otherPointCloud) = default;

        /// Copy Operator
        PointCloud& operator=(const PointCloud&) = default;

        /// Move Constructor
        PointCloud(PointCloud&&) = default;

        /// Move Operator
        PointCloud& operator=(PointCloud&&) = default;

        /// sensor_msgs::PointCloud2 Constructor
        PointCloud(const sensor_msgs::PointCloud2& otherPointCloud);

        /// pcl::PCLPointCLoud2 Constructor
        PointCloud(const pcl::PCLPointCloud2& otherPointCloud);

        /// pcl::PointCloud<pcl::PointXYZ> Constructor
        PointCloud(const pcl::PointCloud<pcl::PointXYZ>& otherPointCloud);

        /// Implicit Conversion to sensor_msgs::PointCloud2
        operator sensor_msgs::PointCloud2() const;

        /// Implicit Conversion to pcl::PCLPointCloud2
        operator pcl::PCLPointCloud2() const;

        /// Implicit Conversion to pcl::PointCloud<pcl::PointXYZ>
        operator pcl::PointCloud<pcl::PointXYZ>() const;

        /// Destructor
        ~PointCloud() = default;

        using Ptr = boost::shared_ptr<::uqr::PointCloud>;
        using ConstPtr = boost::shared_ptr<const ::uqr::PointCloud>;

    private:
        /// Define Cloud Storage Enumerator
        enum storedCloud{
            NO_CLOUD    = 0,
            SENSOR_MSGS = 1,
            PCL_PC2     = 2,
            PCL_PXYZ    = 3,
        };

        int currentType = storedCloud::NO_CLOUD;

        // Define Pointers for each of the conversions
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud = nullptr;
        pcl::PCLPointCloud2::Ptr pclPointCloud2 = nullptr;
        sensor_msgs::PointCloud2::Ptr sensorPointCloud2 = nullptr;
    };

    /**
     * Voxelise an input cloud.
     *
     * A wrapper for the PCL voxelisation functions to
     * produce a uniformally voxelised pointcloud. 
     *
     * @param inputCloud Pointer to the desired input cloud. This data will be voxelised.
     * @param outputCloud Pointer to the desired output cloud. The voxelised cloud will be saved here.
     * @param voxel_size Size in metres of the desired voxels.
     */
    void voxelise(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud, const float voxel_size);

    /**
     * Perform pass-through filtering.
     *
     * A wrapper for the PCL pass-through filter functions to
     * produce a sectioned pointcloud. 
     *
     * @param inputCloud Pointer to the desired input cloud. This data will be filtered.
     * @param outputCloud Pointer to the desired output cloud. The voxelised cloud will be saved here.
     * @param field_name The input field to be evaluated.
     * @param lower_limit The lower bound of the filter.
     * @param upper_limit The upper bound of the filter.
     * @param invert Whether or not to invert the selected points.
     */
    void pass_through_filter(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud,
                             const std::string&  field_name, const double lower_limit,
                             const double upper_limit, const bool invert);
    
    /**
     * Perform radius outlier filtering.
     *
     * A wrapper for the PCL radius-outlier filter functions to
     * produce a reduced pointcloud. 
     *
     * @param inputCloud Pointer to the desired input cloud. This data will be filtered.
     * @param outputCloud Pointer to the desired output cloud. The voxelised cloud will be saved here.
     * @param radius The radius of the sphere in which to look for neighbours.
     * @param neighbours The minimum number of neighbours for a point to be kept.
     */
    void radius_outlier_removal(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud,
                                const double radius, const double neighbours);

    /**
     * Sample Consensus Segmentation.
     *
     * A wrapper for the SACSegmenter to
     * produce a segmented cloud.
     *
     * @param inputCloud Pointer to the desired input cloud. This data will be segmented.
     * @param outputCoeff Pointer to the desired output coefficients. The resultant coefficients will be saved here.
     * @param outputIndex Pointer to the desired output indicies. The segmented indicies will be saved here.
     * @param model The desired model type. Types can be found at pcl/sample_consensus/model_types.h.
     * @param method The desired method type. Types can be found at pcl/sample_consensus/method_types.h.
     * @param threshold The minimum distance to the model.
     * @param optimize_coeff Whether or not to optimise the coefficients.
     */
    void sac_segmentation(const uqr::PointCloud::ConstPtr& inputCloud, pcl::ModelCoefficients &outputCoeff, pcl::PointIndices &outputIndex,
                          const int model, const int method, const double threshold, const bool optimize_coeff);

    /**
     * Index Subtraction.
     *
     * A wrapper for the PCL index removal to
     * remove the specified indicies from a cloud.
     *
     * @param inputCloud Pointer to the desired input cloud. This data will be subtracted.
     * @param outputCloud Pointer to the desired output cloud. The voxelised cloud will be saved here.
     * @param subtraction Pointer to the desired indicies.
     * @param invert Whether or not to invert the selected points.
     */
    void subtract_indices(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud, const pcl::PointIndices::Ptr& subtraction, const bool invert);

    /**
     * Input Condition Filtering.
     *
     * A wrapper for the PCL filter condition to
     * filter a cloud based on user defined conditions.
     * Sample condition:
     * pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
     * range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
     * range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
     *
     * @param inputCloud Pointer to the desired input cloud. This data will be subtracted.
     * @param outputCloud Pointer to the desired output cloud. The voxelised cloud will be saved here.
     * @param condition The Condition object used for filtering.
     */
    void conditional_filter(const uqr::PointCloud::ConstPtr &inputCloud, uqr::PointCloud &outputCloud,
                            pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition);
};

#endif //LIDAR_CONES_DETECTION_PCLWRAPPER_H
