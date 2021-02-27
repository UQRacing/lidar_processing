/**
 * @author Caleb Aitken
 * @date 14-07-2020
 *
 * @brief pcl wrapper library sources
 */

#include "lidar_cones_detection/UQRPointCloud.hpp"

#include <utility>

// Empty Class Constructor
uqr::PointCloud::PointCloud() {
    this->storedCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

// Sensor Messaages Class Constructor
uqr::PointCloud::PointCloud(const sensor_msgs::PointCloud2& otherPointCloud) {
    pcl::fromROSMsg(otherPointCloud, *this->storedCloud);
    this->header = otherPointCloud.header;
}

// pcl::PCLPointCloud2 Class contructor
uqr::PointCloud::PointCloud(const pcl::PCLPointCloud2& otherPointCloud) {
    pcl::fromPCLPointCloud2(otherPointCloud, *this->storedCloud);
    this->header.stamp = ros::Time(otherPointCloud.header.stamp);
    this->header.frame_id = otherPointCloud.header.frame_id;
}

// pcl::PointCloud<pcl::PointXYZ> Class Constructor
uqr::PointCloud::PointCloud(const pcl::PointCloud<pcl::PointXYZ>& otherPointCloud) {
    this->storedCloud.reset(new pcl::PointCloud<pcl::PointXYZ>(otherPointCloud));
    this->header.stamp = ros::Time::now();
    this->header.frame_id = "map";
}

// pcl::PointCloud<pcl::PointXYZ>::Ptr Class Constructor
uqr::PointCloud::PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr otherPointCloud) {
    // Reset All Clouds and Assign Input to Corresponding Type
    this->storedCloud = otherPointCloud;  // MUST BE COPIED TO INCREMENT OWNER COUNT
}

// Conversion to sensor_msgs::PointCloud2
uqr::PointCloud::operator sensor_msgs::PointCloud2::Ptr() const {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr --> sensor_msgs::PointCloud2::Ptr
    sensor_msgs::PointCloud2::Ptr rosCloud;
    pcl::toROSMsg(*this->storedCloud, *rosCloud);
    rosCloud->header = this->header;
    return rosCloud;
}

// Conversion to pcl::PCLPointCloud2
uqr::PointCloud::operator pcl::PCLPointCloud2::Ptr() const {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr --> pcl::PCLPointCloud2::Ptr
    pcl::PCLPointCloud2::Ptr pclCloud;
    pcl::toPCLPointCloud2(*this->storedCloud, *pclCloud);
    return pclCloud;
}

// Conversion to pcl::PointCloud<pcl::PointXYZ>
uqr::PointCloud::operator pcl::PointCloud<pcl::PointXYZ>::Ptr() const {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr --> pcl::PointCloud<pcl::PointXYZ>::Ptr
    return this->storedCloud;
}

/**
 * Voxelise the cloud
 * 
 * @param voxel_size    Size in metres of the desired voxels. 
 */
void uqr::PointCloud::voxelise(const float voxel_size) {
    // Configure Filter and Input
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(this->storedCloud);

    // Configure Parameters
    filter.setLeafSize(voxel_size, voxel_size, voxel_size);

    // Replace stored cloud with filtered cloud
    filter.filter(*this->storedCloud);
}

/**
 * Perform pass-through filtering.
 * 
 * @param field_name    The input field to be evaluated.
 * @param lower_limit   The lower bound of the filter.
 * @param upper_limit   The upper bound of the filter.
 * @param invert        Whether or not to invert the selected points.
 */
void uqr::PointCloud::pass_through_filter(const std::string&  field_name, const double lower_limit,
                                          const double upper_limit, const bool invert) {

    // Configure Filter and Input
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(this->storedCloud);

    // Configure Parameters
    filter.setFilterFieldName(field_name);
    filter.setFilterLimits(lower_limit, upper_limit);
    filter.setFilterLimitsNegative(invert);

    // Replace stored cloud with filted cloud
    filter.filter(*this->storedCloud);
}

/**
 * Perform radius outlier filtering.
 * 
 * @param radius        The radius of the sphere in which to look for neighbours.
 * @param neighbours    The minimum number of neighbours for a point to be kept.
 */
void uqr::PointCloud::radius_outlier_removal(const double radius, const double neighbours) {

    // Configure Filter and Input
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(this->storedCloud);

    // Configure Parameters
    filter.setRadiusSearch(radius);
    filter.setMinNeighborsInRadius(neighbours);

    // Replace stored cloud with filtered cloud
    filter.filter(*this->storedCloud);
}

// Does not modify stored cloud, hence removed
//void uqr::PointCloud::sac_segmentation(const uqr::PointCloud::ConstPtr& inputCloud, pcl::ModelCoefficients &outputCoeff, pcl::PointIndices &outputIndex,
//                           const int model, const int method, const double threshold, const bool optimize_coeff) {
//
//    // Define input clouds for filtering
//    pcl::PointCloud<pcl::PointXYZ>::Ptr preSegment(new pcl::PointCloud<pcl::PointXYZ>((pcl::PointCloud<pcl::PointXYZ>) *inputCloud));
//
//    // Configure Segmenter and Input
//    pcl::SACSegmentation<pcl::PointXYZ> segment;
//    segment.setInputCloud(preSegment);
//
//    // Configure Parameters
//    segment.setOptimizeCoefficients(optimize_coeff);
//    segment.setModelType(model);
//    segment.setMethodType(method);
//    segment.setDistanceThreshold(threshold);
//
//    // Segment
//    segment.segment(outputIndex, outputCoeff);
//}

/**
 * Index Subtraction.
 * 
 * @param subtraction   Pointer to the desired indicies.
 * @param invert        Whether or not to invert the selected points.
 */
void uqr::PointCloud::subtract_indices(const pcl::PointIndices::Ptr& subtraction, const bool invert) {

    // Configure Filter and Input
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(this->storedCloud);
    extract.setIndices(subtraction);

    // Configure Parameters
    extract.setNegative(invert);

    // Replace stored cloud with filtered cloud
    extract.filter(*this->storedCloud);
}

/**
 * Input Condition Filtering.
 * 
 * @param condition The Condition object used for filtering.
 */
void uqr::PointCloud::conditional_filter(pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition) {
    // Configure Segmenter and Input
    pcl::ConditionalRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(this->storedCloud);

    // Configure Parameters
    filter.setCondition(condition);

    // Replace stored cloud with filtered cloud
    filter.filter(*this->storedCloud);
}
