/**
 * @author Caleb Aitken
 * @date 14-07-2020
 *
 * @brief pcl wrapper library sources
 */

#include "lidar_cones_detection/PCLWrapper.hpp"

// Empty Class Constructor
uqr::PointCloud::PointCloud() {
    this->pclPointCloud2.reset(new pcl::PCLPointCloud2);
    this->pclPointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->sensorPointCloud2.reset(new sensor_msgs::PointCloud2);
}

// Sensor Messaages Class Constructor
uqr::PointCloud::PointCloud(const sensor_msgs::PointCloud2& otherPointCloud) {
    // Reset All Clouds and Assign Input to Corresponding Type
    this->pclPointCloud.reset();
    this->pclPointCloud2.reset();
    this->sensorPointCloud2 = boost::make_shared<sensor_msgs::PointCloud2>(sensor_msgs::PointCloud2(otherPointCloud));
    this->currentType = storedCloud::SENSOR_MSGS;
}

uqr::PointCloud::PointCloud(const pcl::PCLPointCloud2& otherPointCloud) {
    // Reset All Clouds and Assign Input to Corresponding Type
    this->pclPointCloud.reset();
    this->sensorPointCloud2.reset();
    this->pclPointCloud2 = boost::make_shared<pcl::PCLPointCloud2>(pcl::PCLPointCloud2(otherPointCloud));
    this->currentType = storedCloud::PCL_PC2;
}

uqr::PointCloud::PointCloud(const pcl::PointCloud<pcl::PointXYZ>& otherPointCloud) {
    // Reset All Clouds and Assign Input to Corresponding Type
    this->sensorPointCloud2.reset();
    this->pclPointCloud2.reset();
    this->pclPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>(otherPointCloud));
    this->currentType = storedCloud::PCL_PXYZ;
}

// Conversion to sensor_msgs::PointCloud2
uqr::PointCloud::operator sensor_msgs::PointCloud2() const {
    switch (this->currentType) {
        case storedCloud::NO_CLOUD: {
            // None --> sensor_msgs::PointCloud2
            return sensor_msgs::PointCloud2();

        } case storedCloud::SENSOR_MSGS: {
            // sensor_msgs::PointCloud2 --> sensor_msgs::PointCloud2
            return sensor_msgs::PointCloud2(*this->sensorPointCloud2);

        } case storedCloud::PCL_PC2: {
            // pcl::PCLPointCloud2 --> sensor_msgs::PointCloud2
            sensor_msgs::PointCloud2 rosCloud;
            pcl_conversions::fromPCL(*this->pclPointCloud2, rosCloud);
            return rosCloud;

        } case storedCloud::PCL_PXYZ: {
            // pcl::PointCloud<pcl::PointXYZ> --> sensor_msgs::PointCloud2
            sensor_msgs::PointCloud2 rosCloud;
            pcl::toROSMsg(*this->pclPointCloud, rosCloud);
            return rosCloud;

        } default: {
            // Unkown Identifier
            throw std::out_of_range("uqr::PointCloud type identifier out of range");
        }
    }
}

// Conversion to pcl::PCLPointCloud2
uqr::PointCloud::operator pcl::PCLPointCloud2() const {
    switch (this->currentType) {
        case storedCloud::NO_CLOUD: {
            // None --> pcl::PCLPointCloud2
            return pcl::PCLPointCloud2();

        } case storedCloud::SENSOR_MSGS: {
            // sensor_msgs::PointCloud2 --> pcl::PCLPointCloud2
            pcl::PCLPointCloud2 pclCloud;
            pcl_conversions::toPCL(*this->sensorPointCloud2,pclCloud);
            return pclCloud;

        } case storedCloud::PCL_PC2: {
            // pcl::PCLPointCloud2 --> pcl::PCLPointCloud2
            return pcl::PCLPointCloud2(*this->pclPointCloud2);

        } case storedCloud::PCL_PXYZ: {
            // pcl::PointCloud<pcl::PointXYZ> --> pcl::PCLPointCloud2
            pcl::PCLPointCloud2 pclCloud;
            pcl::toPCLPointCloud2(*this->pclPointCloud, pclCloud);
            return pclCloud;

        } default: {
            // Unkown Identifier
            throw std::out_of_range("uqr::PointCloud type identifier out of range");
        }
    }
}

// Conversion to pcl::PointCloud<pcl::PointXYZ>
uqr::PointCloud::operator pcl::PointCloud<pcl::PointXYZ>() const {
    switch (this->currentType) {
        // None --> pcl::PointCloud<pcl::PointXYZ>
        case storedCloud::NO_CLOUD: {
            return pcl::PointCloud<pcl::PointXYZ>();

        } case storedCloud::SENSOR_MSGS: {
            // sensor_msgs::PointCloud2 --> pcl::PointCloud<pcl::PointXYZ>
            pcl::PointCloud<pcl::PointXYZ> pclCloud;
            pcl::fromROSMsg(*this->sensorPointCloud2, pclCloud);
            return pclCloud;

        } case storedCloud::PCL_PC2: {
            // pcl::PointCloud2 --> pcl::PointCloud<pcl::PointXYZ> 
            pcl::PointCloud<pcl::PointXYZ> pclCloud;
            pcl::fromPCLPointCloud2(*this->pclPointCloud2, pclCloud);
            return pclCloud;

        } case storedCloud::PCL_PXYZ: {
            // pcl::PointCloud<pcl::PointXYZ> --> pcl::PointCloud<pcl::PointXYZ>
            return pcl::PointCloud<pcl::PointXYZ>(*this->pclPointCloud);

        } default: {
            // Unkown Identifier
            throw std::out_of_range("uqr::PointCloud type identifier out of range");
        }
    }
}

void uqr::voxelise(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud, const float voxel_size) {

    // Define input and output clouds for filtering
    pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2((pcl::PCLPointCloud2) *inputCloud));
    pcl::PCLPointCloud2 cloud_filtered;

    // Configure Filter and Input
    pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(preFilter);

    // Configure Parameters
    filter.setLeafSize(voxel_size, voxel_size, voxel_size);

    // Filter
    // filter.filter((pcl::PCLPointCloud2&) outputCloud);  // FIXME: figure out this cast
    filter.filter(cloud_filtered);

    // Set output
    outputCloud = cloud_filtered;
}

void uqr::pass_through_filter(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud,
                              const std::string&  field_name, const double lower_limit,
                              const double upper_limit, const bool invert) {

    // Define input and output clouds for filtering
    pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2((pcl::PCLPointCloud2) *inputCloud));
    pcl::PCLPointCloud2 cloud_filtered;

    // Configure Filter and Input
    pcl::PassThrough<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(preFilter);

    // Configure Parameters
    filter.setFilterFieldName(field_name);
    filter.setFilterLimits(lower_limit, upper_limit);
    filter.setFilterLimitsNegative(invert);

    // Filter
    filter.filter(cloud_filtered);

    outputCloud = cloud_filtered;
}

void uqr::radius_outlier_removal(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud,
                                 const double radius, const double neighbours) {

    // Define input and output clouds for filtering
    pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2((pcl::PCLPointCloud2) *inputCloud));
    pcl::PCLPointCloud2 cloud_filtered;

    // Configure Filter and Input
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(preFilter);

    // Configure Parameters
    filter.setRadiusSearch(radius);
    filter.setMinNeighborsInRadius(neighbours);

    // Filter
    filter.filter(cloud_filtered);

    outputCloud = cloud_filtered;
}

void uqr::sac_segmentation(const uqr::PointCloud::ConstPtr& inputCloud, pcl::ModelCoefficients &outputCoeff, pcl::PointIndices &outputIndex,
                           const int model, const int method, const double threshold, const bool optimize_coeff) {

    // Define input clouds for filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr preSegment(new pcl::PointCloud<pcl::PointXYZ>((pcl::PointCloud<pcl::PointXYZ>) *inputCloud));

    // Configure Segmenter and Input
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

void uqr::subtract_indices(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud, const pcl::PointIndices::Ptr& subtraction, const bool invert) {

    // Define input and output clouds for filtering
    pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2((pcl::PCLPointCloud2) *inputCloud));
    pcl::PCLPointCloud2 cloud_filtered;

    // Configure Filter and Input
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(preFilter);
    extract.setIndices(subtraction);

    // Configure Parameters
    extract.setNegative(invert);

    // Filter
    extract.filter(cloud_filtered);

    outputCloud = cloud_filtered;
}


void uqr::conditional_filter(const uqr::PointCloud::ConstPtr &inputCloud, uqr::PointCloud &outputCloud,
                             pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition) {
    
    // Define input and output clouds for filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr preFilter(new pcl::PointCloud<pcl::PointXYZ>((pcl::PointCloud<pcl::PointXYZ>) *inputCloud));
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

    // Configure Segmenter and Input
    pcl::ConditionalRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(preFilter);

    // Configure Parameters
    filter.setCondition(condition);

    // Filter
    filter.filter(cloud_filtered);

    outputCloud = cloud_filtered;
}
