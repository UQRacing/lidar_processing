/**
 * @author Caleb Aitken
 * @date 14-07-2020
 *
 * @brief pcl wrapper library sources
 */

#include "lidar_cones_detection/PCLWrapper.hpp"

// TODO: GOD DAMN BRUH this code is wack. WHAT DOES IT MEAn?
//      maybe?,.., explain it with comments????

// TODO: haha i haven't tested any of this yet :)
//      at the least, it runs without crashing

uqr::PointCloud::PointCloud() {
    this->pclPointCloud2.reset(new pcl::PCLPointCloud2);
    this->pclPointCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->sensorPointCloud2.reset(new sensor_msgs::PointCloud2);
}

uqr::PointCloud::PointCloud(const sensor_msgs::PointCloud2& otherPointCloud) {
    this->pclPointCloud.reset();
    this->pclPointCloud2.reset();
    this->sensorPointCloud2 = boost::make_shared<sensor_msgs::PointCloud2>(sensor_msgs::PointCloud2(otherPointCloud));
    this->type = 1;
}

uqr::PointCloud::PointCloud(const pcl::PCLPointCloud2& otherPointCloud) {
    this->pclPointCloud.reset();
    this->sensorPointCloud2.reset();
    this->pclPointCloud2 = boost::make_shared<pcl::PCLPointCloud2>(pcl::PCLPointCloud2(otherPointCloud));
    this->type = 2;
}

uqr::PointCloud::PointCloud(const pcl::PointCloud<pcl::PointXYZ>& otherPointCloud) {
    this->sensorPointCloud2.reset();
    this->pclPointCloud2.reset();
    this->pclPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>(otherPointCloud));
    this->type = 3;
}

// if you, the reader, are wondering "why did this man put braces on a switch case statement"
// I dare you to remove them and try compiling this code
uqr::PointCloud::operator sensor_msgs::PointCloud2() const {
    switch (this->type) {
        case 0: {
            return sensor_msgs::PointCloud2();
        } case 1: {
            // from sensor_msgs::PointCloud2
            return sensor_msgs::PointCloud2(*this->sensorPointCloud2);
        } case 2: {
            // from pcl::PCLPointCloud2
            sensor_msgs::PointCloud2 rosCloud;
            pcl::PointCloud<pcl::PointXYZ> betweenConversion;
            pcl::fromPCLPointCloud2(*this->pclPointCloud2, betweenConversion);
            pcl::toROSMsg(betweenConversion, rosCloud);
            return rosCloud;
        } case 3: {
            // from pcl::PointCloud<pcl::PointXYZ>
            sensor_msgs::PointCloud2 rosCloud;
            pcl::toROSMsg(*this->pclPointCloud, rosCloud);
            return rosCloud;
        } default: {
            throw std::out_of_range("uqr::PointCloud type identifier out of range");
        }
    }
}

// if you, the reader, are wondering "why did this man put braces on a switch case statement"
// I dare you to remove them and try compiling this code
uqr::PointCloud::operator pcl::PCLPointCloud2() const {
    switch (this->type) {
        case 0: {
            return pcl::PCLPointCloud2();
        } case 1: {
            // from sensor_msgs::PointCloud2
            // TODO: why tf doesn't this convert straight to the other. they have the same name??
            pcl::PCLPointCloud2 pclCloud;
            pcl::PointCloud<pcl::PointXYZ> betweenConversion;
            pcl::fromROSMsg(*this->sensorPointCloud2, betweenConversion);
            pcl::toPCLPointCloud2(betweenConversion, pclCloud);
            return pclCloud;
        } case 2: {
            // from pcl::PCLPointCloud2
            return pcl::PCLPointCloud2(*this->pclPointCloud2);
        } case 3: {
            // from pcl::PointCloud<pcl::PointXYZ>
            pcl::PCLPointCloud2 pclCloud;
            pcl::toPCLPointCloud2(*this->pclPointCloud, pclCloud);
            return pclCloud;
        } default: {
            throw std::out_of_range("uqr::PointCloud type identifier out of range");
        }
    }
}

// if you, the reader, are wondering "why did this man put braces on a switch case statement"
// I dare you to remove them and try compiling this code
uqr::PointCloud::operator pcl::PointCloud<pcl::PointXYZ>() const {
    switch (this->type) {
        case 0: {
            return pcl::PointCloud<pcl::PointXYZ>();
        } case 1: {
            // from sensor_msgs::PointCloud2
            pcl::PointCloud<pcl::PointXYZ> pclCloud;
            pcl::fromROSMsg(*this->sensorPointCloud2, pclCloud);
            return pclCloud;
        } case 2: {
            // from pcl::PCLPointCloud2
            pcl::PointCloud<pcl::PointXYZ> pclCloud;
            pcl::fromPCLPointCloud2(*this->pclPointCloud2, pclCloud);
            return pclCloud;
        } case 3: {
            // from pcl::PointCloud<pcl::PointXYZ>
            return pcl::PointCloud<pcl::PointXYZ>(*this->pclPointCloud);
        } default: {
            throw std::out_of_range("uqr::PointCloud type identifier out of range");
        }
    }
}

void uqr::voxelise(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud, const float voxel_size) {
    // Convert to PCLPointCloud2 Format
    // pcl::PCLPointCloud2 pc2InputCloud(inputCloud);

    // Define input and output clouds for filtering
    pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2((pcl::PCLPointCloud2) *inputCloud));
    pcl::PCLPointCloud2 cloud_filtered;

    // Copy across data
    // pcl::copyPointCloud(pc2InputCloud, *preFilter);

    // Define Input
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
                              const double upper_limit, const bool invert_filter) {
    // Convert to PCLPointCloud2 Format
    //pcl::PCLPointCloud2 pc2InputCloud(inputCloud);

    // Define input and output clouds for filtering
    pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2((pcl::PCLPointCloud2) *inputCloud));
    pcl::PCLPointCloud2 cloud_filtered;

    // Copy across data
    //pcl::copyPointCloud(pc2InputCloud, *preFilter);

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

void uqr::radius_outlier_removal(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud,
                                 const double radius, const double neighbours) {
    // Convert to PCLPointCloud2 Format
    //pcl::PCLPointCloud2 pc2InputCloud(inputCloud);

    // Define input and output clouds for filtering
    pcl::PCLPointCloud2::Ptr preFilter(new pcl::PCLPointCloud2((pcl::PCLPointCloud2) *inputCloud));
    pcl::PCLPointCloud2 cloud_filtered;

    // Copy across data
    //pcl::copyPointCloud(pc2InputCloud, *preFilter);

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

void uqr::sac_segmentation(const uqr::PointCloud::ConstPtr& inputCloud, pcl::ModelCoefficients &outputCoeff, pcl::PointIndices &outputIndex,
                           const int model, const int method, const double threshold, const bool optimize_coeff) {
    // Method Types can be found at pcl/sample_consensus/method_types.h
    // Model Types can be found at pcl/sample_consensus/model_types.h

    // Convert to PCLPointCloud2 Format
    //pcl::PointCloud<pcl::PointXYZ> pclInputCloud(inputCloud);

    // Define input clouds for filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr preSegment(new pcl::PointCloud<pcl::PointXYZ>((pcl::PointCloud<pcl::PointXYZ>) *inputCloud));

    // Copy across data
    //pcl::copyPointCloud(pclInputCloud, *preSegment);

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

void uqr::subtract_indices(const uqr::PointCloud::ConstPtr& inputCloud, uqr::PointCloud& outputCloud, const pcl::PointIndices::Ptr& subtraction) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr preSubtraction(new pcl::PointCloud<pcl::PointXYZ>((pcl::PointCloud<pcl::PointXYZ>) *inputCloud));
    pcl::PointCloud<pcl::PointXYZ> cloudSubtracted;
    
    extract.setInputCloud(preSubtraction);
    extract.setIndices(subtraction);
    extract.setNegative(true);
    extract.filter(cloudSubtracted);
    outputCloud = cloudSubtracted;
}


void uqr::conditional_filter(const uqr::PointCloud::ConstPtr &inputCloud, uqr::PointCloud &outputCloud,
                             pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition) {
    // Sample condition:
    // pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));

    // Convert to PCL::PointCloud Format
    //pcl::PointCloud<pcl::PointXYZ> pclInputCloud(inputCloud);

    // Define input and output clouds for filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr preFilter(new pcl::PointCloud<pcl::PointXYZ>((pcl::PointCloud<pcl::PointXYZ>) *inputCloud));
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

    // Copy across data
    //pcl::copyPointCloud(pclInputCloud, *preFilter);

    // Define Input
    pcl::ConditionalRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(preFilter);

    // Configure Parameters
    filter.setCondition(condition);

    // Filter
    filter.filter(cloud_filtered);

    outputCloud = cloud_filtered;
}
