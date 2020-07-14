/**
 * @author Caleb Aitken
 * @date 14-07-2020
 *
 * @brief pcl wrapper library sources
 */

#include "lidar_cones_detection/PCLWrapper.hpp"

uqr::PointCloud::PointCloud(const sensor_msgs::PointCloud2& otherPointCloud) {
    this->sensorPointCloud2 = otherPointCloud;
    this->sensorPointCloud2IsInitialised = true;
}

uqr::PointCloud::PointCloud(const pcl::PCLPointCloud2& otherPointCloud) {
    this->pclPointCloud2 = otherPointCloud;
    this->pclPointCloud2IsInitialised = true;
}

uqr::PointCloud::PointCloud(const pcl::PointCloud<pcl::PointXYZ>& otherPointCloud) {
    this->pclPointCloud = otherPointCloud;
    this->pclPointCloudIsInitialised = true;
}

uqr::PointCloud::operator sensor_msgs::PointCloud2() {
    // TODO: check if type initialised, if not create it
    return sensor_msgs::PointCloud2(this->sensorPointCloud2);
}

uqr::PointCloud::operator pcl::PCLPointCloud2() {
    // TODO: check if type initialised, if not create it
    return pcl::PCLPointCloud2(this->pclPointCloud2);
}

uqr::PointCloud::operator pcl::PointCloud<pcl::PointXYZ>() {
    // TODO: check if type initialised, if not create it
    return pcl::PointCloud<pcl::PointXYZ>(this->pclPointCloud);
}

// TODO
void uqr::PointCloud::convert_to_pclPointCloud2() {
    if (this->sensorPointCloud2IsInitialised) {

    }

    this->pclPointCloud2IsInitialised = true;
}

// TODO
void uqr::PointCloud::convert_to_sensorPointCLoud2() {

}

//TODO
void uqr::PointCloud::convert_to_pclPointCloud() {

}
