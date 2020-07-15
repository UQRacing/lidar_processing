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

uqr::PointCloud::PointCloud(const sensor_msgs::PointCloud2& otherPointCloud) {
    this->sensorPointCloud2 = boost::make_shared<sensor_msgs::PointCloud2>(sensor_msgs::PointCloud2(otherPointCloud));
    this->type = 1;
}

uqr::PointCloud::PointCloud(const pcl::PCLPointCloud2& otherPointCloud) {
    this->pclPointCloud2 = boost::make_shared<pcl::PCLPointCloud2>(pcl::PCLPointCloud2(otherPointCloud));
    this->type = 2;
}

uqr::PointCloud::PointCloud(const pcl::PointCloud<pcl::PointXYZ>& otherPointCloud) {
    this->pclPointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>(otherPointCloud));
    this->type = 3;
}

// if you, the reader, are wondering "why did this man put braces on a switch case statement"
// I dare you to remove them and try compiling this code
uqr::PointCloud::operator sensor_msgs::PointCloud2() {
    switch (this->type) {
        case 0: {
            std::cerr << "uqr::PointCloud uninitialised" << std::endl;
            throw std::bad_cast();
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
uqr::PointCloud::operator pcl::PCLPointCloud2() {
    switch (this->type) {
        case 0: {
            std::cerr << "uqr::PointCloud uninitialised" << std::endl;
            throw std::bad_cast();
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
uqr::PointCloud::operator pcl::PointCloud<pcl::PointXYZ>() {
    switch (this->type) {
        case 0: {
            std::cerr << "uqr::PointCloud uninitialised" << std::endl;
            throw std::bad_cast();
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
