/// this file is for testing purposes

#include "lidar_cones_detection/PCLWrapper.hpp"

int main(int argc, char** argv) {
    pcl::PCLPointCloud2::Ptr origin(new pcl::PCLPointCloud2());
    std::cout << "pcl::PCLPointCloud2::Ptr points to: " << origin << std::endl;
    uqr::PointCloud pointCloud(*origin);
    std::cout << "uqr::PointCloud points to: " << &pointCloud << std::endl;
    pcl::PCLPointCloud2 pclPointCloud = pointCloud;
    std::cout << "uqr::PointCloud points to: " << &pointCloud << std::endl;
    std::cout << "pcl::PCLPointCloud2 points to: " << &pclPointCloud << std::endl;
}