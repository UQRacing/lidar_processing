/**
 * @author Caleb Aitken
 * @brief this file is for testing purposes
 */


#include "lidar_cones_detection/PCLWrapper.hpp"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"

// Caleb Test Main Loop
// int main(int argc, char** argv) {
//     pcl::PCLPointCloud2::Ptr origin(new pcl::PCLPointCloud2());
//     std::cout << "pcl::PCLPointCloud2::Ptr points to: " << origin << std::endl;
//     uqr::PointCloud pointCloud(*origin);
//     std::cout << "uqr::PointCloud points to: " << &pointCloud << std::endl;
//     pcl::PCLPointCloud2 pclPointCloud = pointCloud;
//     sensor_msgs::PointCloud2 sensorPointCloud = pointCloud;
//     std::cout << "uqr::PointCloud points to: " << &pointCloud << std::endl;
//     std::cout << "pcl::PCLPointCloud2 points to: " << &pclPointCloud << std::endl;
//     std::cout << "sensor_msgs::PointCloud2 points to: " << &sensorPointCloud << std::endl;
// }

// Riley Test Main Loop
int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
                << cloud->width * cloud->height
                << " data points from test_pcd.pcd with the following fields: "
                << std::endl;

    uqr::PointCloud pointCloud(*cloud);
    uqr::PointCloud pointCloudOut;
    uqr::voxelise(pointCloud, pointCloudOut, 0.5);
    
    sensor_msgs::PointCloud2 inputCloud = pointCloud;
    sensor_msgs::PointCloud2 outputCloud = pointCloudOut;

    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher inputPub = nh.advertise<sensor_msgs::PointCloud2> ("input_cloud", 1);
    ros::Publisher outputPub = nh.advertise<sensor_msgs::PointCloud2> ("output_cloud", 1);

    inputCloud.header.frame_id = "map";
    inputCloud.header.stamp = ros::Time::now();
    outputCloud.header.frame_id = "map";
    outputCloud.header.stamp = ros::Time::now();

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        inputCloud.header.frame_id = "map";
        inputCloud.header.stamp = ros::Time::now();
        outputCloud.header.frame_id = "map";
        outputCloud.header.stamp = ros::Time::now();
        inputPub.publish(inputCloud);
        outputPub.publish(outputCloud);
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}