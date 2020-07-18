/**
 * @author Riley Bowyer
 * @date 18-07-2020
 *
 * @brief IO Utility function source
 */

#include "lidar_cones_detection/IOUtility.hpp"

void uqr::load_cloud(const std::string& file_path, uqr::PointCloud& outputCloud){

    // Retrieve File Extension, could be either pcd or ply.
    std::string extension;
    if(file_path.find_last_of(".") != std::string::npos){
        extension =  file_path.substr(file_path.find_last_of(".")+1);
    }

    // Load Cloud into pcl::PointCloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(extension == "pcd" || extension == "PCD"){
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) // Load File
        {
            std::cout << "Couldn't read file at " << file_path << "." << std::endl;
            return;
        }
    }
    else if(extension == "ply" || extension == "PLY"){
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path, *cloud) == -1) // Load File
        {
            std::cout << "Couldn't read file at " << file_path << "." << std::endl;
            return;
        }
    }
    else{
        std::cout << "Couldn't read file at " << file_path << ", File Type Not Supported." << std::endl;
    }
    
    // Save to uqr::PointCloud
    outputCloud = *cloud;
}

void uqr::write_cloud(const std::string file_path, uqr::PointCloud& input_cloud){

    // Retrieve File Extension, could be either pcd or ply.
    std::string extension;

    if(file_path.find_last_of(".") != std::string::npos){
        extension =  file_path.substr(file_path.find_last_of(".")+1);
    }

    // Call the appropriate write function.
    if(extension == "pcd" || extension == "PCD"){
        pcl::io::savePCDFileBinary(file_path, (pcl::PointCloud<pcl::PointXYZ>) input_cloud); // Write File
    }
    else if(extension == "ply" || extension == "PLY"){
        pcl::io::savePLYFileBinary(file_path, (pcl::PointCloud<pcl::PointXYZ>) input_cloud); // Write File
    }
    else{
        std::cout << "Couldn't write file at " << file_path << ", File Type Not Supported." << std::endl;
    }
}

uqr::cloudPublisher::cloudPublisher(std::string topic){
    // Initialise Publisher
    this->cloudPub = this->nh.advertise<sensor_msgs::PointCloud2>(topic, 10);
    this->frame = "map"; // Default Frame

}

uqr::cloudPublisher::cloudPublisher(std::string topic, std::string frame){
    // Initialise Publisher
    this->cloudPub = this->nh.advertise<sensor_msgs::PointCloud2>(topic, 10);
    this->frame = frame;
}

void uqr::cloudPublisher::publish(uqr::PointCloud& cloud){
    sensor_msgs::PointCloud2 outputCloud((sensor_msgs::PointCloud2) cloud);

    // Set Header Information
    outputCloud.header.frame_id = this->frame;
    outputCloud.header.stamp = ros::Time::now();

    // Publish
    this->cloudPub.publish(outputCloud);

}

void uqr::cloudPublisher::publish(sensor_msgs::PointCloud2& cloud){
    // Set Header Information
    cloud.header.frame_id = this->frame;
    cloud.header.stamp = ros::Time::now();

    // Publish
    this->cloudPub.publish(cloud);

    
}

void uqr::cloudPublisher::publish(pcl::PCLPointCloud2& cloud){
    // Abuse the fact we've defined conversions in uqr::PointCloud
    uqr::PointCloud uqrCloud(cloud);
    sensor_msgs::PointCloud2 outputCloud((sensor_msgs::PointCloud2) uqrCloud);

    // Set Header Information
    outputCloud.header.frame_id = this->frame;
    outputCloud.header.stamp = ros::Time::now();

    // Publish
    this->cloudPub.publish(outputCloud);
}

void uqr::cloudPublisher::publish(pcl::PointCloud<pcl::PointXYZ>& cloud){
    // Abuse the fact we've defined conversions in uqr::PointCloud
    uqr::PointCloud uqrCloud(cloud);
    sensor_msgs::PointCloud2 outputCloud((sensor_msgs::PointCloud2) uqrCloud);

    // Set Header Information
    outputCloud.header.frame_id = this->frame;
    outputCloud.header.stamp = ros::Time::now();

    // Publish
    this->cloudPub.publish(outputCloud);
}