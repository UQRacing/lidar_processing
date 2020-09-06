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

void uqr::view_cloud(uqr::PointCloud& input_cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = (pcl::PointCloud<pcl::PointXYZ>) input_cloud;
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){}
}

uqr::cloudPublisher::cloudPublisher(std::string topic){
    // Initialise Publisher
    this->cloudPub = this->nh.advertise<sensor_msgs::PointCloud2>(topic, 10);
}

void uqr::cloudPublisher::publish(uqr::PointCloud& cloud, std::string frame, ros::Time timeStamp){
    sensor_msgs::PointCloud2 outputCloud((sensor_msgs::PointCloud2) cloud);

    // Set Header Information
    outputCloud.header.frame_id = frame;
    outputCloud.header.stamp = timeStamp;

    // Publish
    this->cloudPub.publish(outputCloud);
}

void uqr::cloudPublisher::publish(sensor_msgs::PointCloud2& cloud, std::string frame, ros::Time timeStamp){
    // Set Header Information
    cloud.header.frame_id = frame;
    cloud.header.stamp = timeStamp;

    // Publish
    this->cloudPub.publish(cloud);
}

void uqr::cloudPublisher::publish(pcl::PCLPointCloud2& cloud, std::string frame, ros::Time timeStamp){
    // Abuse the fact we've defined conversions in uqr::PointCloud
    uqr::PointCloud uqrCloud(cloud);
    sensor_msgs::PointCloud2 outputCloud((sensor_msgs::PointCloud2) uqrCloud);

    // Set Header Information
    outputCloud.header.frame_id = frame;
    outputCloud.header.stamp = timeStamp;

    // Publish
    this->cloudPub.publish(outputCloud);
}

void uqr::cloudPublisher::publish(pcl::PointCloud<pcl::PointXYZ>& cloud, std::string frame, ros::Time timeStamp){
    // Abuse the fact we've defined conversions in uqr::PointCloud
    uqr::PointCloud uqrCloud(cloud);
    sensor_msgs::PointCloud2 outputCloud((sensor_msgs::PointCloud2) uqrCloud);

    // Set Header Information
    outputCloud.header.frame_id = frame;
    outputCloud.header.stamp = timeStamp;

    // Publish
    this->cloudPub.publish(outputCloud);
}

uqr::ImagePublisher::ImagePublisher(std::string topic){
    // Initialise Publisher
    this->imgPub = this->nh.advertise<sensor_msgs::Image>(topic, 10);
}

void uqr::ImagePublisher::publish(const cv::Mat& image, float scale, std::string frame, ros::Time timeStamp){
    cv::Mat out_img;
    image.convertTo(out_img, 0, scale);
    cv_bridge::CvImage out_msg;

    // Set Header Information
    out_msg.header.frame_id = frame;
    out_msg.header.stamp = timeStamp;
	out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = out_img;

    // Publish
    this->imgPub.publish(out_msg);
}

void uqr::ImagePublisher::colour_publish(const cv::Mat& label_image, std::string frame, ros::Time timeStamp){
    cv::Mat color_image(label_image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	for (int row = 0; row < label_image.rows; ++row) {
		for (int col = 0; col < label_image.cols; ++col) {
			auto label = label_image.at<uint16_t>(row, col);
			auto random_color = uqr::RANDOM_COLORS[label % uqr::RANDOM_COLORS.size()];
			cv::Vec3b color = cv::Vec3b(random_color[0], random_color[1], random_color[2]);
			color_image.at<cv::Vec3b>(row, col) = color;
		}
	}
    cv_bridge::CvImage out_msg;

    // Set Header Information
    out_msg.header.frame_id = frame;
    out_msg.header.stamp = timeStamp;
	out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    out_msg.image = color_image;

    // Publish
    this->imgPub.publish(out_msg);
}