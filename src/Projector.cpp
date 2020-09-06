/**
 * @author Riley Bowyer
 * @date 24-08-2020
 *
 * @brief Projector source
 */

#include "lidar_cones_detection/Projector.hpp"

uqr::Projector::Projector(){
    /// Init empty instance
    this->rowParams = ProjectionParams();
    this->colParams = ProjectionParams();

    this->depthImage = cv::Mat::zeros(this->rowParams.len(), this->colParams.len(), cv::DataType<float>::type);
}


uqr::Projector::Projector(uqr::ProjectionParams rowParams, uqr::ProjectionParams colParams){
    this->rowParams = rowParams;
    this->colParams = colParams;

    this->depthImage = cv::Mat::zeros(this->rowParams.len(), this->colParams.len(), cv::DataType<float>::type);
}

void uqr::Projector::convert(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud){
    this->max_depth = 0.0;
    this->depthImage = cv::Mat::zeros(this->rowParams.len(), this->colParams.len(), cv::DataType<float>::type);
    
    for(int index=0;index<inputCloud->size();index++){
        pcl::PointXYZ point = inputCloud->points[index];
        float depth = pow(pow(point.x,2)+pow(point.y,2)+pow(point.z,2),0.5);
        if(depth < 0.01f){ // Check Min Threshold
            continue;
        }

        // Get Pixel
        int row = this->rowParams.from_angle(asin(point.z / depth));
        int col = this->colParams.from_angle(atan2(point.y,point.x));
        float current_depth = this->depthImage.at<float>(row, col);
        if (depth > current_depth) {
            this->depthImage.at<float>(row, col) = depth;
            if(depth > max_depth){
                this->max_depth = depth;
            }
        }
    }
}

cv::Mat* uqr::Projector::get_depth(){
    return &this->depthImage;
}

float uqr::Projector::get_max_depth(){
    return this->max_depth;
}