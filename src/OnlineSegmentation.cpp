/**
 * @author Riley Bowyer
 * @date 23-07-2020
 *
 * @brief Online Cloud Segmenter source
 */

#include "lidar_cones_detection/OnlineSegmentation.hpp"

uqr::ProjectionParams::ProjectionParams(){
    this->num_beams = 0;
    this->step = 0;
    this->min_angle = 0;
    this->max_angle = 0;
}

uqr::ProjectionParams::ProjectionParams(float min_angle, float max_angle, float step){
    this->num_beams = floor((max_angle - min_angle) / step);
    this->step = step;
    this->min_angle = min_angle;
    this->max_angle = max_angle;
}

uqr::ProjectionParams::ProjectionParams(float min_angle, float max_angle, int num_beams){
    this->num_beams = num_beams;
    this->step = step = (max_angle - min_angle) / num_beams;
    this->min_angle = min_angle;
    this->max_angle = max_angle;
}

void uqr::ProjectionParams::fill_angles(){
    float angle = this->min_angle;
    for(int i=0; i < this->num_beams; i++){
        angles.push_back(angle);
        angle += this->step;
    }
}

int uqr::ProjectionParams::find_closest(float angle){
    size_t found = 0;
    if (this->angles.front() < this->angles.back()) {
        found = std::upper_bound(this->angles.begin(), this->angles.end(), angle) - this->angles.begin();
    } else {
        found = this->angles.rend() - std::upper_bound(this->angles.rbegin(), this->angles.rend(), angle);
    }
    if (found == 0) {
        return found;
    }
    if (found == this->angles.size()) {
        return found - 1;
    }
    auto diff_next = fabs(this->angles[found] - angle);
    auto diff_prev = fabs(angle - this->angles[found - 1]);
    return diff_next < diff_prev ? found : found - 1;
}

float uqr::ProjectionParams::from_index(int index){
    return this->angles[index];
}

int uqr::ProjectionParams::from_angle(float angle){
    return find_closest(angle);
}

void uqr::ProjectionParams::show_angles(){
    for (auto i = this->angles.begin(); i != this->angles.end(); ++i){
        std::cout << *i << ' ';
    }
    std::cout << std::endl;
}

int uqr::ProjectionParams::len(){
    return this->num_beams;
}

uqr::Projector::Projector(){
}


uqr::Projector::Projector(uqr::ProjectionParams rowParams, uqr::ProjectionParams colParams){
    this->rowParams = rowParams;
    this->colParams = colParams;

    this->rowParams.fill_angles();
    this->colParams.fill_angles();
    this->depthImage = cv::Mat::zeros(this->rowParams.len(), this->colParams.len(), cv::DataType<float>::type);
}

void uqr::Projector::convert(pcl::PointCloud<pcl::PointXYZ> inputCloud){
    this->max_depth = 0.0;
    for(int index=0;index<inputCloud.size();index++){
        pcl::PointXYZ point = inputCloud[index];
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

cv::Mat uqr::Projector::get_depth(){
    return this->depthImage;
}

float uqr::Projector::get_max_depth(){
    return this->max_depth;
}