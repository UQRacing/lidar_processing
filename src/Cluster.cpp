/**
 * @author Riley Bowyer
 * @date 06-09-2020
 *
 * @brief Cluster source
 */


#include "lidar_cones_detection/Cluster.hpp"

uqr::Cluster::Cluster(){
    this->angleThresh = 0;
}

uqr::Cluster::Cluster(uint16_t rows, uint16_t cols, float angleThresh){
    this->angleThresh = angleThresh;
    this->labels = cv::Mat::zeros(rows,cols, cv::DataType<uint16_t>::type);

    int16_t counter = 0;
    for (int16_t r = this->ROWS; r > 0; --r) {
      this->Neighbourhood[counter++] = uqr::PointCord(-r, 0);
      this->Neighbourhood[counter++] = uqr::PointCord(r, 0);
    }
    for (int16_t c = this->COLS; c > 0; --c) {
      this->Neighbourhood[counter++] = uqr::PointCord(0, -c);
      this->Neighbourhood[counter++] = uqr::PointCord(0, c);
    }
}

void uqr::Cluster::set_depth(const cv::Mat& depth_image){
    this->depth_image_ptr = &depth_image;
}

void uqr::Cluster::set_angle(const cv::Mat& angle_image){
    this->angle_image_ptr = &angle_image;
}

void uqr::Cluster::clear_labels(){
    this->labels = cv::Mat::zeros(this->depth_image_ptr->size(), cv::DataType<uint16_t>::type);
}

void uqr::Cluster::label_search(uqr::PointCord start, uint16_t label){

    std::queue<uqr::PointCord> labeling_queue;
    labeling_queue.push(start);

    size_t max_queue_size = 0;
    while (!labeling_queue.empty()){
        max_queue_size = std::max(labeling_queue.size(), max_queue_size);

        // Copy Cord
        const PointCord current = labeling_queue.front();
        labeling_queue.pop();

        uint16_t current_label = this->get_label(current);

        if (current_label > 0){
            continue; // Already Labelled
        }

        // set the label of this point to current label
        this->set_label(current, label);

        // check the depth
        float current_depth = this->get_depth(current);
        if (current_depth < 0.001f) {
            continue; // Invalid Point
        }

        for (const auto& step : this->Neighbourhood){
            uqr::PointCord neighbour = current + step;
            if (neighbour.r < 0 || neighbour.r >= this->labels.rows){
                continue; // Point Out of Range
            }

            // Check Wrap-Around
            if(neighbour.c < 0){
                neighbour.c += this->labels.cols;
            }
            else if(neighbour.c >= this->labels.cols){
                neighbour.c -= this->labels.cols;
            }

            uint16_t neigh_label = this->get_label(neighbour);
            if (neigh_label > 0){
                continue;// Already Labelled
            }

            // Calc Diff
            float diff = abs(this->get_angle(current) - this->get_angle(neighbour));
            if (diff >= this->angleThresh) {
            labeling_queue.push(neighbour);
            }
        }
    }
}

void uqr::Cluster::set_label(uqr::PointCord point, uint16_t label){
    labels.at<uint16_t>(point.r, point.c) = label;
}

uint16_t uqr::Cluster::get_label(uqr::PointCord point){
    return labels.at<uint16_t>(point.r, point.c);
}

float uqr::Cluster::get_depth(uqr::PointCord point){
    return depth_image_ptr->at<uint16_t>(point.r, point.c);
}

float uqr::Cluster::get_angle(uqr::PointCord point){
    return angle_image_ptr->at<uint16_t>(point.r, point.c);
}

// TODO: Implement search to return a image mask for the given label
cv::Mat uqr::Cluster::label_mask(uint16_t label){ return cv::Mat::zeros(this->depth_image_ptr->size(), cv::DataType<uint16_t>::type);}

cv::Mat* uqr::Cluster::label_image(){
    return &this->labels;
}

