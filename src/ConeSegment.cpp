/**
 * @author Riley Bowyer
 * @date 06-09-2020
 *
 * @brief Cone Segment source
 */

#include "lidar_cones_detection/ConeSegment.hpp"

uqr::ConeSegmenter::ConeSegmenter(ProjectionParams rowParams, ProjectionParams colParams, float angleStep, int windowSize){
    this->rowParams = rowParams;
    this->colParams = colParams;
    this->labeler = uqr::Cluster(this->rowParams.len(),this->colParams.len(),this->angleStep*M_PI/180);
	this->col_angles = cv::Mat::zeros(this->rowParams.len(),this->colParams.len(), cv::DataType<float>::type);

	for(int r = 0; r < this->col_angles.rows; r++){
		for(int c = 0; c < this->col_angles.cols; c++){
			this->col_angles.at<float>(r,c) = this->colParams.from_index(c);
		}
	}
    this->windowSize = windowSize;
}

void uqr::ConeSegmenter::process_image(const cv::Mat& depth_image){

    this->labeler.set_depth(depth_image);
    this->labeler.set_angle(this->col_angles);

    this->segment_cones(depth_image);
}

void uqr::ConeSegmenter::segment_cones(const cv::Mat& depth_image){
	this->labels = 1;
	this->labeler.clear_labels();
    auto label_image_ptr = this->labeler.label_image();
    for (int row = 0; row < label_image_ptr->rows; ++row) {
		for (int col = 0; col < label_image_ptr->cols; ++col) {
			if (label_image_ptr->at<uint16_t>(row, col) > 0){
			continue;
			}
			if (depth_image.at<float>(row, col) < 0.001f){
			continue;
			}
			this->labeler.horizontal_search(uqr::PointCord(row, col), this->labels);
			this->labels++;
		}
    }
}

int uqr::ConeSegmenter::total_segments(){
	return this->labels;
}

cv::Mat uqr::ConeSegmenter::label_image(){
	return *labeler.label_image();
}

cv::Mat uqr::ConeSegmenter::get_cluster(const cv::Mat& depth_image, int id){
    auto label_image_ptr = labeler.label_image();
	cv::Mat masked = cv::Mat::zeros(depth_image.size(), CV_32F);

	this->windowSize = std::max(this->windowSize - 2, 3);
	cv::Mat kernel = this->depthUtil.uniform_kernal(this->windowSize, CV_8U);
	cv::Mat dilated = cv::Mat::zeros(label_image_ptr->size(), label_image_ptr->type());
	cv::dilate(*label_image_ptr, dilated, kernel);
	for (int r = 0; r < dilated.rows; ++r) {
		for (int c = 0; c < dilated.cols; ++c) {
			if (dilated.at<uint16_t>(r, c) == id) {
				masked.at<float>(r, c) = depth_image.at<float>(r, c);
			}
		}
	}
	return masked;
}