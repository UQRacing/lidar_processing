/**
 * @author Riley Bowyer
 * @date 31-08-2020
 *
 * @brief Ground Removal source
 */

#include "lidar_cones_detection/GroundRemoval.hpp"

uqr::GroundRemover::GroundRemover(ProjectionParams rowParams, ProjectionParams colParams, float angleStep, int windowSize){
    this->rowParams = rowParams;
    this->colParams = colParams;
    this->angleStep = angleStep;

    this->labeler = uqr::Cluster(rowParams.len(),colParams.len(),angleStep*M_PI/180);

    this->windowSize = windowSize;
    this->maxAngle = -1;
}

void uqr::GroundRemover::process_image(const cv::Mat& depth_image){
    const cv::Mat repaired_depth = this->depthUtil.repair_depth(depth_image, 7, 1.0f);
    const cv::Mat angle_image = this->angle_image(repaired_depth);
    const cv::Mat smoothed_image = this->depthUtil.smooth_image(angle_image, this->windowSize);

    this->labeler.set_depth(depth_image);
    this->labeler.set_angle(smoothed_image);

    this->remove_ground(depth_image, smoothed_image);
}

cv::Mat uqr::GroundRemover::angle_image(const cv::Mat& depth_image) {
    cv::Mat angle_image = cv::Mat::zeros(depth_image.size(), cv::DataType<float>::type);
    cv::Mat x_mat = cv::Mat::zeros(depth_image.size(), cv::DataType<float>::type);
    cv::Mat y_mat = cv::Mat::zeros(depth_image.size(), cv::DataType<float>::type);

    std::vector<float> sines = rowParams.sines_vector();
    std::vector<float> cosines = rowParams.cosines_vector();

    float dx, dy;
    x_mat.row(0) = depth_image.row(0) * cosines[0];
    y_mat.row(0) = depth_image.row(0) * sines[0];

    for (int r = 1; r < angle_image.rows; ++r) {
        x_mat.row(r) = depth_image.row(r) * cosines[r];
        y_mat.row(r) = depth_image.row(r) * sines[r];
        for (int c = 0; c < angle_image.cols; ++c) {
            dx = fabs(x_mat.at<float>(r, c) - x_mat.at<float>(r - 1, c));
            dy = fabs(y_mat.at<float>(r, c) - y_mat.at<float>(r - 1, c));
            float angle = atan2(dy, dx);
            if(angle > this-> maxAngle){
                this->maxAngle = angle;
            }
            angle_image.at<float>(r, c) = angle;
        }
    }
    return angle_image;
}

void uqr::GroundRemover::remove_ground(const cv::Mat& depth_image, const cv::Mat& angle_image){

	this->groundless = cv::Mat::zeros(depth_image.size(), CV_32F);
	this->labeler.clear_labels();

	for (int c = 0; c < depth_image.cols; ++c) {
		// start at bottom pixels and do bfs
		int r = 0;
		while (r < depth_image.rows && depth_image.at<float>(r, c) < 0.001f) {
		++r;
		}
		uqr::PointCord current_coord = PointCord(r, c);
		uint16_t current_label = this->labeler.get_label(current_coord);
		if (current_label > 0) {
		// this coord was already labeled, skip
		continue;
		}

		this->labeler.vertical_search(current_coord, 1);
	}

	auto label_image_ptr = labeler.label_image();

	cv::Mat kernel = this->depthUtil.uniform_kernal(std::max(this->windowSize - 2, 3), CV_8U);
	cv::Mat dilated = cv::Mat::zeros(label_image_ptr->size(), label_image_ptr->type());
	cv::dilate(*label_image_ptr, dilated, kernel);
	for (int r = 0; r < dilated.rows; ++r) {
		for (int c = 0; c < dilated.cols; ++c) {
			if (dilated.at<uint16_t>(r, c) == 0) {
				// all unlabeled points are non-ground
				this->groundless.at<float>(r, c) = depth_image.at<float>(r, c);
			}
		}
	}
}

float uqr::GroundRemover::get_max_angle(){
    return this->maxAngle;
}

cv::Mat* uqr::GroundRemover::get_groundless(){
    return &this->groundless;
}

cv::Mat uqr::GroundRemover::label_image(){
	return *labeler.label_image();
}