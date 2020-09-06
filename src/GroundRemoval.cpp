/**
 * @author Riley Bowyer
 * @date 31-08-2020
 *
 * @brief Projection Parameter source
 */

#include "lidar_cones_detection/GroundRemoval.hpp"

uqr::GroundRemover::GroundRemover(ProjectionParams rowParams, ProjectionParams colParams, float angleStep, int windowSize){
    this->rowParams = rowParams;
    this->colParams = colParams;
    this->labeler = uqr::Cluster(rowParams.len(),colParams.len(),angleStep);

    this->angleStep = angleStep*2*M_PI/180;
    this->windowSize = windowSize;

    this->maxAngle = -1;
}

void uqr::GroundRemover::process_image(const cv::Mat& depth_image){

    const cv::Mat repaired_depth = this->repair_depth(depth_image, 7, 1.0f);
    const cv::Mat angle_image = this->angle_image(repaired_depth);
    const cv::Mat smoothed_image = this->smooth_image(angle_image, this->windowSize);

    this->labeler.set_depth(depth_image);
    this->labeler.set_angle(smoothed_image);

    groundless = this->remove_ground(depth_image, smoothed_image, this->angleStep, this->windowSize);
}

cv::Mat uqr::GroundRemover::repair_depth(const cv::Mat& depth_image, int step,
                                    float depth_threshold) {
  cv::Mat inpainted_depth = depth_image.clone();
  for (int c = 0; c < inpainted_depth.cols; ++c) {
    for (int r = 0; r < inpainted_depth.rows; ++r) {
      float& curr_depth = inpainted_depth.at<float>(r, c);
      if (curr_depth < 0.001f) {
        int counter = 0;
        float sum = 0.0f;
        for (int i = 1; i < step; ++i) {
          if (r - i < 0) {
            continue;
          }
          for (int j = 1; j < step; ++j) {
            if (r + j > inpainted_depth.rows - 1) {
              continue;
            }
            const float& prev = inpainted_depth.at<float>(r - i, c);
            const float& next = inpainted_depth.at<float>(r + j, c);
            if (prev > 0.001f && next > 0.001f &&
                fabs(prev - next) < depth_threshold) {
              sum += prev + next;
              counter += 2;
            }
          }
        }
        if (counter > 0) {
          curr_depth = sum / counter;
        }
      }
    }
  }
  return inpainted_depth;
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


cv::Mat uqr::GroundRemover::get_kernel(int window_size){
  if (window_size % 2 == 0) {
    throw std::logic_error("only odd window size allowed");
  }
  bool window_size_ok = window_size == 5 || window_size == 7 ||
                        window_size == 9 || window_size == 11;
  if (!window_size_ok) {
    throw std::logic_error("bad window size");
  }
  // below are no magic constants. See Savitsky-golay filter.
  cv::Mat kernel;
  switch (window_size) {
    case 5:
      kernel = cv::Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -3.0f;
      kernel.at<float>(0, 1) = 12.0f;
      kernel.at<float>(0, 2) = 17.0f;
      kernel.at<float>(0, 3) = 12.0f;
      kernel.at<float>(0, 4) = -3.0f;
      kernel /= 35.0f;
      return kernel;
    case 7:
      kernel = cv::Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -2.0f;
      kernel.at<float>(0, 1) = 3.0f;
      kernel.at<float>(0, 2) = 6.0f;
      kernel.at<float>(0, 3) = 7.0f;
      kernel.at<float>(0, 4) = 6.0f;
      kernel.at<float>(0, 5) = 3.0f;
      kernel.at<float>(0, 6) = -2.0f;
      kernel /= 21.0f;
      return kernel;
    case 9:
      kernel = cv::Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -21.0f;
      kernel.at<float>(0, 1) = 14.0f;
      kernel.at<float>(0, 2) = 39.0f;
      kernel.at<float>(0, 3) = 54.0f;
      kernel.at<float>(0, 4) = 59.0f;
      kernel.at<float>(0, 5) = 54.0f;
      kernel.at<float>(0, 6) = 39.0f;
      kernel.at<float>(0, 7) = 14.0f;
      kernel.at<float>(0, 8) = -21.0f;
      kernel /= 231.0f;
      return kernel;
    case 11:
      kernel = cv::Mat::zeros(window_size, 1, CV_32F);
      kernel.at<float>(0, 0) = -36.0f;
      kernel.at<float>(0, 1) = 9.0f;
      kernel.at<float>(0, 2) = 44.0f;
      kernel.at<float>(0, 3) = 69.0f;
      kernel.at<float>(0, 4) = 84.0f;
      kernel.at<float>(0, 5) = 89.0f;
      kernel.at<float>(0, 6) = 84.0f;
      kernel.at<float>(0, 7) = 69.0f;
      kernel.at<float>(0, 8) = 44.0f;
      kernel.at<float>(0, 9) = 9.0f;
      kernel.at<float>(0, 10) = -36.0f;
      kernel /= 429.0f;
      return kernel;
  }
  return kernel;
}

cv::Mat uqr::GroundRemover::smooth_image(const cv::Mat& image, int window_size) {
  cv::Mat kernel = this->get_kernel(window_size);

  cv::Mat smoothed_image;  // init an empty smoothed image
  cv::filter2D(image, smoothed_image, -1, kernel, cv::Point(-1, -1),
               0, cv::BORDER_REFLECT101);
  return smoothed_image;
}

cv::Mat uqr::GroundRemover::uniform_kernal(int window_size, int type){
    if (window_size % 2 == 0) {
        throw std::logic_error("only odd window size allowed");
    }
    cv::Mat kernel = cv::Mat::zeros(window_size, 1, type);
    kernel.at<float>(0, 0) = 1;
    kernel.at<float>(window_size - 1, 0) = 1;
    kernel /= 2;
    return kernel;
}

cv::Mat uqr::GroundRemover::remove_ground(const cv::Mat& depth_image,
                                          const cv::Mat& angle_image,
                                          float threshold,
                                          int kernel_size){

  cv::Mat groundless = cv::Mat::zeros(depth_image.size(), CV_32F);

  for (int c = 0; c < depth_image.cols; ++c) {
    // start at bottom pixels and do bfs
    int r = depth_image.rows - 1;
    while (r > 0 && depth_image.at<float>(r, c) < 0.001f) {
      --r;
    }
    uqr::PointCord current_coord = PointCord(r, c);
    uint16_t current_label = this->labeler.get_label(current_coord);
    if (current_label > 0) {
      // this coord was already labeled, skip
      continue;
    }

    if (angle_image.at<float>(r, c) > 0.523599) {
      continue;
    }

    this->labeler.label_search(current_coord, 1);
  }

  auto label_image_ptr = labeler.label_image();

  kernel_size = std::max(kernel_size - 2, 3);
  cv::Mat kernel = this->uniform_kernal(kernel_size, CV_8U);
  cv::Mat dilated = cv::Mat::zeros(label_image_ptr->size(), label_image_ptr->type());
  cv::dilate(*label_image_ptr, dilated, kernel);
  for (int r = 0; r < dilated.rows; ++r) {
    for (int c = 0; c < dilated.cols; ++c) {
      if (dilated.at<uint16_t>(r, c) == 0) {
        // all unlabeled points are non-ground
        groundless.at<float>(r, c) = depth_image.at<float>(r, c);
      }
    }
  }
  return groundless;
}

float uqr::GroundRemover::get_max_angle(){
    return this->maxAngle;
}

cv::Mat* uqr::GroundRemover::get_groundless(){
    return &this->groundless;
}