/**
 * @author Riley Bowyer
 * @date 06-09-2020
 *
 * @brief Image Utility source
 */

#include "lidar_cones_detection/ImageUtil.hpp"

cv::Mat uqr::ImageUtil::repair_depth(const cv::Mat& depth_image, int step,
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

cv::Mat uqr::ImageUtil::get_kernel(int window_size){
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

cv::Mat uqr::ImageUtil::smooth_image(const cv::Mat& image, int window_size) {
  cv::Mat kernel = this->get_kernel(window_size);

  cv::Mat smoothed_image;  // init an empty smoothed image
  cv::filter2D(image, smoothed_image, -1, kernel, cv::Point(-1, -1),
               0, cv::BORDER_REFLECT101);
  return smoothed_image;
}

cv::Mat uqr::ImageUtil::uniform_kernal(int window_size, int type){
    if (window_size % 2 == 0) {
        throw std::logic_error("only odd window size allowed");
    }
    cv::Mat kernel = cv::Mat::zeros(window_size, 1, type);
    kernel.at<float>(0, 0) = 1;
    kernel.at<float>(window_size - 1, 0) = 1;
    kernel /= 2;
    return kernel;
}