/**
 * @author Riley Bowyer
 * @date 06-09-2020
 *
 * @brief Image Utility library
 *  This library introduces:
 *    + Default Utility functions depth image processing
 *
 * @namespace uqr
 */

#ifndef IMUTIL_H
#define IMUTIL_H

/// External Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/interface.h>

namespace uqr {
    class ImageUtil{
        public:
            /// Empty Constructor
            ImageUtil(){};

            /// Copy Constructor
            ImageUtil(const ImageUtil& otherUtil) = default;

            /// Copy Operator
            ImageUtil& operator=(const ImageUtil&) = default;

            /// Move Constructor
            ImageUtil(ImageUtil&&) = default;

            /// Move Operator
            ImageUtil& operator=(ImageUtil&&) = default;

            cv::Mat repair_depth(const cv::Mat& depth_image, int step, float depth_threshold);
            cv::Mat get_kernel(int window_size);
            cv::Mat smooth_image(const cv::Mat& image, int window_size);
            cv::Mat uniform_kernal(int window_size, int type);
    };
} // NAMESPACE uqr
#endif //IMUTIL_H
