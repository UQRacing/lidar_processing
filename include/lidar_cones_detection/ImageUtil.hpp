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

            /**
             * Fill in any small gaps.
             *
             * @param depth_image Image to be filled.
             * @param step Radius in which to fill.
             * @param depth_threshold Comparative depth to stop filling at.
             */
            cv::Mat repair_depth(const cv::Mat& depth_image, int step, float depth_threshold);

            /**
             * Generate a Savitsky-golay filter.
             *
             * @param window_size Window size of the filter, making this larger increases the "smoothness".
             */
            cv::Mat get_kernel(int window_size);
            
            /**
             * Smooth a depth image.
             *
             * @param image The image to be smoothed.
             * @param window_size Window size of the filter, making this larger increases the "smoothness".
             */
            cv::Mat smooth_image(const cv::Mat& image, int window_size);
            
            /**
             * Get an identity kernal
             *
             * @param window_size Window size for uniform filter, (dimensions of matrix).
             * @param type Kernal data type.
             */
            cv::Mat uniform_kernal(int window_size, int type);
    };
} // NAMESPACE uqr
#endif //IMUTIL_H
