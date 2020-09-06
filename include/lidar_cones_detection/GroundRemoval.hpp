/**
 * @author Riley Bowyer
 * @date 31-08-2020
 *
 * @brief Projector library
 *  This library introduces:
 *    + Ground removal for projected depth images
 *
 * @namespace uqr
 */

#ifndef GROUNDREMOVAL_H
#define GROUNDREMOVAL_H

/// Internal Libraries
#include "lidar_cones_detection/ProjectionParameters.hpp"
#include "lidar_cones_detection/Projector.hpp"
#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/Cluster.hpp"

/// External Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/interface.h>

namespace uqr {

    class GroundRemover{
        public:
            /// Empty Constructor
            GroundRemover();

            /// Copy Constructor
            GroundRemover(const GroundRemover& otherRemover) = default;

            /// Copy Operator
            GroundRemover& operator=(const GroundRemover&) = default;

            /// Move Constructor
            GroundRemover(GroundRemover&&) = default;

            /// Move Operator
            GroundRemover& operator=(GroundRemover&&) = default;

            /// Constructor
            GroundRemover(ProjectionParams rowParams, ProjectionParams colParams, float angleStep, int windowSize);

           
            void process_image(const cv::Mat& depth_image);

            cv::Mat repair_depth(const cv::Mat& depth_image, int step, float depth_threshold);
            cv::Mat angle_image(const cv::Mat& depth_image);

            cv::Mat get_kernel(int window_size);
            cv::Mat smooth_image(const cv::Mat& image, int window_size);
            cv::Mat uniform_kernal(int window_size, int type);
            cv::Mat remove_ground(const cv::Mat& depth_image, const cv::Mat& angle_image, float threshold, int kernel_size);

            float get_max_angle();
            cv::Mat* get_groundless();

        private:
            /// Sensor Angle Parameters
            ProjectionParams rowParams;
            ProjectionParams colParams;
            Cluster labeler;

            cv::Mat groundless;

            float angleStep;
            float maxAngle;
            int windowSize;
    };
} // NAMESPACE uqr
#endif //GROUNDREMOVAL_H
