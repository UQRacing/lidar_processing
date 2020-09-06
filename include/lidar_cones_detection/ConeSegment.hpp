/**
 * @author Riley Bowyer
 * @date 06-09-2020
 *
 * @brief Projector library
 *  This library introduces:
 *    + Segmenting of depth images
 *
 * @namespace uqr
 */

#ifndef CONESEGMENT_H
#define CONESEGMENT_H

/// Internal Libraries
#include "lidar_cones_detection/ProjectionParameters.hpp"
#include "lidar_cones_detection/Projector.hpp"
#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/Cluster.hpp"
#include "lidar_cones_detection/ImageUtil.hpp"

/// External Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/interface.h>

namespace uqr {
    class ConeSegmenter{
        public:
            /// Empty Constructor
            ConeSegmenter();

            /// Copy Constructor
            ConeSegmenter(const ConeSegmenter& otherSegmenter) = default;

            /// Copy Operator
            ConeSegmenter& operator=(const ConeSegmenter&) = default;

            /// Move Constructor
            ConeSegmenter(ConeSegmenter&&) = default;

            /// Move Operator
            ConeSegmenter& operator=(ConeSegmenter&&) = default;

            /// Constructor
            ConeSegmenter(ProjectionParams rowParams, ProjectionParams colParams, float angleStep, int windowSize);

            /**
             * Cluster Points!
             *
             * @param depth_image The image to be segmented.
             */
            void process_image(const cv::Mat& depth_image);

            /**
             * Get the total number of segments.
             */
            int segments();

            /**
             * Get the label image.
             */
            cv::Mat label_image();

            /**
             * Get a cluster by ID.
             *
             * @param depth_image The depth image to be masked.
             * @param id Segment ID.
             */
            cv::Mat get_cluster(const cv::Mat& depth_image, int id);

        private:
            /// Sensor Angle Parameters
            ProjectionParams rowParams;
            ProjectionParams colParams;

            /// Image Utils
            Cluster labeler;
            ImageUtil depthUtil;

            /// Images
            cv::Mat col_angles;

            /// Misc Variables
            float angleStep;
            int windowSize;
            int labels;
    };
} // NAMESPACE uqr
#endif //CONESEGMENT_H
