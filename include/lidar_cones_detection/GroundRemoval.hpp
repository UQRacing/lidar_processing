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
#include "lidar_cones_detection/ImageUtil.hpp"

/// External Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/interface.h>

namespace uqr {

    class GroundRemover{
        public:
            /// Empty Constructor
            GroundRemover(){};

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

            /**
             * Preprocess and remove the ground from an image.
             *
             * @param depth_image Depth image to be segmented.
             */
            void process_image(const cv::Mat& depth_image);

            /**
             * Create an image consisting of the angles between points vertically.
             *
             * @param depth_image Depth image to be segmented.
             */
            cv::Mat angle_image(const cv::Mat& depth_image);

            /**
             * All in the name, remove the ground from an image.
             *
             * @param depth_image Depth image to be segmented.
             * @param angle_image Angle image to be used.
             */
            void remove_ground(const cv::Mat& depth_image, const cv::Mat& angle_image);

            /**
             * Get the internal label image.
             */
            cv::Mat label_image();

            /**
             * Get the greatest angle difference.
             */
            float get_max_angle();
            
            /**
             * Get the groundless image.
             */
            cv::Mat* get_groundless();

        private:
            /// Sensor Angle Parameters
            ProjectionParams rowParams;
            ProjectionParams colParams;

            /// Image Utilities
            Cluster labeler;
            ImageUtil depthUtil;

            /// Images
            cv::Mat groundless;

            /// Misc Variables
            float angleStep;
            float maxAngle;
            int windowSize;
    };
} // NAMESPACE uqr
#endif //GROUNDREMOVAL_H
