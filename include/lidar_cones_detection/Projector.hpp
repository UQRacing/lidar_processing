/**
 * @author Riley Bowyer
 * @date 23-07-2020
 *
 * @brief Projector library
 *  This library introduces:
 *    + Projector to convert a cloud to a depth image
 *
 * @namespace uqr
 */

#ifndef PROJECTOR_H
#define PROJECTOR_H

/// Internal Libraries
#include "lidar_cones_detection/ProjectionParameters.hpp"
#include "lidar_cones_detection/PCLWrapper.hpp"

/// External Libraries
#include <opencv2/core/mat.hpp>

namespace uqr {
    class Projector{
        public:
            /// Empty Constructor
            Projector();

            /// Copy Constructor
            Projector(const Projector& otherProjector) = default;

            /// Copy Operator
            Projector& operator=(const Projector&) = default;

            /// Move Constructor
            Projector(Projector&&) = default;

            /// Move Operator
            Projector& operator=(Projector&&) = default;

            /// Constructor
            Projector(ProjectionParams rowParams, ProjectionParams colParams);

            /**
            * Depth to Cloud
            *
            * Un-Project a depth image to a pointcloud.
            *
            * @param depthImage Depthimage to Un-Project.
            * @return Void until developped.
            */
            void convert(cv::Mat depthImage);

            /**
            * Cloud to Depth
            *
            * Project a pointcloud to a depth image.
            *
            * @param depthImage Cloud to project.
            */
            void convert(pcl::PointCloud<pcl::PointXYZ> inputCloud);

            /**
            * Maximum Depth
            *
            * Get the maximum depth in the current depthImage for scaling.
            *
            * @return The current max depth.
            */
            float get_max_depth();

            /**
            * Depth Image
            *
            * Get the current depth image.
            *
            * @return The _depth_.
            */
            cv::Mat get_depth();
        private:
            /// Sensor Angle Parameters
            ProjectionParams rowParams;
            ProjectionParams colParams;
            
            /// Max Depth in Image
            float max_depth;

            /// Depth Image object
            cv::Mat depthImage;
    };
} // NAMESPACE uqr
#endif //PROJECTOR_H
