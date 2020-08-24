/**
 * @author Riley Bowyer
 * @date 23-07-2020
 *
 * @brief Online Cloud Segmenter library
 *  This library introduces:
 *    + Segmenter to identify cones
 *
 * @namespace uqr
 */

#ifndef ONLINE_SEGMENTATION_H
#define ONLINE_SEGMENTATION_H

#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"

#include <opencv2/core/mat.hpp>

namespace uqr {
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

    class ProjectionParams{
        public:
            ProjectionParams();
            ProjectionParams(float min_angle, float max_angle, float step);
            ProjectionParams(float min_angle, float max_angle, int num_beams);

            void fill_angles();
            
            int find_closest(float angle);

            float from_index(int index);

            int from_angle(float angle);

            int len();

            void show_angles();
        private:
            float min_angle;
            float max_angle;
            float step;
            int num_beams;

            std::vector<float> angles;
            
    };

    class Projector{
        public:
            Projector();
            Projector(ProjectionParams rowParams, ProjectionParams colParams);
            // void convert(cv::Mat depthImage); // Read off Ray to find row, calc col from xy co-ords. Populate depthimage.
            void convert(pcl::PointCloud<pcl::PointXYZ> inputCloud); // Read off Ray to find row, calc col from xy co-ords. Populate depthimage.

            float get_max_depth();
            cv::Mat get_depth();
        private:
            ProjectionParams rowParams;
            ProjectionParams colParams;
            float max_depth;

            cv::Mat depthImage;
    };
} // NAMESPACE uqr
#endif //ONLINE_SEGMENTATION_H
