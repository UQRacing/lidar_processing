/**
 * @author Riley Bowyer
 * @date 06-09-2020
 *
 * @brief Cluster library
 *  This library introduces:
 *    + Cluster groups of points
 *
 * @namespace uqr
 */

#ifndef CLUSTER_H
#define CLUSTER_H

/// External Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

namespace uqr {

    struct PointCord{
        int r;
        int c;
        PointCord(int r=0, int c=0) : r(r), c(c){}

        PointCord& operator=(const PointCord& a){
            r=a.r;
            c=a.c;
            return *this;
        }

        PointCord operator+(const PointCord& a) const{
            return PointCord(a.r+r, a.c+c);
        }

        PointCord operator-(const PointCord& a) const{
            return PointCord(r-a.r, c-a.c);
        }

        bool operator==(const PointCord& a) const{
            return (r == a.r && c == a.c);
        }
    };

    class Cluster{
        public:
            /// Empty Constructor
            Cluster();

            /// Copy Constructor
            Cluster(const Cluster& otherCluster) = default;

            /// Copy Operator
            Cluster& operator=(const Cluster&) = default;

            /// Move Constructor
            Cluster(Cluster&&) = default;

            /// Move Operator
            Cluster& operator=(Cluster&&) = default;

            /// Constructor
            Cluster(uint16_t rows, uint16_t cols, float angleStep);

            /**
             * Store Depth Image.
             *
             * @param depth_image Depth image to be stored.
             */
            void set_depth(const cv::Mat& depth_image);

            /**
             * Store Angle Image.
             *
             * @param angle_image Angle image to be stored.
             */
            void set_angle(const cv::Mat& angle_image);

            /**
             * Clear the current label image.
             */
            void clear_labels();

            /**
             * Segment the depth image vertically - Ground Removal.
             *
             * @param start The coordinate to start at.
             * @param label Label to be assigned.
             */
            void vertical_search(PointCord start, uint16_t label);

            /**
             * Segment the depth image horizontally - Cone Segment.
             *
             * @param start The coordinate to start at.
             * @param label Label to be assigned.
             */
            void horizontal_search(PointCord start, uint16_t label);

            /**
             * Assign a point a corresponding label.
             *
             * @param point The point to be assigned.
             * @param label The corresponding label.
             */
            void set_label(PointCord point, uint16_t label);

            /**
             * Get the label at a point.
             *
             * @param point The point to be looked at.
             */
            uint16_t get_label(PointCord point);

            /**
             * Get the depth at a point.
             *
             * @param point The point to be looked at.
             */
            float get_depth(PointCord point);

            /**
             * Get the angle at a point.
             *
             * @param point The point to be looked at.
             */
            float get_angle(PointCord point);

            /**
             * Get the internal label image.
             */
            cv::Mat* label_image();
    
        private:
            /// Neighbour Constructors
            static const uint16_t ROWS = 1;
            static const uint16_t COLS = 1;
            static constexpr int16_t NEIGHBOURS = 2 * ROWS + 2 * COLS;
            std::array<PointCord, NEIGHBOURS> Neighbourhood;

            /// Images
            cv::Mat labels;
            const cv::Mat* depth_image_ptr;
            const cv::Mat* angle_image_ptr;

            /// Cut-Off Threshold
            float angleThresh;

    };
} // NAMESPACE uqr
#endif //CLUSTER_H
