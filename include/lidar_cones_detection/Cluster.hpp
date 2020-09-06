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

            void set_depth(const cv::Mat& depth_image);
            void set_angle(const cv::Mat& angle_image);
            void clear_labels();

           
            void vertical_search(PointCord start, uint16_t label);
            void horizontal_search(PointCord start, uint16_t label);
            void set_label(PointCord point, uint16_t label);

            uint16_t get_label(PointCord point);
            float get_depth(PointCord point);
            float get_angle(PointCord point);

            cv::Mat* label_image();
    
        private:
            static const uint16_t ROWS = 1;
            static const uint16_t COLS = 1;
            static constexpr int16_t NEIGHBOURS = 2 * ROWS + 2 * COLS;
            /// Sensor Angle Parameters
            cv::Mat labels;
            float angleThresh;

            const cv::Mat* depth_image_ptr;
            const cv::Mat* angle_image_ptr;

            std::array<PointCord, NEIGHBOURS> Neighbourhood;
    };
} // NAMESPACE uqr
#endif //CLUSTER_H
