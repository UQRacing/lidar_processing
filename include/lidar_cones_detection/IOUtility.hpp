/**
 * @author Riley Bowyer
 * @date 18-07-2020
 *
 * @brief IO Utility function library
 *  This library introduces:
 *    + Simplified ROS publication and subscription
 *    + Simplified pointcloud file  saving and reading.
 *
 * @namespace uqr
 */

#ifndef LIDAR_CONES_DETECTION_IOUTILITY_H
#define LIDAR_CONES_DETECTION_IOUTILITY_H

#include "lidar_cones_detection/PCLWrapper.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>


#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/interface.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace uqr {

    /**
     * Load a pointcloud from a specified file.
     *
     * A wrapper for the PCL voxelisation functions to
     * produce a uniformally voxelised pointcloud. 
     *
     * @param file_path The path to the cloud, including file extension.
     * @param outputCloud Pointer to the desired output cloud. The loaded cloud will be saved here.
     */
    void load_cloud(const std::string& file_path, uqr::PointCloud& outputCloud);

    /**
     * Load a pointcloud from a specified file.
     *
     * A wrapper for the PCL voxelisation functions to
     * produce a uniformally voxelised pointcloud. 
     *
     * @param file_path The path to the cloud, including file extension.
     * @param input_cloud Pointer to the desired input cloud. This cloud will be saved.
     */
    void write_cloud(const std::string file_path, uqr::PointCloud& input_cloud);

    /**
     * Visualise a PointCloud.
     *
     * A wrapper for the PCL visualiser.
     *
     * @param input_cloud Pointer to the desired input cloud. This cloud will be saved.
     */
    void view_cloud(uqr::PointCloud& input_cloud);

    /**
     * @brief Simple PointCloud Publisher
     *
     * Allows easy visualisation of a variety of pointcloud types
     */
    class cloudPublisher {
        public:
            /// Constructor with Topic
            cloudPublisher(std::string topic);
            
            // Publish Methods
            /**
             * Publish a uqr::PointCloud
             *
             * @param cloud The cloud to be published.
             */
            void publish(PointCloud& cloud, std::string frame="map" , ros::Time timeStamp=ros::Time::now());

            /**
             * Publish a sensor_msgs::PointCloud2
             *
             * @param cloud The cloud to be published.
             * @param frame The frame in which the cloud is to be published.
             * @param timeStamp The time-stamp with which the cloud is to be published.
             */
            void publish(sensor_msgs::PointCloud2& cloud, std::string frame="map" , ros::Time timeStamp=ros::Time::now());

            /**
             * Publish a pcl::PCLPointCloud2
             *
             * @param cloud The cloud to be published.
             * @param frame The frame in which the cloud is to be published.
             * @param timeStamp The time-stamp with which the cloud is to be published.
             */
            void publish(pcl::PCLPointCloud2& cloud, std::string frame="map" , ros::Time timeStamp=ros::Time::now());

            /**
             * Publish a pcl::PointCloud<pcl::PointXYZ>
             *
             * @param cloud The cloud to be published.
             * @param frame The frame in which the cloud is to be published.
             * @param timeStamp The time-stamp with which the cloud is to be published.
             */
            void publish(pcl::PointCloud<pcl::PointXYZ>& cloud, std::string frame="map" , ros::Time timeStamp=ros::Time::now());

            /// Destructor
            ~cloudPublisher() = default;

        private:
            ros::NodeHandle nh;
            ros::Publisher cloudPub;
    };
        
    class ImagePublisher {
        public:
            /// Constructor with Topic
            ImagePublisher(std::string topic);
            
            /// Empty Constructor
            ImagePublisher() : ImagePublisher("/image"){};
            
            // Publish Methods
            /**
             * Publish a cv::Mat as monochrome.
             *
             * @param image The image to be published.
             * @param scale The number to scale the input image by.
             */
            void publish(const cv::Mat& image, float scale, std::string frame="map", ros::Time timeStamp=ros::Time::now());

            /**
             * Publish a cv::Mat as colour.
             *
             * @param label_image The image to be published.
             */
            void colour_publish(const cv::Mat& label_image, std::string frame="map", ros::Time timeStamp=ros::Time::now());

            /// Destructor
            ~ImagePublisher() = default;

        private:
            ros::NodeHandle nh;
            ros::Publisher imgPub;
    };

    /// This is my pseudo random colours for display. #SOZ NOT SOZ
    static constexpr std::array<std::array<int, 3>, 200> RANDOM_COLORS = {{
        {{104, 109, 253}}, {{125, 232, 153}}, {{158, 221, 134}},
        {{228, 109, 215}}, {{249, 135, 210}}, {{255, 207, 237}},
        {{151, 120, 235}}, {{145, 123, 213}}, {{172, 243, 184}},
        {{105, 131, 110}}, {{217, 253, 154}}, {{250, 102, 109}},
        {{116, 179, 127}}, {{200, 251, 206}}, {{117, 146, 240}},
        {{234, 162, 176}}, {{160, 172, 171}}, {{205, 129, 168}},
        {{197, 167, 238}}, {{234, 248, 101}}, {{226, 240, 119}},
        {{189, 211, 231}}, {{226, 170, 216}}, {{109, 180, 162}},
        {{115, 167, 221}}, {{162, 134, 131}}, {{203, 169, 114}},
        {{221, 138, 114}}, {{246, 146, 237}}, {{200, 167, 244}},
        {{198, 150, 236}}, {{237, 235, 191}}, {{132, 137, 171}},
        {{136, 219, 103}}, {{229, 210, 135}}, {{133, 188, 111}},
        {{142, 144, 142}}, {{122, 189, 120}}, {{127, 142, 229}},
        {{249, 147, 235}}, {{255, 195, 148}}, {{202, 126, 227}},
        {{135, 195, 159}}, {{139, 173, 142}}, {{123, 118, 246}},
        {{254, 186, 204}}, {{184, 138, 221}}, {{112, 160, 229}},
        {{243, 165, 249}}, {{200, 194, 254}}, {{172, 205, 151}},
        {{196, 132, 119}}, {{240, 251, 116}}, {{186, 189, 147}},
        {{154, 162, 144}}, {{178, 103, 147}}, {{139, 188, 175}},
        {{156, 163, 178}}, {{225, 244, 174}}, {{118, 227, 101}},
        {{176, 178, 120}}, {{113, 105, 164}}, {{137, 105, 123}},
        {{144, 114, 196}}, {{163, 115, 216}}, {{143, 128, 133}},
        {{221, 225, 169}}, {{165, 152, 214}}, {{133, 163, 101}},
        {{212, 202, 171}}, {{134, 255, 128}}, {{217, 201, 143}},
        {{213, 175, 151}}, {{149, 234, 191}}, {{242, 127, 242}},
        {{152, 189, 230}}, {{152, 121, 249}}, {{234, 253, 138}},
        {{152, 234, 147}}, {{171, 195, 244}}, {{254, 178, 194}},
        {{205, 105, 153}}, {{226, 234, 202}}, {{153, 136, 236}},
        {{248, 242, 137}}, {{162, 251, 207}}, {{152, 126, 144}},
        {{180, 213, 122}}, {{230, 185, 113}}, {{118, 148, 223}},
        {{162, 124, 183}}, {{180, 247, 119}}, {{120, 223, 121}},
        {{252, 124, 181}}, {{254, 174, 165}}, {{188, 186, 210}},
        {{254, 137, 161}}, {{216, 222, 120}}, {{215, 247, 128}},
        {{121, 240, 179}}, {{135, 122, 215}}, {{255, 131, 237}},
        {{224, 112, 171}}, {{167, 223, 219}}, {{103, 200, 161}},
        {{112, 154, 156}}, {{170, 127, 228}}, {{133, 145, 244}},
        {{244, 100, 101}}, {{254, 199, 148}}, {{120, 165, 205}},
        {{112, 121, 141}}, {{175, 135, 134}}, {{221, 250, 137}},
        {{247, 245, 231}}, {{236, 109, 115}}, {{169, 198, 194}},
        {{196, 195, 136}}, {{138, 255, 145}}, {{239, 141, 147}},
        {{194, 220, 253}}, {{149, 209, 204}}, {{241, 127, 132}},
        {{226, 184, 108}}, {{222, 108, 147}}, {{109, 166, 185}},
        {{152, 107, 167}}, {{153, 117, 222}}, {{165, 171, 214}},
        {{189, 196, 243}}, {{248, 235, 129}}, {{120, 198, 202}},
        {{223, 206, 134}}, {{175, 114, 214}}, {{115, 196, 189}},
        {{157, 141, 112}}, {{111, 161, 201}}, {{207, 183, 214}},
        {{201, 164, 235}}, {{168, 187, 154}}, {{114, 176, 229}},
        {{151, 163, 221}}, {{134, 160, 173}}, {{103, 112, 168}},
        {{209, 169, 218}}, {{137, 220, 119}}, {{168, 220, 210}},
        {{182, 192, 194}}, {{233, 187, 120}}, {{223, 185, 160}},
        {{120, 232, 147}}, {{165, 169, 124}}, {{251, 159, 129}},
        {{182, 114, 178}}, {{159, 116, 158}}, {{217, 121, 122}},
        {{106, 229, 235}}, {{164, 208, 214}}, {{180, 178, 142}},
        {{110, 206, 136}}, {{238, 152, 205}}, {{109, 245, 253}},
        {{213, 232, 131}}, {{215, 134, 100}}, {{163, 140, 135}},
        {{233, 198, 143}}, {{221, 129, 224}}, {{150, 179, 137}},
        {{171, 128, 119}}, {{210, 245, 246}}, {{209, 111, 161}},
        {{237, 133, 194}}, {{166, 157, 255}}, {{191, 206, 225}},
        {{125, 135, 110}}, {{199, 188, 196}}, {{196, 101, 202}},
        {{237, 211, 167}}, {{134, 118, 177}}, {{110, 179, 126}},
        {{196, 182, 196}}, {{150, 211, 218}}, {{162, 118, 228}},
        {{150, 209, 185}}, {{219, 151, 148}}, {{201, 168, 104}},
        {{237, 146, 123}}, {{234, 163, 146}}, {{213, 251, 127}},
        {{227, 152, 214}}, {{230, 195, 100}}, {{136, 117, 222}},
        {{180, 132, 173}}, {{112, 226, 113}}, {{198, 155, 126}},
        {{149, 255, 152}}, {{223, 124, 170}}, {{104, 146, 255}},
        {{113, 205, 183}}, {{100, 156, 216}},
    }};

};
#endif //LIDAR_CONES_DETECTION_IOUTILITY_H
