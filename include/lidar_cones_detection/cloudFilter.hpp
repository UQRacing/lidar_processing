#pragma once

// ROS Packages
#include <ros/ros.h>

// Messages
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class cloud_filter{
    public:
        /**
         * @brief Class initialisation.
         *
         * Handles the callback and filtering of a pointcloud2 on a specified topic.
         *
         * @param topic Topic on which the subscriber should listen.
         * @param nh The nodhandle used to generate the ros node.
         */
        cloud_filter(std::string topic, ros::NodeHandle nh, double voxelSize);

    private:

        ros::Subscriber cloudSub_;
        ros::Publisher cloud_filteredPub_;
        pcl::PointCloud<pcl::PointXYZ> cloud_;

        double voxelSize_;

        /**
         * @brief Filter the recieved cloud and republish.
         *
         * @param msg Pointcloud2 Pointer received from subscriber callback.
         */
        void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);
};
