#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include "lidar_cones_detection/Projector.hpp"
#include "lidar_cones_detection/ProjectionParameters.hpp"

#include "lidar_cones_detection/GroundRemoval.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main (int argc, char** argv){
	ros::init(argc, argv, "lidar_cone");
	ros::NodeHandle nh;
	uqr::PointCloud incloud;
	std::string filePath;
	nh.param<std::string>("/segmenter/file_path", filePath, "table.pcd");

	uqr::load_cloud(filePath, incloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud = (pcl::PointCloud<pcl::PointXYZ>)incloud;

	uqr::ProjectionParams rowAngles((float)-0.262,(float)0.262,16);
	uqr::ProjectionParams colAngles((float)-3.1415,(float)3.1415,870);
	rowAngles.fill_angles();
	colAngles.fill_angles();

	uqr::Projector projector(rowAngles,colAngles);
	uqr::GroundRemover ground(rowAngles,colAngles,50,5);

	cv_bridge::CvImage dep_msg, ang_msg;
	cv::Mat dep_img, ang_img;

	dep_msg.header.stamp = ros::Time::now();
	dep_msg.header.frame_id = "map";
	dep_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	ang_msg.header = dep_msg.header;
	ang_msg.encoding = dep_msg.encoding;


	ros::Publisher ang_pub = nh.advertise<sensor_msgs::Image>("angle", 1000);
	ros::Publisher dep_pub = nh.advertise<sensor_msgs::Image>("depth", 1000);

	ros::WallTime start_, end_;

	while(ros::ok()){
		start_ = ros::WallTime::now();

		projector.convert(cloud);
		auto depth_image = projector.get_depth();
		depth_image->convertTo(dep_img, 0, 255/projector.get_max_depth());

		ground.process_image(*depth_image);
		auto groundless = ground.get_groundless();
		groundless->convertTo(ang_img, 0, 255/projector.get_max_depth());

		end_ = ros::WallTime::now();

		// print results
		double execution_time = (end_ - start_).toNSec() * 1e-6;
		ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
		dep_msg.image = dep_img;
		ang_msg.image = ang_img;

		ang_pub.publish(ang_msg.toImageMsg());
		dep_pub.publish(dep_msg.toImageMsg());
		ros::spinOnce();
	}
}