#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include "lidar_cones_detection/Projector.hpp"
#include "lidar_cones_detection/ProjectionParameters.hpp"

#include "lidar_cones_detection/GroundRemoval.hpp"
#include "lidar_cones_detection/ConeSegment.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main (int argc, char** argv){
	ros::init(argc, argv, "lidar_cone");
	ros::NodeHandle nh;
	
	std::string filePath;
	nh.param<std::string>("/segmenter/file_path", filePath, "table.pcd");
	
	float ground_angle;
	float segment_angle;

	int ground_window;
	int segment_window;

	nh.param<float>("/segmenter/ground_angle", ground_angle, 10.0);
	nh.param<int>("/segmenter/ground_window", ground_window, 7);
	nh.param<float>("/segmenter/segment_angle", segment_angle, 10.0);
	nh.param<int>("/segmenter/segment_window", segment_window, 7);

	// Load Cloud
	uqr::PointCloud incloud;
	uqr::load_cloud(filePath, incloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud = (pcl::PointCloud<pcl::PointXYZ>)incloud;

	// Fill Projection Params
	uqr::ProjectionParams rowAngles((float)-0.262,(float)0.262,16);
	uqr::ProjectionParams colAngles((float)-M_PI,(float)M_PI,870);
	rowAngles.fill_angles();
	colAngles.fill_angles();

	// Init Segmenters
	uqr::Projector projector(rowAngles,colAngles);
	uqr::GroundRemover ground(rowAngles,colAngles,10,5);
	uqr::ConeSegmenter cones(rowAngles,colAngles,10,5);

	// Init Pubs
	uqr::ImagePublisher colPub("/colour");
	uqr::ImagePublisher depPub("/depth");
	uqr::ImagePublisher groundPub("/ground");

	ros::WallTime start_, end_;

	while(ros::ok()){
		start_ = ros::WallTime::now();

		projector.convert(cloud);
		auto depth_image = projector.get_depth();
		depPub.publish(*depth_image,255/projector.get_max_depth());

		ground.process_image(*depth_image);
		auto groundless = ground.get_groundless();
		groundPub.publish(*groundless,255/projector.get_max_depth());

		cones.process_image(*groundless);
		colPub.colour_publish(cones.label_image());
		
		end_ = ros::WallTime::now();

		// print results
		double execution_time = (end_ - start_).toNSec() * 1e-6;
		ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
		ros::spinOnce();
	}
}