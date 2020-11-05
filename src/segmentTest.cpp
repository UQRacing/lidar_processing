#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include "lidar_cones_detection/Projector.hpp"
#include "lidar_cones_detection/ProjectionParameters.hpp"

#include "lidar_cones_detection/GroundRemoval.hpp"
#include "lidar_cones_detection/ConeSegment.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

uqr::PointCloud inputCloud;
bool cloudRec = false;

void cloud_cb(const sensor_msgs::PointCloud2& msg){
	inputCloud = uqr::PointCloud(msg);
	cloudRec = true;
}

int main (int argc, char** argv){
	ros::init(argc, argv, "lidar_cone");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("velodyne_points", 1, cloud_cb);
	
	float ground_angle;
	float segment_angle;

	int ground_window;
	int segment_window;

	nh.param<float>("/segmenter/ground_angle", ground_angle, 10.0);
	nh.param<int>("/segmenter/ground_window", ground_window, 7);
	nh.param<float>("/segmenter/segment_angle", segment_angle, 10.0);
	nh.param<int>("/segmenter/segment_window", segment_window, 7);

	// Fill Projection Params
	uqr::ProjectionParams rowAngles((float)-0.262,(float)0.262,16);
	uqr::ProjectionParams colAngles((float)-M_PI,(float)M_PI,1875); // Note: This must be reflected in the simulated sensor otherwise errors occur...
	rowAngles.fill_angles();
	colAngles.fill_angles();

	// Init Segmenters
	uqr::Projector projector(rowAngles,colAngles);
	uqr::GroundRemover ground(rowAngles,colAngles,ground_angle,ground_window);
	uqr::ConeSegmenter cones(rowAngles,colAngles,segment_angle,segment_window);

	// Init Pubs
	uqr::ImagePublisher colPub("/colour");
	uqr::ImagePublisher depPub("/depth");
	uqr::ImagePublisher groundPub("/ground");

	ros::WallTime start_, end_;
	while(ros::ok()){
		if(cloudRec){
			start_ = ros::WallTime::now();
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
			cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
			*cloud = (pcl::PointCloud<pcl::PointXYZ>)inputCloud;

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
		}
		ros::spinOnce();
	}
}