#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include "lidar_cones_detection/Projector.hpp"
#include "lidar_cones_detection/ProjectionParameters.hpp"

#include "lidar_cones_detection/GroundRemoval.hpp"
#include "lidar_cones_detection/ConeSegment.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#define DEG_TO_RAD 0.01745329251
class SegmenterTester{
  public:
    SegmenterTester();
    
    void cloud_cb(const sensor_msgs::PointCloud2& msg);
  
  private:
    ros::NodeHandle nh;
    
    float ground_angle;
    float segment_angle;

    int ground_window;
    int segment_window;
    
    uqr::ProjectionParams rowAngles;
    uqr::ProjectionParams colAngles;
    
    uqr::Projector projector;
    uqr::GroundRemover ground;
    uqr::ConeSegmenter cones;
    
    uqr::ImagePublisher colPub;
    uqr::ImagePublisher depPub;
    uqr::ImagePublisher groundPub;
    
    ros::Subscriber sub;
};

SegmenterTester::SegmenterTester(){
  nh.getParam("/segmenter/ground/angle", ground_angle);
	nh.getParam("/segmenter/ground/window", ground_window);
	nh.getParam("/segmenter/segment/angle", segment_angle);
	nh.getParam("/segmenter/segment/window", segment_window);
  
  float v_res;
  float v_min;
  float v_max;
  
  nh.getParam("/segmenter/vertical/min_angle", v_min);
	nh.getParam("/segmenter/vertical/max_angle", v_max);
	nh.getParam("/segmenter/vertical/resolution", v_res);
  
  float h_res;
  float h_min;
  float h_max;
  
  nh.getParam("/segmenter/horizontal/min_angle", h_min);
	nh.getParam("/segmenter/horizontal/max_angle", h_max);
	nh.getParam("/segmenter/horizontal/resolution", h_res);
  
	sub = nh.subscribe("lidar/raw", 1, &SegmenterTester::cloud_cb, this);
  
  ROS_INFO_STREAM("Vertical: (" << v_res << ", " << v_min << ", " << v_max << ")");
  ROS_INFO_STREAM("Horizontal: (" << h_res << ", " << h_min << ", " << h_max << ")");
  ROS_INFO_STREAM("Vertical " << round((v_max-v_min)/v_res) << "\tHorizontal: " << round((h_max-h_min)/h_res));
  
  // Note: This must be reflected in the simulated sensor otherwise errors occur...
  rowAngles = uqr::ProjectionParams(v_min*DEG_TO_RAD, v_max*DEG_TO_RAD, v_res*DEG_TO_RAD);
	colAngles = uqr::ProjectionParams(h_min*DEG_TO_RAD, h_max*DEG_TO_RAD, h_res*DEG_TO_RAD);

  // Fill Projection Params
	rowAngles.fill_angles();
	colAngles.fill_angles();
  
  // Init Segmenters
	projector = uqr::Projector(rowAngles,colAngles);
	ground = uqr::GroundRemover(rowAngles,colAngles,ground_angle,ground_window);
	cones = uqr::ConeSegmenter(rowAngles,colAngles,segment_angle,segment_window);

	// Init Pubs
	colPub = uqr::ImagePublisher("/colour");
	depPub = uqr::ImagePublisher("/depth");
	groundPub = uqr::ImagePublisher("/ground");
}

void SegmenterTester::cloud_cb(const sensor_msgs::PointCloud2& msg){
	ros::WallTime start_, end_;
  uqr::PointCloud inputCloud(msg);

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

int main (int argc, char** argv){
	ros::init(argc, argv, "lidar_cone");
  SegmenterTester tester;
	while(ros::ok()){
		ros::spinOnce();
	}
}