#include "lidar_cones_detection/PCLWrapper.hpp"
#include "lidar_cones_detection/IOUtility.hpp"
#include "lidar_cones_detection/Projector.hpp"
#include "lidar_cones_detection/ProjectionParameters.hpp"

#include "lidar_cones_detection/GroundRemoval.hpp"
#include "lidar_cones_detection/ConeSegment.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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

cv::Mat LabelsToColor(const cv::Mat& label_image) {
	cv::Mat color_image(label_image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	for (int row = 0; row < label_image.rows; ++row) {
		for (int col = 0; col < label_image.cols; ++col) {
			auto label = label_image.at<uint16_t>(row, col);
			auto random_color = RANDOM_COLORS[label % RANDOM_COLORS.size()];
			cv::Vec3b color = cv::Vec3b(random_color[0], random_color[1], random_color[2]);
			color_image.at<cv::Vec3b>(row, col) = color;
		}
	}
	return color_image;
}

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
	uqr::ProjectionParams colAngles((float)-M_PI,(float)M_PI,870);
	rowAngles.fill_angles();
	colAngles.fill_angles();

	uqr::Projector projector(rowAngles,colAngles);
	uqr::GroundRemover ground(rowAngles,colAngles,10,5);
	uqr::ConeSegmenter cones(rowAngles,colAngles,10,5);

	cv_bridge::CvImage dep_msg, ang_msg, lab_msg;
	cv::Mat dep_img, ang_img, lab_img;

	dep_msg.header.stamp = ros::Time::now();
	dep_msg.header.frame_id = "map";
	dep_msg.encoding = sensor_msgs::image_encodings::MONO8; // Or whatever
	ang_msg.header = dep_msg.header;
	ang_msg.encoding = dep_msg.encoding;
	lab_msg.header = dep_msg.header;
	lab_msg.encoding = sensor_msgs::image_encodings::RGB8;



	ros::Publisher ang_pub = nh.advertise<sensor_msgs::Image>("angle", 1000);
	ros::Publisher dep_pub = nh.advertise<sensor_msgs::Image>("depth", 1000);
	ros::Publisher lab_pub = nh.advertise<sensor_msgs::Image>("colour", 1000);

	ros::WallTime start_, end_;

	while(ros::ok()){
		start_ = ros::WallTime::now();

		projector.convert(cloud);
		auto depth_image = projector.get_depth();
		depth_image->convertTo(dep_img, 0, 255/projector.get_max_depth());

		ground.process_image(*depth_image);
		auto groundless = ground.get_groundless();
		groundless->convertTo(ang_img, 0, 255/projector.get_max_depth());
		cones.process_image(*groundless);
		LabelsToColor(cones.label_image()).convertTo(lab_img,CV_8UC3,1);
		
		end_ = ros::WallTime::now();

		// print results
		double execution_time = (end_ - start_).toNSec() * 1e-6;
		// ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
		dep_msg.image = dep_img;
		ang_msg.image = ang_img;
		lab_msg.image = lab_img;

		ang_pub.publish(ang_msg.toImageMsg());
		dep_pub.publish(dep_msg.toImageMsg());
		lab_pub.publish(lab_msg.toImageMsg());
		ros::spinOnce();
	}
}