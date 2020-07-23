/**
 * @author Riley Bowyer
 * @date 23-07-2020
 *
 * @brief Online Cloud Segmenter source
 */

#include "lidar_cones_detection/OnlineSegmentation.hpp"
void uqr::OnlineSegmenter::set_viewer_pose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose){
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}
uqr::OnlineSegmenter::OnlineSegmenter(){
    coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    rangeImage = pcl::RangeImage::Ptr(new pcl::RangeImage); 

    nh.param<float>("/segmenter/cam_pose/translation/x", x_trans, 0);
    nh.param<float>("/segmenter/cam_pose/translation/y", y_trans, 0);
    nh.param<float>("/segmenter/cam_pose/translation/z", z_trans, 0);

    nh.param<float>("/segmenter/cam_pose/rotation/w", w_rot, 1);
    nh.param<float>("/segmenter/cam_pose/rotation/x", x_rot, 0);
    nh.param<float>("/segmenter/cam_pose/rotation/y", y_rot, 0);
    nh.param<float>("/segmenter/cam_pose/rotation/z", z_rot, 0);

    scene_sensor_pose =  Eigen::Affine3f(Eigen::Affine3f::Identity());
    scene_sensor_pose.translate(Eigen::Vector3f(x_trans,y_trans,z_trans));
    scene_sensor_pose.rotate(Eigen::Quaternionf(w_rot,x_rot,y_rot,z_rot));

    nh.param<double>("/segmenter/vertical_fov", vertical_fov, 180);
    nh.param<double>("/segmenter/horizontal_fov", horizontal_fov, 360);
    
    nh.param<double>("/segmenter/vertical_resolution", vertical_angle_res, 0.1);
    nh.param<double>("/segmenter/horizontal_resolution", horizontal_angle_res, 0.1);

    nh.param<double>("/segmenter/noise", noise_level, 0.0);
    nh.param<double>("/segmenter/min_range", min_range, 0.0);
    nh.param<int>("/segmenter/border_size", border_size, 1);

    nh.param<bool>("/segmenter/calibrate_pose", calibrate_pose, false);
}

void uqr::OnlineSegmenter::to_range_image(uqr::PointCloud& input_cloud){

    rangeImage->createFromPointCloud((pcl::PointCloud<pcl::PointXYZ>) input_cloud, 
                                      pcl::deg2rad(horizontal_angle_res), pcl::deg2rad(vertical_angle_res),
                                      pcl::deg2rad(horizontal_fov), pcl::deg2rad(vertical_fov),
                                      scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

    if(calibrate_pose){
        calibrate_cam(input_cloud);
    }
}

void uqr::OnlineSegmenter::calibrate_cam(uqr::PointCloud& input_cloud){

    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (rangeImage, 0, 0, 0);
    viewer.addPointCloud (rangeImage, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

    viewer.initCameraParameters ();
    set_viewer_pose(viewer, rangeImage->getTransformationToWorldSystem ());
    
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage(*rangeImage);
    
    while (!viewer.wasStopped ())
    {
        range_image_widget.spinOnce ();
        viewer.spinOnce ();
        pcl_sleep (0.01);
        
        if (true){
        scene_sensor_pose = viewer.getViewerPose();
        rangeImage->createFromPointCloud((pcl::PointCloud<pcl::PointXYZ>) input_cloud, 
                                      pcl::deg2rad(horizontal_angle_res), pcl::deg2rad(vertical_angle_res),
                                      pcl::deg2rad(horizontal_fov), pcl::deg2rad(vertical_fov),
                                      scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
        range_image_widget.showRangeImage(*rangeImage);
        }
    }

    Eigen::Quaternionf q = Eigen::Quaternionf(scene_sensor_pose.rotation());
    Eigen::Vector3f v = scene_sensor_pose.translation();
    std::cout << "Rotation: " << std::endl << q.w() << std::endl << q.vec() <<std::endl;
    std::cout << "Translation: " << std::endl << v << std::endl;
}



