/**
 * @author Riley Bowyer
 * @date 23-07-2020
 *
 * @brief Online Cloud Segmenter source
 */

#include "lidar_cones_detection/OnlineSegmentation.hpp"

uqr::OnlineSegmenter::OnlineSegmenter(){
    nh.param<double>("/segmenter/focal_x", focal_x, 50);
    nh.param<double>("/segmenter/focal_y", focal_y, 50);
    nh.param<int>("/segmenter/height", height, 200);
    nh.param<int>("/segmenter/width", width, 200);
    nh.param<int>("/segmenter/x_offset", x_offset, 0);
    nh.param<int>("/segmenter/y_offset", y_offset, -100);

    centre_x = x_offset + (width/2);
    centre_y = y_offset + (height/2);

    nh.param<bool>("/segmenter/view_image", view_image, false);
    
    imagePub = nh.advertise<sensor_msgs::Image>("/UQR/segment/image", 10);
}

void uqr::OnlineSegmenter::segment(uqr::PointCloud input_cloud){
    uqr::MatrixXd depthImage = uqr::MatrixXd::Zero(width,height); // This is very costly (adds 4ms to what would be 2ms)
    pcl::PointCloud<pcl::PointXYZ> cloud(input_cloud);

    double max_depth = 1;
    
    for (int i=0; i<cloud.size();i++){
        // Note for those asking why I'm letting 'z' = 'x',         ...coordinate systems be wack.
        double z = cloud.points[i].x;
        double u = (cloud.points[i].y*focal_x) / z;
        double v = (cloud.points[i].z*focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);

        if (pixel_pos_x > (width-1)){
        pixel_pos_x = width -1;
        }
        else if( pixel_pos_x < 0){
        pixel_pos_x = 0;
        }
        if (pixel_pos_y > (height-1)){
        pixel_pos_y = height-1;
        }
        else if( pixel_pos_y < 0){
        pixel_pos_y = 0;
        }
        
        // This has the potential to overwrite image data, but that is something we'll have to live with for the moment
        depthImage(pixel_pos_x,pixel_pos_y) = i+1;
        
        if(view_image){
            double norm = sqrt(pow(cloud.points[i].x,2)+pow(cloud.points[i].y,2)+pow(cloud.points[i].z,2));
            // ROS_INFO("X: %d Y: %d Value: %0.2f",pixel_pos_x,pixel_pos_y,norm);
            if(norm > max_depth){
                max_depth = norm;
            }
        }
    }
    if(view_image){ 
        double rangeScale = 255/max_depth;
        sensor_msgs::Image output_image;
        output_image.encoding = "mono8";
        output_image.header.stamp = ros::Time::now();
        output_image.height = height;
        output_image.width = width;
        output_image.step = width;
        output_image.is_bigendian = false;
        for(int h=0; h < height;h++){
            for(int w=0; w < width;w++){
                int i = depthImage(w,h)-1;
                if(i>0){
                    double norm = sqrt(pow(cloud.points[i].x,2)+pow(cloud.points[i].y,2)+pow(cloud.points[i].z,2));
                    output_image.data.push_back((int)norm*rangeScale);
                    // output_image.data.push_back((int)(norm*255/max_depth));
                }
                else{
                    output_image.data.push_back(0);
                }
                
            }
        }
        imagePub.publish(output_image);
    }
}

void uqr::OnlineSegmenter::angle_column(){


}



