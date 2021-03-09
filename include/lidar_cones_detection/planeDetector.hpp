
#ifndef LIDAR_CONES_DETECTION_PLANEDETECTOR_H
#define LIDAR_CONES_DETECTION_PLANEDETECTOR_H

// Local Files
#include "lidar_cones_detection/UQRPointCloud.hpp"

// Processing Files
#include <eigen3/Eigen/Eigen>


class PlaneDetector {
  public:
    enum ConeType{
      BLUE        = 0,
      YELLOW      = 1,
      ORANGE      = 2,
      ORANGE_BIG  = 3,
      UNKNOWN     = 4
    };

    struct Bin{
      std::vector<pcl::PointXYZ> points;
      pcl::PointXYZ maxPoint;
      pcl::PointXYZ minPoint;

      double obstacleHeight;
      double minHeight;
      double maxHeight;
      
      int numPoints;
      ConeType colour;
      
      void add_point(pcl::PointXYZ point);
      void reset();
    };
  
  
  
  public:
    // constructor
    PlaneDetector();

    // default destructor
    ~PlaneDetector() = default;

    // deleted copy constructor
    PlaneDetector(const PlaneDetector&) = delete;

    // deleted copy operator
    PlaneDetector& operator=(PlaneDetector&) = delete;

    std::vector<boost::shared_ptr<Bin>> update(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

  private:
    int point_to_index(pcl::PointXYZ& point);
    
    double get_height(double x, double y);
  
    double error(int binIndex);
    
    void combine_bin();

    ConeType get_bin_colour(const int xBin,  const int yBin);
    
    std::vector<boost::shared_ptr<Bin>> binImage;
    
    Eigen::Vector3d plane;
    
    double maxHeight;
    int minPoints;
    double heightThreshold;
    double correspondenceThreshold;
    
    int binImgWidth;
    int binImgHeight;
    
    double binCentreX;
    double binCentreY;
    double binWidth;
    double binHeight;
    
};

#endif // LIDAR_CONES_DETECTION_PLANEDETECTOR_H