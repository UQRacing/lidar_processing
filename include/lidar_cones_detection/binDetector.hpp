
#ifndef LIDAR_CONES_DETECTION_BINDETECTOR_H
#define LIDAR_CONES_DETECTION_BINDETECTOR_H

// Local Files
#include "lidar_cones_detection/UQRPointCloud.hpp"


class BinDetector {
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
    BinDetector();

    // default destructor
    ~BinDetector() = default;

    // deleted copy constructor
    BinDetector(const BinDetector&) = delete;

    // deleted copy operator
    BinDetector& operator=(BinDetector&) = delete;

    std::vector<Bin> update(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

  private:
    int point_to_index(pcl::PointXYZ& point);
  
    double error(int binIndex);
    
    void combine_bin();

    ConeType get_bin_colour(const int xBin,  const int yBin);
    
    std::vector<Bin> binImage;
    
    bool useNeighbourError;
    
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

#endif // LIDAR_CONES_DETECTION_BINDETECTOR_H