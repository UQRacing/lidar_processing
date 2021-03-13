/*******************************************************************************************************************//**
 *  @file    binDetector.cpp
 *  @brief   Provides an interface for a bin filtering based cone segmentation algorithm.
 *
 *  @author  Riley Bowyer (riley.d.bowyer@gmail.com)
 *  @date    March 2021
 *
***********************************************************************************************************************/


#ifndef LIDAR_CONES_DETECTION_BINDETECTOR_H
#define LIDAR_CONES_DETECTION_BINDETECTOR_H


// Local Files
#include "lidar_cones_detection/UQRPointCloud.hpp"

// Processing Files
#include <eigen3/Eigen/Eigen>


class BinDetector {
  public:
    /**
     * @brief Cone type enumerator. Provides simple lookup for
     *        assigning colour information to detected obstacles.
     *
     */
    enum ConeType{
      BLUE        = 0,
      YELLOW      = 1,
      ORANGE      = 2,
      ORANGE_BIG  = 3,
      UNKNOWN     = 4
    };


    /**
     * @brief Bin struct. Provides a data structure to sort large quantities of points into,
     *        presenting key information about each segment in worldspace.
     *
     */
    struct Bin{
      // Pointcloud storage
      std::vector<pcl::PointXYZ> points;
      pcl::PointXYZ maxPoint;
      pcl::PointXYZ minPoint;

      // Calculated Parameters
      double obstacleHeight;
      double minHeight;
      double maxHeight;
      int numPoints;

      // Reported Cone Colour
      ConeType colour;
      
      
    /**
     * @brief Add a point to the bin. Recalculates parameters and adds
     *        the point to the stored vector.
     * @param point The point to add to the bin.
     *
     */
      void add_point(pcl::PointXYZ point);
      
      
    /**
     * @brief Reset the bin. Clears all calculated parameters and
     *        clears the stored point vector.
     *
     */
      void reset();
    };
  
  
  public:
    /**
     * @brief Default Constructor.
     *
     */
    BinDetector();


    /**
     * @brief Default Destructor.
     *
     */
    ~BinDetector() = default;


    /**
     * @brief Main update function. Sorts the provided pointcloud into bins before
     *        returning a vector of identified cones.
     * @param pointCloud The pointcloud to sort and identify cones in.
     * @returns Vector of bins containing identified cone.
     *
     */
    std::vector<boost::shared_ptr<Bin>> update(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

  private:
    /**
     * @brief Converts an x,y,z coordinate to a bin index.
     * @param point The point to get an index for
     * @return The bin index
     *
     */
    int point_to_index(pcl::PointXYZ& point);
  
  
    std::vector<boost::shared_ptr<Bin>> binImage;
    
    double maxHeight;
    int minPoints;
    double heightThreshold;
    double correspondenceThreshold;
    bool useNeighbourError;
    
    int binImgWidth;
    int binImgHeight;
    double cellResolution;
    
    double binCentreX;
    double binCentreY;
    double binWidth;
    double binHeight;
    
};

#endif // LIDAR_CONES_DETECTION_BINDETECTOR_H