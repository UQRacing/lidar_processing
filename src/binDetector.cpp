/*******************************************************************************************************************//**
 *  @file    binDetector.cpp
 *  @brief   Provides an interface for a bin filtering based cone segmentation algorithm.
 *
 *  @author  Riley Bowyer (riley.d.bowyer@gmail.com)
 *  @date    March 2021
 *
***********************************************************************************************************************/


// Main Include
#include "lidar_cones_detection/binDetector.hpp"


/**
 * @brief Add a point to the bin. Recalculates parameters and adds
 *        the point to the stored vector.
 * @param point The point to add to the bin.
 *
 */
void BinDetector::Bin::add_point(pcl::PointXYZ point){
  points.push_back(point);
  numPoints += 1;
  
  if(point.z > maxPoint.z || (maxPoint.x == 0 &&  maxPoint.y == 0 && maxPoint.z == 0)){
    maxPoint = point;
  }
  if(point.z < minPoint.z || (minPoint.x == 0 &&  minPoint.y == 0 && minPoint.z == 0)){
    minPoint = point;
  }
}


/**
 * @brief Reset the bin. Clears all calculated parameters and
 *        clears the stored point vector.
 *
 */
void BinDetector::Bin::reset(){
  points.clear();
  numPoints = 0;
  maxPoint = pcl::PointXYZ();
  minPoint = pcl::PointXYZ();
  
  obstacleHeight  = 0;
}


/**
 * @brief Default Constructor.
 *
 */
BinDetector::BinDetector(){
  
  this->maxHeight = 1;
  this->minPoints = 1;
  this->useNeighbourError = false;
  
  this->heightThreshold = 0.2;
  this->correspondenceThreshold = 0.2;
  
  this->binCentreX = 15;
  this->binCentreY = 0;
  this->binWidth = 30;
  this->binHeight = 30;
  this->cellResolution = 0.5;
  
  // (int) round(width/RES)
  this->binImgWidth = (int) round(this->binWidth/this->cellResolution);
  this->binImgHeight = (int) round(this->binHeight/this->cellResolution); 
  
  this->binImage.resize(this->binImgWidth * this->binImgHeight);
  
  for (auto &bin: this->binImage){
    bin = boost::shared_ptr<BinDetector::Bin>(new BinDetector::Bin);
    bin->reset();
  }  
}


/**
 * @brief Converts an x,y,z coordinate to a bin index.
 * @param point The point to get an index for
 * @return The bin index
 *
 */
int BinDetector::point_to_index(pcl::PointXYZ& point){
  int xBin = (int) round(this->binImgHeight*(point.x+this->binHeight/2)/this->binHeight);
  int yBin = (int) round(this->binImgWidth*(point.y+this->binWidth/2)/this->binWidth);
  
  int bindex = yBin + xBin * this->binImgWidth;
  
  if (bindex < 0 || bindex >= this->binImgWidth * this->binImgHeight) bindex = -1;
  return bindex;
}


/**
 * @brief Main update function. Sorts the provided pointcloud into bins before
 *        returning a vector of identified cones.
 * @param pointCloud The pointcloud to sort and identify cones in.
 * @returns Vector of bins containing identified cone.
 *
 */
std::vector<boost::shared_ptr<BinDetector::Bin>> BinDetector::update(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud){
  for (auto &bin: this->binImage) bin->reset();

  // Sort Points
  std::vector<int> lsPointsBins;
  for (auto & point : pointCloud->points) {
    if(point.z > this->maxHeight || (point.x == 0 && point.y ==0 && point.z == 0))  continue;
    int binIndex = this->point_to_index(point);    
    if(binIndex == -1) continue;
    
    if(this->binImage[binIndex]->numPoints == (this->minPoints-1)) lsPointsBins.push_back(binIndex); 
    this->binImage[binIndex]->add_point(point);
  }
  
  std::vector<boost::shared_ptr<BinDetector::Bin>> outputBins;
  
  for(int index=0; index < this->binImgWidth*this->binImgHeight; index += 1){
    boost::shared_ptr<BinDetector::Bin> bin = this->binImage[index];

    if(bin->numPoints < this->minPoints) continue;
    if(this->useNeighbourError){      
      double diffCubed = 0.0;
      int totalPoints = 0;

      for(int neighbourX=-this->binImgHeight; neighbourX <= this->binImgHeight; neighbourX+=this->binImgHeight){
        for(int neighbourY=-1; neighbourY <= 1; neighbourY+=1){
          int neighbourIndex = index + neighbourX + neighbourY;
          if(neighbourIndex == index || neighbourIndex < 0 || neighbourIndex > this->binImgWidth*this->binImgHeight) continue;
          boost::shared_ptr<BinDetector::Bin> neighbourBin = this->binImage[neighbourIndex];
          double diff = bin->maxPoint.z - neighbourBin->maxPoint.z;
          diffCubed += (diff*diff*diff)*neighbourBin->numPoints;
          totalPoints += neighbourBin->numPoints;
        }
      }
      if(totalPoints > this->minPoints){
        bin->obstacleHeight = powf(diffCubed/totalPoints,1.0/3.0);
        if(fabs(bin->obstacleHeight) > this->heightThreshold) outputBins.push_back(bin);
      }
    }
    else{
      if(fabs(bin->maxPoint.z - bin->minPoint.z) > this->heightThreshold) outputBins.push_back(bin);
    }
  }
  
  return outputBins;
}