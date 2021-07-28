/*******************************************************************************************************************//**
 *  @file    planeDetector.cpp
 *  @brief   Provides an interface for a plane based cone segmentation algorithm.
 *
 *  @author  Riley Bowyer (riley.d.bowyer@gmail.com)
 *  @date    March 2021
 *
***********************************************************************************************************************/


// Main Include
#include "lidar_cones_detection/planeDetector.hpp"


/**
 * @brief Add a point to the bin. Recalculates parameters and adds
 *        the point to the stored vector.
 * @param point The point to add to the bin.
 *
 */
void PlaneDetector::Bin::add_point(pcl::PointXYZ point){
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
void PlaneDetector::Bin::reset(){
  points.clear();
  numPoints = 0;
  maxPoint = pcl::PointXYZ();
  minPoint = pcl::PointXYZ();
  containsDetection = false;
  
  obstacleHeight  = 0;
}


/**
 * @brief Default Constructor.
 *
 */
PlaneDetector::PlaneDetector(){
  
  this->maxHeight = 1;
  this->minPoints = 1;
  
  this->heightThreshold = 0.15;
  
  this->binCentreX = 0;
  this->binCentreY = 15;
  this->binWidth = 30;
  this->binHeight = 30;
  this->cellResolution = 0.5;
  
  // (int) round(width/RES)
  this->binImgWidth = (int) round(this->binWidth/this->cellResolution);
  this->binImgHeight = (int) round(this->binHeight/this->cellResolution); 
  
  this->binImage.resize(this->binImgWidth * this->binImgHeight);
  this->plane.setZero();
  
  for (auto &bin: this->binImage){
    bin = boost::shared_ptr<PlaneDetector::Bin>(new PlaneDetector::Bin);
    bin->reset();
  }  
  
  this->coneRadius = 0.2;
  this->pointPortion = 0.5;
  this->minConePoints = 5;
}


/**
 * @brief Get the height of the internally stored plane
 *        at the provided coordinate.
 * @param x The x-coordinate
 * @param y The y-coordinate
 * @return The z-coordinate of the plane
 *
 */
double PlaneDetector::get_height(double x, double y){
  return this->plane(0)*(x-this->binCentreX) + this->plane(1)*(y-this->binCentreY) + this->plane(2);
}


/**
 * @brief Converts an x,y,z coordinate to a bin index.
 * @param point The point to get an index for
 * @return The bin index
 *
 */
int PlaneDetector::point_to_index(pcl::PointXYZ& point){
  int xBin = (int) round(this->binImgWidth*(point.x+this->binWidth/2 - this->binCentreX)/this->binWidth);
  int yBin = (int) round(this->binImgHeight*(point.y+this->binHeight/2 - this->binCentreY)/this->binHeight);
  
  int bindex = xBin + yBin * this->binImgWidth;
  if (xBin < 0 || xBin > (this->binImgWidth - 1) || yBin < 0 || yBin > (this->binImgHeight-1)) bindex = -1;
  return bindex;
}


/**
 * @brief Main update function. Sorts the provided pointcloud into bins before
 *        returning a vector of identified cones.
 * @param pointCloud The pointcloud to sort and identify cones in.
 * @returns Vector of bins containing identified cone.
 *
 */
std::vector<boost::shared_ptr<PlaneDetector::Bin>> PlaneDetector::update(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud){
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
  
  // Construct 'A' Matrix
  Eigen::MatrixXd A(lsPointsBins.size(), 3);
  for (int i = 0; i < lsPointsBins.size(); i++)
    A.row(i) = Eigen::Vector3d(this->binImage[lsPointsBins[i]]->minPoint.x - this->binCentreX, this->binImage[lsPointsBins[i]]->minPoint.y - this->binCentreY, 1).transpose();
  
  // Construct 'b' Vector
  Eigen::VectorXd b(lsPointsBins.size());
  for (int i = 0; i < lsPointsBins.size(); i++)
    b[i] = this->binImage[lsPointsBins[i]]->minPoint.z;
  
  Eigen::MatrixXd AT = A.transpose();

  this->plane = (AT*A).ldlt().solve(AT*b);
  
  std::vector<int> detectedBins;
  
  // Loop Over bins and filter MaxPoints
  for (int i = 0; i < this->binImage.size(); ++i){
    if(this->binImage[i]->maxPoint.z - this->get_height(this->binImage[i]->maxPoint.x, this->binImage[i]->maxPoint.y) > this->heightThreshold &&
       this->binImage[i]->numPoints > this->minPoints) {
      this->binImage[i]->containsDetection = true;
      detectedBins.push_back(i);
    }
  }
  std::vector<int> visitedBins;
  std::vector<boost::shared_ptr<PlaneDetector::Bin>> clusterBins;
  
  // Need to -> Loop through all indexes, check surrounding bins -> append to current bin if contains detection + add neighbours to visiting list
  for (auto &i : detectedBins){
    if(std::find(visitedBins.begin(), visitedBins.end(), i) == visitedBins.end()){
      auto curBin = this->binImage[i];
      std::vector<int> toVisitBins;
      toVisitBins.push_back(i);
      while (toVisitBins.size() > 0){
        int indexCheck = toVisitBins[0];
        visitedBins.push_back(toVisitBins[0]);
        toVisitBins.erase(toVisitBins.begin());
        // Add all points from bin to curBin unless checking curBin 
        if (i != indexCheck) {
          for (auto point : this->binImage[indexCheck]->points){
            curBin->add_point(point);
          }
        }
        
        // Add Neighbouring Bins to check
        for (int nXBin = -1; nXBin < 2; nXBin++){
          for (int nYBin = -1; nYBin < 2; nYBin++){
            int cYBin = int(indexCheck/this->binImgWidth);
            int cXBin = indexCheck - cYBin * this->binImgWidth;
            
            int pXBin = (cXBin+nXBin);
            int pYBin = (cYBin+nYBin);
            
            int pIndex = pXBin + pYBin * this->binImgWidth;
            if (pYBin >= 0 && pYBin < (this->binImgHeight-1) && pXBin >= 0 && pXBin < this->binImgWidth && 
                this->binImage[pIndex]->containsDetection &&
                std::find(visitedBins.begin(), visitedBins.end(), pIndex) == visitedBins.end() && 
                std::find(toVisitBins.begin(), toVisitBins.end(), pIndex) == toVisitBins.end()) // Valid Index
              toVisitBins.push_back(pIndex);
          }
        }
      }
      clusterBins.push_back(this->binImage[i]);
    }
  }
  
  std::vector<boost::shared_ptr<PlaneDetector::Bin>> outputBins;
  
  for (auto &bin : clusterBins){
    int validPoints = 0;
    int totalPoints = 0;
    for (auto &point : bin->points){
      if (point.z - this->get_height(point.x, point.y) < this->heightThreshold/4.0) continue;
      if (sqrt(pow(bin->maxPoint.x-point.x, 2) + pow(bin->maxPoint.y-point.y, 2)) < this->coneRadius) {
        validPoints++;
      }
      totalPoints++;
    }
    
    if (validPoints/((float) totalPoints) > this->pointPortion && bin->numPoints > this->minConePoints)
      outputBins.push_back(bin);
  }
  
  
  return outputBins;
}