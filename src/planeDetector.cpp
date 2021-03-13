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
  
  obstacleHeight  = 0;
}


/**
 * @brief Default Constructor.
 *
 */
PlaneDetector::PlaneDetector(){
  
  this->maxHeight = 1;
  this->minPoints = 1;
  
  this->heightThreshold = 0.2;
  this->correspondenceThreshold = 0.2;
  
  this->binCentreX = 15;
  this->binCentreY = 0;
  this->binWidth = 30;
  this->binHeight = 30;
  
  // (int) round(width/RES)
  this->binImgWidth = (int) round(this->binWidth/0.5);
  this->binImgHeight = (int) round(this->binHeight/0.5); 
  
  this->binImage.resize(this->binImgWidth * this->binImgHeight);
  this->plane.setZero();
  
  for (auto &bin: this->binImage){
    bin = boost::shared_ptr<PlaneDetector::Bin>(new PlaneDetector::Bin);
    bin->reset();
  }  
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
  return this->plane(0)*(x-this->binCentreX) + this->plane(1)*y + this->plane(2);
}


/**
 * @brief Converts an x,y,z coordinate to a bin index.
 * @param point The point to get an index for
 * @return The bin index
 *
 */
int PlaneDetector::point_to_index(pcl::PointXYZ& point){
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
  
  std::vector<boost::shared_ptr<PlaneDetector::Bin>> outputBins;
  
  // Loop Over bins and filter MaxPoints
  for(auto &bin: this->binImage){
    if(bin->maxPoint.z - this->get_height(bin->maxPoint.x, bin->maxPoint.y) > this->heightThreshold)
      outputBins.push_back(bin);
  }
  
  return outputBins;
}