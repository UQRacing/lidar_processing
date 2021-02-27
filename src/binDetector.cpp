
// Main Include
#include "lidar_cones_detection/binDetector.hpp"

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

void BinDetector::Bin::reset(){
  points.clear();
  numPoints = 0;
  maxPoint = pcl::PointXYZ();
  minPoint = pcl::PointXYZ();
  
  obstacleHeight  = 0;
}

BinDetector::BinDetector(){
  this->useNeighbourError = false;
  
  double groundLevel = -0.4;
  this->maxHeight = 1 + groundLevel;
  this->minPoints = 1;
  this->heightThreshold = 0.2;
  this->correspondenceThreshold = 0.2;
  
  this->binCentreX = 15;
  this->binCentreY = 0;
  this->binWidth = 50;
  this->binHeight = 50;
  
  // (int) round(width/RES)
  this->binImgWidth = (int) round(this->binWidth/0.5);
  this->binImgHeight = (int) round(this->binHeight/0.5); 
  
  this->binImage.resize(this->binImgWidth * this->binImgHeight);
  for (auto &bin: this->binImage) bin.reset();
}

int BinDetector::point_to_index(pcl::PointXYZ& point){
  int xBin = (int) round(this->binImgHeight*(point.x+this->binHeight/2)/this->binHeight);
  int yBin = (int) round(this->binImgWidth*(point.y+this->binWidth/2)/this->binWidth);
  
  int bindex = yBin + xBin * this->binImgWidth;
  
  if (bindex < 0 || bindex >= this->binImgWidth * this->binImgHeight) bindex = -1;
  return bindex;
}

std::vector<BinDetector::Bin> BinDetector::update(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud){
  for (auto &bin: this->binImage) bin.reset();
  
  for (auto & point : pointCloud->points) {
    if(point.z > this->maxHeight || (point.x == 0 && point.y ==0 && point.z == 0))  continue;
    int binIndex = this->point_to_index(point);
    if(binIndex == -1) continue;
    this->binImage[binIndex].add_point(point);
  }
  
  std::vector<int> obstacleVec;
  
  for(int index=0; index < this->binImgWidth*this->binImgHeight; index += 1){
    BinDetector::Bin& bin = this->binImage[index];

    if(bin.numPoints < this->minPoints) continue;
    if(this->useNeighbourError){      
      double diffCubed = 0.0;
      int totalPoints = 0;

      for(int neighbourX=-this->binImgHeight; neighbourX <= this->binImgHeight; neighbourX+=this->binImgHeight){
        for(int neighbourY=-1; neighbourY <= 1; neighbourY+=1){
          int neighbourIndex = index + neighbourX + neighbourY;
          if(neighbourIndex == index || neighbourIndex < 0 || neighbourIndex > this->binImgWidth*this->binImgHeight) continue;
          BinDetector::Bin& neighbourBin = this->binImage[neighbourIndex];
          double diff = bin.maxPoint.z - neighbourBin.maxPoint.z;
          diffCubed += (diff*diff*diff)*neighbourBin.numPoints;
          totalPoints += neighbourBin.numPoints;
        }
      }
      if(totalPoints > this->minPoints){
        bin.obstacleHeight = powf(diffCubed/totalPoints,1.0/3.0);
        if(fabs(bin.obstacleHeight) > this->heightThreshold) obstacleVec.push_back(index);
      }
    }
    else{
      if(fabs(bin.maxPoint.z - bin.minPoint.z) > this->heightThreshold) obstacleVec.push_back(index);
    }
  }
  
  std::vector<BinDetector::Bin> mergedBins;
  std::set<int> visitedBins;
  
  for(auto &index: obstacleVec){
    std::vector<int> searchVec;
    BinDetector::Bin cone;
    cone.reset();
    
    searchVec.push_back(index);
    while(searchVec.size() != 0){
      int index = searchVec.back();
      searchVec.pop_back();
      auto it = visitedBins.find(index);
      if(it != visitedBins.end()) continue;
      visitedBins.insert(index);
      
      BinDetector::Bin& bin = this->binImage[index];
      
      for(auto &point: bin.points){
        cone.add_point(point);
      }
      
      for(int neighbourX=-this->binImgHeight; neighbourX <= this->binImgHeight; neighbourX+=this->binImgHeight){
        for(int neighbourY=-1; neighbourY <= 1; neighbourY+=1){
          int neighbourIndex = index + neighbourX + neighbourY;
          if(neighbourIndex == index || neighbourIndex < 0 || neighbourIndex > this->binImgWidth*this->binImgHeight) continue;
          
          BinDetector::Bin& neighbourBin = this->binImage[neighbourIndex];
          if(fabs(neighbourBin.obstacleHeight) > this->heightThreshold){
            double binDist = powf(powf(neighbourBin.maxPoint.x - bin.maxPoint.x, 2) + 
                                  powf(neighbourBin.maxPoint.y - bin.maxPoint.y, 2), 1.0/2.0);
          
            if(binDist < this->correspondenceThreshold){
              searchVec.push_back(neighbourIndex);
              // obstacleVec.erase(neighbourIndex);
            }
          }
        }
      }
    }
    mergedBins.push_back(cone);
  }
  
  // TODO: Implement Me!
  // std::vector<BinDetector::Bin> coneBins;
  // for(auto &bin: mergedBins){
  //   std::vector<int> searchVec;
  //   BinDetector::Bin cone;
  //   cone.reset();
  //   // Estimate Colour From Intensity, Classify Cones, Perform Cylinder Filter
  // }
  return mergedBins;
}