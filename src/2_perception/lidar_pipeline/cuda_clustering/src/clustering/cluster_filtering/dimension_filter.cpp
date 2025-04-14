#include "cuda_clustering/clustering/cluster_filtering/dimension_filter.hpp"
DimensionFilter::DimensionFilter(){
    clusterMaxX = 0.2;
    clusterMaxY = 0.2;
    clusterMaxZ = 0.4;
    clusterMinX = 0.05;
    clusterMinY = 0.05;
    clusterMinZ = 0.1;
    maxHeight = 0.3;
}

std::optional<geometry_msgs::msg::Point> DimensionFilter::analiseCluster(float* outputPoints, unsigned int points_num){
    float x, y, z;

    double maxX=-1000, maxY=-1000, maxZ=-1000, minX=1000, minY=1000, minZ=1000;
    for(size_t k = 0; k < points_num; ++k)
    {
      x = outputPoints[k*4+0];
      y = outputPoints[k*4+1];
      z = outputPoints[k*4+2];

      if(x > maxX)
        maxX = x;
      if(y > maxY)
        maxY = y;
      if(z > maxZ)
        maxZ = z;
      if(x < minX)
        minX = x;
      if(y < minY)
        minY = y;
      if(z < minZ)
        minZ = z;
    }
    
    if(isCone(minX, maxX, minY, maxY, minZ, maxZ)){
      geometry_msgs::msg::Point pnt;
      pnt.x = (maxX + minX) / 2;
      pnt.y = (maxY + minY) / 2;
      pnt.z = (maxZ + minZ) / 2;
      
      return pnt;
    }
    else{
        return std::nullopt;
    }
}

bool DimensionFilter::isCone(float minX, float maxX, float minY, float maxY, float minZ, float maxZ){
    if(minZ < this->maxHeight &&
        (maxX - minX) < this->clusterMaxX &&
        (maxY - minY) < this->clusterMaxY &&
        (maxZ - minZ) < this->clusterMaxZ && 
        (maxX - minX) > this->clusterMinX &&
        (maxY - minY) > this->clusterMinY &&
        (maxZ - minZ) > this->clusterMinZ){
        
        return true;
    }
    return false;

}