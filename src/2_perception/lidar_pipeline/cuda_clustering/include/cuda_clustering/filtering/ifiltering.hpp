#pragma once 
#include <pcl_conversions/pcl_conversions.h>

class IFilter
{
    protected:
        
    public:
        virtual pcl::PointCloud<pcl::PointXYZ>::Ptr filterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc) = 0;
};