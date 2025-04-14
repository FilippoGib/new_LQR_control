#pragma once 
#include "cuda_clustering/filtering/ifiltering.hpp"

#include "cuda_runtime.h"

typedef enum {
    PASSTHROUGH = 0,
    VOXELGRID = 1,
} FilterType_t;

typedef struct {
    FilterType_t type;
    //0=x,1=y,2=z
    //type PASSTHROUGH
    int dim;
    float upFilterLimits;
    float downFilterLimits;
    bool limitsNegative;
    //type VOXELGRID
    float voxelX;
    float voxelY;
    float voxelZ;
} FilterParam_t;

#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}

class cudaFilter
{
public:
    cudaFilter(cudaStream_t stream = 0);
    ~cudaFilter(void);
    /*
    Input:
        source: data pointer for points cloud
        nCount: count of points in cloud_in
    Output:
        output: data pointer which has points filtered by CUDA
        countLeft: count of points in output
    */
    int set(FilterParam_t param);
    int filter(void *output, unsigned int *countLeft, void *source, unsigned int nCount);

    void *m_handle = NULL;
};

class CudaFilter : public IFilter
{
    private:
        FilterParam_t setP;
    public:
        CudaFilter();
        pcl::PointCloud<pcl::PointXYZ>::Ptr filterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc);
};