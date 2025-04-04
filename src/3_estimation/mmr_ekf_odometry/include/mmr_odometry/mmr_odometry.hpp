#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

typedef Array<bool, 3, 1> config_vector_t;

class MmrOdometry
{
protected:
    config_vector_t config_vector;
};
