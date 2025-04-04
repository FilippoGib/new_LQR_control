#include <InsFilterNonHolonomicTypes.hpp>

using namespace InsFilterNonHolonomicTypes;

AccelerometerBias::AccelerometerBias(double accel_bias_x, double accel_bias_y, double accel_bias_z)
{
    this->accel_bias_x = accel_bias_x;
    this->accel_bias_y = accel_bias_y;
    this->accel_bias_z = accel_bias_z;
}