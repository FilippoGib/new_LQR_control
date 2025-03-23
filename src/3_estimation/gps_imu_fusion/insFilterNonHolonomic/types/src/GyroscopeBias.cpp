#include <InsFilterNonHolonomicTypes.hpp>

using namespace InsFilterNonHolonomicTypes;

GyroscopeBias::GyroscopeBias(double gyro_bias_x, double gyro_bias_y, double gyro_bias_z)
{
    this->gyro_bias_x = gyro_bias_x;
    this->gyro_bias_y = gyro_bias_y;
    this->gyro_bias_z = gyro_bias_z;
}
