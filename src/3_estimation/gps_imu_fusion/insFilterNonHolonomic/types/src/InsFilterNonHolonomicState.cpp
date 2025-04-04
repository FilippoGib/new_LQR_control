#include <InsFilterNonHolonomicState.hpp>

InsFilterNonHolonomicState::InsFilterNonHolonomicState(Quaternion* q_init, GyroscopeBias* gyro_bias_init, 
        NEDPosition* ned_position_init, NEDVelocities* ned_velocities_init, AccelerometerBias* accel_bias_init)
{
    this->q = q_init;
    this->gyro_bias = gyro_bias_init;
    this->ned_position = ned_position_init;
    this->ned_velocities = ned_velocities_init;
    this->accel_bias = accel_bias_init;
}

Quaternion InsFilterNonHolonomicState::getActualOrientation()
{
    return *this->q;
}
GyroscopeBias InsFilterNonHolonomicState::getActualGyroBias()
{
    return *this->gyro_bias;
}
NEDPosition InsFilterNonHolonomicState::getActualPosition()
{
    return *this->ned_position;
}
NEDVelocities InsFilterNonHolonomicState::getActualVelocities()
{
    return *this->ned_velocities;
}
AccelerometerBias InsFilterNonHolonomicState::getActualAccelBias()
{
    return *this->accel_bias;
}