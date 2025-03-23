#pragma once

#include <InsFilterNonHolonomicTypes.hpp>

using namespace InsFilterNonHolonomicTypes;

class InsFilterNonHolonomicState
{
    private:
    /* Definition of the initial states of the filter (16 variables) */

    /* Quaternion defining the orientation of the system */
    Quaternion* q;

    /* Gyroscope Bias */
    GyroscopeBias* gyro_bias;

    /* NED Position */
    NEDPosition* ned_position;

    /* NED Velocities */
    NEDVelocities* ned_velocities;

    /* Accelerometer Bias */
    AccelerometerBias* accel_bias;

    public:

    /* Constructor */
    InsFilterNonHolonomicState(Quaternion* q_init, GyroscopeBias* gyro_bias_init, 
        NEDPosition* ned_position_init, NEDVelocities* ned_velocities_init, AccelerometerBias* accel_bias_init);

    Quaternion getActualOrientation();
    GyroscopeBias getActualGyroBias();
    NEDPosition getActualPosition();
    NEDVelocities getActualVelocities();
    AccelerometerBias getActualAccelBias();

    void setActualOrientation();
    void setActualGyroBias();
    void setActualPosition();
    void setActualVelocities();
    void setActualAccelBias();
};
