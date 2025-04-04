#pragma once

#include <stdint.h>
#include <gps.h>
#include <InsFilterNonHolonomicState.hpp>
#include <InsFilterNonHolonomicTypes.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <iostream>

#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

using namespace Eigen;
using Eigen::MatrixXd;

class InsFilterNonHolonomic
{
private:

    /* IMU and GPS frequency */
    uint8_t imu_fs = 100, gps_fs = 10;

    /* Delta time from previous IMU sample */
    double dt;

    /* Initial system location [latitude longitude altitude]*/
    gps_fix_t reference_location;

    /* Decimation Factor used for applying kinematics constraints */
    uint8_t decimation_factor = 1;
    uint8_t applyConstraintCount = 0;


    /* Definition of the initial states of the filter (16 variables) */
    InsFilterNonHolonomicState* filter_state_old;
    MatrixXd filter_state;

    /** Initial error covariance.
     * This is defined as a 16x16 Identity Matrix with each element in the main diagonal having an 
     * inital value of 1e-e9.
    */
    MatrixXd state_covariance;

    /* Define sensors noise */
    Vector3d gyroscope_noise;                   /* Gyro noise */
    Vector3d gyroscope_bias_noise;              /* Gyro bias noise */
    double gyroscope_bias_decay_factor;          /* A decay factor of 0 models gyroscope bias as a white noise process. A decay factor of 1 models the gyroscope bias as a random walk process */
    Vector3d accelerometer_noise;               /* Accel noise */
    Vector3d accelerometer_bias_noise;          /* Accel bias noise */
    double accel_bias_decay_factor;              /* A decay factor of 0 models gyroscope bias as a white noise process. A decay factor of 1 models the gyroscope bias as a random walk process */
    double r_vel = 0.0f;                         /* Noise of measured velocity */
    double r_pos = 0.005f;                       /* Noise of measured position */

    /**
     * The dynamic model of the ground vehicle for this filter assumes there is
     * no side slip or skid during movement. This means that the velocity is 
     * constrained to only the forward body axis. The other two velocity axis 
     * readings are corrected with a zero measurement weighted by the 
     * |zero_velocity_constraint_noise| parameter.
    */ 
    double zero_velocity_constraint_noise = 1e-3;/* Velocity constraints noise in (m/s)^2 */

    /* TODO: SPECIFY PARAMETERS IN A .yaml FILE. And check that you read that correctly */

    /**
     * Predict forward the state estimate one time sample, based on current IMU data
     * 
     * @param[in] accel_data system accelerations around X,Y and Z axis in m/s^2
     * @param[in] gyro_data system accelerations around X,Y and Z axis in rad/s
    */
    void setState(Vector3d accel_data, Vector3d gyro_data);

    /**
     * Jacobian of process equations. Compute the Jacobian matrix F 
     * of the state transition function f(x) with respect to state x.
     * 
     * @param[in] accel_data system accelerations around X,Y and Z axis in m/s^2
     * @param[in] gyro_data system accelerations around X,Y and Z axis in rad/s
    */
    MatrixXd stateTransitionJacobianFcn(Vector3d accel_data, Vector3d gyro_data);

    /**
     * Compute jacobian for multiplicative process noise
     * The process noise Jacobian G for state vector x and multiplicative
     * process noise w is L* W * (L.') where:
     *      L = jacobian of update function f with respect to drive inputs
     *      W = covariance matrix of multiplicative process noise w.
    */
    MatrixXd processNoiseJacobianFcn();

    /**
     * Process noises covariance
    */
    MatrixXd processNoiseCovariance();

    /**
     * Process new StateCovariance
    */
    void setStateCovariance(MatrixXd F, MatrixXd U, MatrixXd G);


    MatrixXd measurementFcnKinematics();

    MatrixXd measurementJacobianFcnKinematics();

    MatrixXd measurementNoiseKinematics();

    /**
     * correct state estimates based on the kinematic constraints
    */
   void correctKinematics();

public:
    
    /* Base Constructor  */
    InsFilterNonHolonomic();

    /**
     * Load parameters inital noise parameters 
     * 
     * @param[in] gyroscope_noise Vector defining noise of the gyroscope for X, Y and Z axis
     * @param[in] gyroscope_bias_noise Vector defining BIAS of the noise of the gyroscope for X, Y and Z axis
     * @param[in] accelerometer_noise Vector defining noise of the accelerometer for X, Y and Z axis
     * @param[in] accelerometer_bias_noise Vector defining BIAS of the noise of the accelerometer for X, Y and Z axis
     * @param[in,optional] r_vel Noise of measured velocity
     * @param[in,optional] r_pos Noise of measured position
     * @param[in,optional] gyroscope_bias_decay_factor Gyroscope bias noise decay factor
     * @param[in,optional] accel_bias_decay_factor Accelerometer bias noise decay factor
     * @param[in,optional] zero_velocity_constraint_noise Noise for side slip
     * 
    */
    void loadParameters(Vector3d gyroscope_noise,
        Vector3d gyroscope_bias_noise,
        Vector3d accelerometer_noise,
        Vector3d accelerometer_bias_noise,
        double r_vel,
        double r_pos,
        double gyroscope_bias_decay_factor,
        double accel_bias_decay_factor,
        double zero_velocity_constraint_noise
    );


    /**
     * Set initial state of the filter
     * 
    */
    void setInitState(Vector4d q_init, 
        Vector3d gyro_bias_init, 
        Vector3d position_init, 
        Vector3d velocity_init, 
        Vector3d accel_bias_init);

    /**
     * Set reference location of the filter
    */
   void setRefLocation(double latitude, double longitude, double altitude);

    /**
     * Print filter constraints
    */
    void printFilterConstraints();

    /**
     * Print current state of the filter
    */
   void printCurrentState();

    /**
     * Print current noises of the filter
    */
    void printFilterNoises();

    /**
     * Print current state covariant of the filter
    */
    void printCurrentStateCovariance();

    /**
     *  Update Filter state by prediction, using gyroscope AND accelerometer data.
     * 
     * @param[in] accel_data system accelerations around X,Y and Z axis in m/s^2
     * @param[in] gyro_data system accelerations around X,Y and Z axis in rad/s
     * @param[in] delta_t delta_t with previous imu_sample
    */
    void predict(Vector3d accel_data, Vector3d gyro_data, double delta_time);

    /**
     * Get position in NED format
     * @param[in] lla new Latitude Longitude and Altitude
    */
    MatrixXd getPosInNEDFormat(Vector3d lla);

    /**
     * Measure course and course covariance
     * @param[in] vel Vx and Vy velocities
     * @param[in] vel_r Velocities noise
     * @param[out] course Calculated course
     * @param[out] couseR Calculated course noise
    */
    void velAndCovToCourseAndCov(Vector2d vel, Matrix3d vel_r, double &course, double &courseR);

    /**
     * Measurement function h(x) for filter state vector 
     * 4 measurements from GPS [posN, posE, posD, heading];
    */
    MatrixXd measurementFcnGPS();

    /**
     * Compute the jacobian H of measurement function h(x)
    */
    MatrixXd measurementJacobianFcnGPS();

    /**
     * 
    */
    void correctEqn(MatrixXd h,
        MatrixXd H,
        MatrixXd z,
        Matrix4d R
    );


    /**
     * correct filter state using GPS data.
     * 
     * @param[in] lla_position Position of GPS receiver in geodetic latitude, longitude, and altitude (LLA) format. Latitude and longitude are in degrees with north and east being positive. Altitude is in meters.
     * @param[in] velocity Velocity of the GPS receiver in the local NED coordinate system in m/s
    */
    void fusegps(Vector3d lla_position, Vector3d velocity);

    /**
     * Return current system position, orientation and velocity
     * 
     * @param[out] curr_position Position estimate expressed in the local coordinate system of the filter in meters
     * @param[out] curr_orientation Orientation estimate expressed in the local coordinate system of the filter 
     * @param[out] curr_velocity Velocity estimate expressed in the local coordinate system of the filter in m/s
    */
    void pose(Vector3d &curr_position, Vector4d &curr_orientation, 
        Vector3d &curr_velocity);

    /* Destructor */
    ~InsFilterNonHolonomic();
};