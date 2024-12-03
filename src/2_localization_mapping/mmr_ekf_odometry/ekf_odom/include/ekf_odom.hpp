#pragma once

#include <Eigen/Dense>
// #include <Eigen/Core>

#include "color_logic.hpp"

#include <iostream>

#define N_CONES 400           /* This constant is the initial cones number. */
#define INF 1e1       /* 1e10 Constant value to represent Infinite */

using namespace Eigen;

/**
 *  type defining the signature vector. A signature is way of indexing a particular landmark. In our use case
 * the landmark (cones) are identified by the INDEX in the state vector. This further vector is used to identify
 * the COLOR of the cone which is extremely important. Each element in this vector refers to an already mapped cone in the 
 * state vector with the same index
*/
typedef Eigen::Matrix<ColorLogic, N_CONES, 1> SignatureVector;

class EKFOdom
{
private:
    Eigen::VectorXf x_;     /* Robot state vector */
    Eigen::MatrixXf P_;     /* Covariance matrix */
    Eigen::Matrix2f Q_;     /* Process noise covariance */
    Eigen::Matrix3f R_;     /* Measurement covariance */
    Eigen::MatrixXf Fx_;    /* Convenience matrix used for mapping the 3D state vector to (3, 3N + 3)D during update step */
    Eigen::MatrixXf Fx_k;   /* Convenience matrix used for mapping the 3D state vector to (6,  3N + 3)D during correction step */
    SignatureVector s_;     /* Vector containing the signature for each landmark. In our case the signature is the COLOR of the cone. The ID for each color are defined in hedaer file: "color_logic.hpp" . */

    float act_vel = 0.0;       /* Actual vehicle velocity [m/s] */
    float act_ang_vel = 0.0;   /* Actual vehicle angular_velocity [rad/s] */

    size_t landmark_count = 0;  /* Counter of mapped landmarks */

    float max_new_cone_dist;   /* Max distance for creating a new cone */

    bool is_pose_initialized = false;  /* Bool to check if an initial pose has been set */

    bool is_first_lap_completed = false; /* Bool to understand if the first lap is completed. In that case, do not map any new cone */

public:
    EKFOdom(Vector2f process_noise, Vector3f measurement_noise, const float alpha);
    virtual ~EKFOdom();
    

    void predict(const float dt);

    /**
     * Correct filter state using measurement(s) z:
     * 
     * @param z: Measurement(s) of actual cones. This vector contains:
     *  - actual range from vehicle to the landmark;
     *  - actual bearing from vehicle to the landmark;
     *  - signature of the landmark;
     *  @param act_cones_detected: Number of actually observed landmarks
    */
    void correct(const Vector3f *z, const size_t act_cones_detected);
    
    VectorXf getState() const;
    MatrixXf getCovariance() const;
    Vector3f getPoseCovariance() const;
    Matrix2f getProcessNoiseCovariance() const;
    Matrix3f getMeasurementNoiseCovariance() const;
    MatrixXf getFx() const;
    size_t getActMappedLandmarks() const;
    SignatureVector getSignatures() const;

    void setFirstLapCompleted(const bool first_lap_completed);
    void setActVel(const float vel);
    void setActAngVel(const float ang_vel);
    void setPose(const Vector3f pose);
    void setPoseCovariance(const Vector3f pos_cov);


    inline float euclideanDistance(float x1, float x2, float y1, float y2) {
      return sqrt(((x2 - x1)*(x2 - x1)) + ((y2 - y1)*(y2 - y1)));
    }

    inline float normalizeAngle(float angle) {
      while(angle > M_PI)
        angle -= 2*M_PI;
      while(angle < -M_PI)
        angle += 2*M_PI;

      return angle;
    }

    inline float normalizeYaw(float yaw) {
      while(yaw > (2*M_PI))
      {
        yaw -= (2*M_PI);
      }
      while(yaw < -(2*M_PI))
      {
        yaw += (2*M_PI);
      }

      return yaw;
    }



};