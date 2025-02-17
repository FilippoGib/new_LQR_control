#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>

#include <InsFilterNonHolonomic.hpp>

#include <fstream>

using namespace std;
using namespace Eigen;

class GpsImuFusion : public rclcpp::Node
{
private:
    /* Bool to check if filter is initialized */
    bool is_filter_orientation_initialized = false;
    bool is_filter_location_initialized = false;
    bool is_ref_location_set = false;
    
    /* Bool to understand if vehicle is still and so if filter doesn't need to be updated */
    bool is_vehicle_still = true;

    /* Velocity parameters */
    double total_dist = 0.0;
    double act_dist = 0.0;

    /* Sensors Frequency (Hz) */
    uint8_t imu_fs, gps_fs;

    /* Delta time for IMU samples */
    rclcpp::Time last_imu_sample;
    rclcpp::Duration *dt;

    /* TF publishing bound */
    int tf_required_topics = 5;

    /* Offset between Track and imu_link TFs */
    double tf_yaw_offset = -1.0;

    /* Bool for Logging */
    bool enable_logging = false;

    /* Noises Parameters */
    std::vector<double> gyroscope_noise;
    std::vector<double> gyroscope_bias_noise;
    std::vector<double> accelerometer_noise;
    std::vector<double> accelerometer_bias_noise;
    double r_vel = 0.0f;
    double r_pos = 0.005f;
    double gyroscope_bias_decay_factor = 1.0f;
    double accel_bias_decay_factor = 1.0f;
    double zero_velocity_constraint_noise = 1e-3;
    double gps_acc_unc = 0.005;

    /**
     * TODO: Create log files
     */
    uint16_t count_imu_topics = 0;
    uint16_t count_gps_topics = 0;
    uint16_t count_total_topic = 0;
    Vector3d prev_loc = Vector3d::Zero();
    ofstream predictFile;
    double vx = 0;
    double vy = 0;

    bool enable_tf = false;


    /* IMU, GPS, cones ROS 2 topics */
    string imu_topic, gps_topic, cones_topic;

    string odom_topic;

    /* IMU and GPS EKF */
    InsFilterNonHolonomic* gndFusion;

    /* IMU data subscriber */
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr cones_sub;

    /* GPS data subscriber */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;

    /* TF2 broadcaster */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /* Odometry publisher */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    /**
     * Load ROS 2 parameters function
    */
    void loadParameters();

    /**
     * IMU data callback
    */
    void imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data);

    void conesCallback(const visualization_msgs::msg::Marker::SharedPtr data);

    /**
     * GPS data callback
    */
    void gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_data);

    /**
     * Measure distance between two GPS messages
    */
    inline void measure_distance(double& dLat, double& dLon, double& d, Vector3d act_loc, Vector3d prev_loc)
    {
        double R = 6378.137; 
        dLat = act_loc[0] * M_PI / 180.0 - prev_loc[0] * M_PI / 180.0;
        dLon = act_loc[1] * M_PI / 180.0 - prev_loc[1] * M_PI / 180.0;
        double a = sin(dLat/2.0) * sin(dLat/2.0) + cos(prev_loc[0] * M_PI / 180) * cos(act_loc[0] * M_PI / 180.0) * sin(dLon/2.0) * sin(dLon/2.0);
        double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
        d = R * c;
    }

    /**
     * Measure azimuth between two GPS messages
    */
    inline void measure_azimuth(double& azimuth, Vector3d act_loc, Vector3d prev_loc)
    {
        azimuth = atan2(sin(act_loc[1] - prev_loc[1])*cos(act_loc[0]), (cos(prev_loc[0])*sin(act_loc[0])) - (sin(prev_loc[0]) * cos(act_loc[0])*cos(act_loc[1] - prev_loc[1])));
        if (azimuth < 0)
        {
            std::cerr << "Angle is < 0\n";
            azimuth += 2*M_PI;
        }
        if (azimuth > M_PI)
            azimuth = azimuth - (2*M_PI);

    }

    /**
     * From Global to Local frame
    */
    void inline globalToLocalFrame(tf2::Quaternion& q, Vector3d& act_position, const Vector4d act_orientation)
    {
        double roll, pitch, yaw;
        Quaterniond eq;

        q.setX(act_orientation[1]);
        q.setY(act_orientation[2]);
        q.setZ(act_orientation[3]);
        q.setW(act_orientation[0]);
        
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        yaw += -M_PI_2;

        if (this->count_gps_topics == this->tf_required_topics)
        {
            this->tf_yaw_offset = yaw;
        }

        /* Get rotation between earth and ROS 2 */
        eq = AngleAxisd(0.0, Vector3d::UnitX())
            * AngleAxisd(0.0, Vector3d::UnitY())
            * AngleAxisd(-tf_yaw_offset, Vector3d::UnitZ());

        /* Translate position */
        // act_position = eq * act_position;
        
        /* Update YAW */
        // q.setRPY(0, 0, -(yaw - this->tf_yaw_offset));
        q.setRPY(0, 0, -yaw);
    }

    /**
     * Update TF Pose
     * 
     * @param[in] p Vehicle position
     * @param[in] q Vehicle orientation
    */
    void updateTf(Vector3d p, tf2::Quaternion q, rclcpp::Time timestamp);

public:
    /* Default constructor */
    GpsImuFusion();
};