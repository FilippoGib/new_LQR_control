#ifndef GPS_TO_ODOM_HPP
#define GPS_TO_ODOM_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <numeric>
#include <cmath>

class GpsToOdom : public rclcpp::Node
{
public:
    GpsToOdom();

private:
    void imuCallback(const sensor_msgs::msg::Imu imu_data);
    void gpsCallback_normal(const sensor_msgs::msg::NavSatFix::SharedPtr full_msg);

    std::string gps_topic_;
    std::string imu_topic_;
    std::string odom_topic_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_normal;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool calibrated_;
    int count_;
    int calib_count_;
    double origin_latitude_;
    double origin_longitude_;
    double initial_yaw_;
    std::vector<double> origin_latitudes_;
    std::vector<double> origin_longitudes_;
    double roll, pitch, yaw;
};

#endif