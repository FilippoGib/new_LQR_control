#include "gps_to_odom/gps_to_odom.hpp"

double normalizeAngle(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

GpsToOdom::GpsToOdom()
: Node("gps_to_odom"), calibrated_(false), count_(0)
{
    this->declare_parameter("topics.gps_data", "gps/fix");
    this->declare_parameter("topics.output_odom", "odom/gps");
    declare_parameter("calibration.count", 10);
    this->get_parameter("topics.gps_data", gps_topic_);
    this->get_parameter("topics.imu", imu_topic_);
    this->get_parameter("topics.output_odom", odom_topic_);
    this->get_parameter("calibration.count", calib_count_);

    // qos best effort
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    // gps_sub_ = this->create_subscription<common_msgs::msg::NavSatHeading>(
    //     gps_topic_, qos, std::bind(&GpsToOdom::gpsCallback, this, std::placeholders::_1));
    gps_sub_normal = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, qos, std::bind(&GpsToOdom::gpsCallback_normal, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, std::bind(&GpsToOdom::imuCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void GpsToOdom::imuCallback(const sensor_msgs::msg::Imu imu_data)
{
    tf2::Quaternion quat;
    tf2::fromMsg(imu_data.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "track";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = quat.x();
    odom_msg.pose.pose.orientation.y = quat.y();
    odom_msg.pose.pose.orientation.z = quat.z();
    odom_msg.pose.pose.orientation.w = quat.w();


    odom_pub_->publish(odom_msg);
    RCLCPP_INFO(this->get_logger(), "IMU Callback: Yaw: %f", yaw);
}

void GpsToOdom::gpsCallback_normal(const sensor_msgs::msg::NavSatFix::SharedPtr full_msg)
{
    auto msg = full_msg;
    if (!calibrated_)
    {
        if (count_ < calib_count_)
        {
        origin_latitudes_.push_back(msg->latitude);
        origin_longitudes_.push_back(msg->longitude);
        initial_yaw_ += yaw;
        count_++;
        if (count_ == calib_count_)
        {
            origin_latitude_ = std::accumulate(origin_latitudes_.begin(), origin_latitudes_.end(), 0.0) / origin_latitudes_.size();
            origin_longitude_ = std::accumulate(origin_longitudes_.begin(), origin_longitudes_.end(), 0.0) / origin_longitudes_.size();
            initial_yaw_ /= -calib_count_;
            calibrated_ = true;
            RCLCPP_INFO(this->get_logger(), "Calibration complete: Origin set. Yaw: %f", initial_yaw_);
        }
        return;
        }
    }

    double delta_lat = msg->latitude - origin_latitude_;
    double delta_lon = msg->longitude - origin_longitude_;

    // Simple conversion from degrees to meters (approximation)
    double x = delta_lon * 111320.0 * cos(origin_latitude_ * M_PI / 180.0);
    double y = delta_lat * 110540.0;

    double x_new = cos(initial_yaw_) * x - sin(initial_yaw_) * y;
    double y_new = sin(initial_yaw_) * x + cos(initial_yaw_) * y;

    yaw += initial_yaw_;
    yaw = normalizeAngle(yaw);

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);

    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "track";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = x_new;
    odom_msg.pose.pose.position.y = y_new;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = quat.x();
    odom_msg.pose.pose.orientation.y = quat.y();
    odom_msg.pose.pose.orientation.z = quat.z();
    odom_msg.pose.pose.orientation.w = quat.w();


    odom_pub_->publish(odom_msg);
}