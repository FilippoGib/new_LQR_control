#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <common_msgs/msg/nav_sat_heading.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>

class GpsToOdom : public rclcpp::Node
{
public:
    GpsToOdom()
    : Node("gps_to_odom"), calibrated_(false), count_(0)
    {
        this->declare_parameter("topics.gps_data", "gps/fix");
        this->declare_parameter("topics.output_odom", "odom/gps");
        declare_parameter("calibration.count", 10);
        this->get_parameter("topics.gps_data", gps_topic_);
        this->get_parameter("topics.output_odom", odom_topic_);
        this->get_parameter("calibration.count", calib_count_);

        // qos best effort
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        gps_sub_ = this->create_subscription<common_msgs::msg::NavSatHeading>(
            gps_topic_, qos, std::bind(&GpsToOdom::gpsCallback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void gpsCallback(const common_msgs::msg::NavSatHeading::SharedPtr full_msg)
    {
        auto msg = full_msg->gps_data;
        if (!calibrated_)
        {
            if (count_ < calib_count_)
            {
                origin_latitudes_.push_back(msg.latitude);
                origin_longitudes_.push_back(msg.longitude);
                count_++;
                if (count_ == calib_count_)
                {
                    origin_latitude_ = std::accumulate(origin_latitudes_.begin(), origin_latitudes_.end(), 0.0) / origin_latitudes_.size();
                    origin_longitude_ = std::accumulate(origin_longitudes_.begin(), origin_longitudes_.end(), 0.0) / origin_longitudes_.size();
                    calibrated_ = true;
                    RCLCPP_INFO(this->get_logger(), "Calibration complete: Origin set.");
                }
                return;
            }
        }

        double delta_lat = msg.latitude - origin_latitude_;
        double delta_lon = msg.longitude - origin_longitude_;

        // Simple conversion from degrees to meters (approximation)
        double x = delta_lon * 111320.0 * cos(origin_latitude_ * M_PI / 180.0);
        double y = delta_lat * 110540.0;

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;

        odom_pub_->publish(odom_msg);

        // Broadcast the transform
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "odom";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    std::string gps_topic_;
    std::string odom_topic_;

    rclcpp::Subscription<common_msgs::msg::NavSatHeading>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool calibrated_;
    int count_;
    int calib_count_;
    double origin_latitude_;
    double origin_longitude_;
    std::vector<double> origin_latitudes_;
    std::vector<double> origin_longitudes_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsToOdom>());
    rclcpp::shutdown();
    return 0;
}