#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "KalmanFilter1D.h"
#include <mmr_base/msg/ecu_status.hpp>

class WheelOdom : public rclcpp::Node {
public:
WheelOdom() : Node("wheel_speed_to_odometry") {
        // read config from YAML
        this->declare_parameter<std::string>("generic.ecu_topic", "/status/ecu");
        this->declare_parameter<std::string>("generic.wheel_odom_topic", "/wheel_odom");
        this->declare_parameter<int>("generic.frequency", 100);
        this->declare_parameter<double>("generic.sensor_timeout", 0.2);
        this->declare_parameter<std::string>("debug.noise_odom_topic", "/odom/noise");

        std::string ecu_topic = this->get_parameter("generic.ecu_topic").as_string();
        std::string wheel_odom_topic = this->get_parameter("generic.wheel_odom_topic").as_string();
        // int frequency = this->get_parameter("generic.frequency").as_int();
        // double sensor_timeout = this->get_parameter("generic.sensor_timeout").as_double();
        std::string noise_odom_topic = this->get_parameter("debug.noise_odom_topic").as_string();

        wheel_speed_sub_ = this->create_subscription<mmr_base::msg::EcuStatus>(
            ecu_topic, 10, std::bind(&WheelOdom::ecu_cb, this, std::placeholders::_1));

        noise_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(noise_odom_topic, 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(wheel_odom_topic, 10);
        kalman_filter.setMeasurementError(0.1);
        kalman_filter.setEstimateError(0.1);
        kalman_filter.setProcessNoise(0.2);

        odom_msg.pose.pose.position.x = 0.0;
        odom_msg.pose.pose.position.y = 0.0;
        odom_msg.pose.pose.position.z = 0.0;
    }

private:
    rclcpp::Subscription<mmr_base::msg::EcuStatus>::SharedPtr wheel_speed_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noise_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_front_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_rear_;
    nav_msgs::msg::Odometry odom_msg;
    SimpleKalmanFilter kalman_filter;

    void ecu_cb(const mmr_base::msg::EcuStatus msg) {
        double front_speed = (msg.wheel_speed_front_left + msg.wheel_speed_front_right) / 2 / 3.6; // km/h to m/s
        double rear_speed = (msg.wheel_speed_rear_left + msg.wheel_speed_rear_right) / 2 / 3.6; // km/h to m/s

        double wheel_speed = (front_speed + rear_speed) / 2;

        double kalman_speed = kalman_filter.updateEstimate(wheel_speed);

        double steer_angle = (msg.steering_angle / 180.0) * M_PI; // degree to radian
        
        nav_msgs::msg::Odometry front_odom;
        nav_msgs::msg::Odometry rear_odom;

        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "track";
        odom_msg.child_frame_id = "base_link";
        
        odom_msg.twist.twist.linear.x = kalman_speed;

        static rclcpp::Time prev_time = this->now();
        double dt = (this->now() - prev_time).seconds();
        prev_time = this->now();
        
        // Calculate the new yaw angle
        static double previous_yaw = 0.0;
        double new_yaw = previous_yaw;
        if (kalman_speed > 0.05) // if the vehicle is standstill, don't update yaw
        {
            double new_yaw = previous_yaw + steer_angle * dt;
            previous_yaw = new_yaw;
        }

        // Update the orientation based on the new yaw angle
        odom_msg.pose.pose.orientation.z = sin(new_yaw / 2.0);
        odom_msg.pose.pose.orientation.w = cos(new_yaw / 2.0);

        // project position using yaw, previous position and speed
        double dx = kalman_speed * dt;
        odom_msg.pose.pose.position.x += dx * cos(steer_angle);
        odom_msg.pose.pose.position.y += dx * sin(steer_angle);

        // Set diagonal elements for pose covariance matrix
        odom_msg.pose.covariance[0] = 0.1;  // x position covariance
        odom_msg.pose.covariance[7] = 0.1;  // y position covariance
        odom_msg.pose.covariance[14] = 0.1; // z position covariance
        odom_msg.pose.covariance[21] = 0.1; // roll
        odom_msg.pose.covariance[28] = 0.1; // pitch
        odom_msg.pose.covariance[35] = 0.1; // yaw

        // Set diagonal elements for twist covariance matrix
        odom_msg.twist.covariance[0] = 0.1;  // x velocity covariance
        odom_msg.twist.covariance[7] = 0.1;  // y velocity covariance
        odom_msg.twist.covariance[14] = 0.1; // z velocity covariance
        odom_msg.twist.covariance[21] = 0.1; // roll
        odom_msg.twist.covariance[28] = 0.1; // pitch
        odom_msg.twist.covariance[35] = 0.1; // yaw

        odom_pub_->publish(odom_msg);
        // DEBUG: TODO, REMOVE!
        // publish odom with random gaussian noise
        static uint32_t seq = 0;
        random_noise_speed(odom_msg, seq);
        seq++;
    }

    void random_noise_speed(nav_msgs::msg::Odometry& source, uint32_t& seq) {
        nav_msgs::msg::Odometry noise_odom;
        noise_odom.header.stamp = this->now();
        noise_odom.header.frame_id = "odom";
        noise_odom.child_frame_id = "base_link";
        if (seq >= 300)
        {// add 20% noise to the speed
            noise_odom.twist.twist.linear.x = source.twist.twist.linear.x + source.twist.twist.linear.x * 0.3;
            seq = 0;
        }
        else // add random gaussian noise
            noise_odom.twist.twist.linear.x = source.twist.twist.linear.x + 0.01 * (rand() % 100 - 50);
        noise_odom.twist.covariance[0] = 0.1;  // x velocity covariance
        noise_odom.twist.covariance[7] = 0.1;  // y velocity covariance
        noise_odom.twist.covariance[14] = 0.1; // z velocity covariance
        noise_odom.twist.covariance[21] = 0.1; // roll
        noise_odom.twist.covariance[28] = 0.1; // pitch
        noise_odom.twist.covariance[35] = 0.1; // yaw
        for (size_t i = 0; i<36; i++) {
            noise_odom.pose.covariance[i] = 0.1;
            noise_odom.twist.covariance[i] = 0.1;
        }

        noise_odom_pub_->publish(noise_odom);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdom>());
    rclcpp::shutdown();
    return 0;
}
