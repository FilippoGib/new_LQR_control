#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>

class SpeedEstimatorNode : public rclcpp::Node {
public:
    SpeedEstimatorNode() : Node("kalman_filter_node") {
        // Load parameters
        this->declare_parameter("process_noise", 0.1);
        this->declare_parameter("gps_noise", 2.0);
        this->declare_parameter("wheel_noise", 1.0);
        this->declare_parameter("initial_speed", 0.0);
        this->declare_parameter("gps_topic", "/gps/speed");
        this->declare_parameter("wheel_topic", "/wheel/speed");
        this->declare_parameter("estimated_speed_topic", "/speed/estimated");

        process_noise_ = this->get_parameter("process_noise").as_double();
        gps_noise_ = this->get_parameter("gps_noise").as_double();
        wheel_noise_ = this->get_parameter("wheel_noise").as_double();
        std::string gps_topic = this->get_parameter("gps_topic").as_string();
        std::string wheel_topic = this->get_parameter("wheel_topic").as_string();
        std::string estimated_speed_topic = this->get_parameter("estimated_speed_topic").as_string();

        // Kalman Filter Initialization
        x_ = Eigen::Matrix<double, 1, 1>(this->get_parameter("initial_speed").as_double());
        P_ = Eigen::Matrix<double, 1, 1>(1.0);
        Q_ = Eigen::Matrix<double, 1, 1>(process_noise_);

        H_ = Eigen::Matrix<double, 2, 1>(1.0, 1.0);
        R_ << gps_noise_, 0.0,
              0.0, wheel_noise_;

        // Subscribers
        gps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            gps_topic, 10, std::bind(&SpeedEstimatorNode::gps_callback, this, std::placeholders::_1));
        
        wheel_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            wheel_topic, 10, std::bind(&SpeedEstimatorNode::wheel_callback, this, std::placeholders::_1));

        // Publisher
        speed_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(estimated_speed_topic, 10);
    }

private:
    void gps_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        gps_speed_ = msg->twist.twist.linear.x;
        update();
    }

    void wheel_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        wheel_speed_ = msg->twist.twist.linear.x;
        update();
    }

    void update() {
        if (!gps_speed_ || !wheel_speed_) return;

        // Prediction Step
        P_ += Q_;

        // Update Step
        Eigen::Matrix<double, 2, 1> Z;
        Z << *gps_speed_, *wheel_speed_;

        Eigen::Matrix<double, 2, 2> S = H_ * P_ * H_.transpose() + R_;
        Eigen::Matrix<double, 1, 2> K = P_ * H_.transpose() * S.inverse();

        x_ += K * (Z - H_ * x_);
        P_ = (Eigen::Matrix<double, 1, 1>::Identity() - K * H_) * P_;

        // Publish estimated speed
        auto speed_msg = nav_msgs::msg::Odometry();
        speed_msg.twist.twist.linear.x = x_(0, 0);
        speed_pub_->publish(speed_msg);

        // Reset sensor readings
        gps_speed_.reset();
        wheel_speed_.reset();
    }

    // Kalman Filter Matrices
    Eigen::Matrix<double, 1, 1> x_, P_, Q_;
    Eigen::Matrix<double, 2, 1> H_;
    Eigen::Matrix<double, 2, 2> R_;

    // Sensor Data
    std::optional<double> gps_speed_;
    std::optional<double> wheel_speed_;

    // ROS Components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr speed_pub_;

    double process_noise_, gps_noise_, wheel_noise_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpeedEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}
