#include <rclcpp/rclcpp.hpp>
#include <common_msgs/msg/nav_sat_heading.hpp>

class GPSVisualizer : public rclcpp::Node
{
public:
    GPSVisualizer() : Node("gps_visualizer")
    {
        RCLCPP_INFO(this->get_logger(), "GPS Visualizer Node has been started.");
        // subscribe to /gps/data
        gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        // best effort
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        gps_subscriber_ = this->create_subscription<common_msgs::msg::NavSatHeading>(
            "/gps/data", qos, [this](const common_msgs::msg::NavSatHeading::SharedPtr msg) {
                sensor_msgs::msg::NavSatFix gps_data;
                gps_data.header = msg->gps_data.header;
                gps_data.latitude = msg->gps_data.latitude;
                gps_data.longitude = msg->gps_data.longitude;
                gps_data.altitude = msg->gps_data.altitude;
                gps_data.position_covariance = msg->gps_data.position_covariance;
                gps_data.position_covariance_type = msg->gps_data.position_covariance_type;
                gps_publisher_->publish(gps_data);
            });
    }
private:
    rclcpp::Subscription<common_msgs::msg::NavSatHeading>::SharedPtr gps_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}