#include "gps_to_odom/gps_to_odom.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsToOdom>());
    rclcpp::shutdown();
    return 0;
}