#include <gps_imu_fusion/gps_imu_fusion.hpp>

int main(int argc, char* argv[])
{
  /* node initialization */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GpsImuFusion>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;

}