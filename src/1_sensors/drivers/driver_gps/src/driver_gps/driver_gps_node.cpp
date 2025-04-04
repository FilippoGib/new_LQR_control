#include <driver_gps/driver_gps.hpp>

#include <unistd.h>

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing driver_camera process.\n";
        rclcpp::shutdown();
    }
}

int main(int argc, char* argv[])
{
  signal(SIGINT, handleSignal);
  /* node initialization */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DriverGPS>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;

}