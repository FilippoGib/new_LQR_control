#include <mmr_ekf_odometry/mmr_ekf_odometry.hpp>
#include <unistd.h>

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing mmr_ekf_odometry process.\n";
        rclcpp::shutdown();
    }
}


int main(int argc, char* argv[])
{
  signal(SIGINT, handleSignal);
  /* node initialization */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MmrEKFOdometry>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;

}