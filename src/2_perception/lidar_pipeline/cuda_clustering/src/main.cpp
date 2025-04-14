#include "cuda_clustering/controller_node.hpp"
#include <unistd.h>

void handleSignal(int signal) {
    if (signal == SIGINT) {
        std::cout << "Received SIGINT. Killing clustering_plane_finder_cpu process.\n";
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv) {
  signal(SIGINT, handleSignal);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ControllerNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
