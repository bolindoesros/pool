#include "loki_control/controller.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<loki::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
