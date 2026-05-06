#include "mbf_control.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MBFControl>());
  rclcpp::shutdown();
  return 0;
}