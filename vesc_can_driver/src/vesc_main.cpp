#include <exception>

#include "vesc_can_driver/vesc_node.hpp"

/// @brief 通常ノードを起動し、終了時に ROS を停止する
/// @return 正常終了で 0、ノード実行中の例外で 1
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<vesc_can_driver::VescNode>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
