#include "vesc_can_driver/motor_state_publisher.hpp"

#include "vesc_can_driver/vesc_state_conversion.hpp"

namespace vesc_can_driver
{
MotorStatePublisher::MotorStatePublisher(
  rclcpp::Node &node, const std::vector<MotorConfig> &configs)
  : configs_(configs)
{
  pubs_.reserve(configs_.size());
  for (const auto &cfg : configs_) {
    pubs_.push_back(node.create_publisher<vesc_msgs::msg::VescStateStamped>(
      "~/motors/" + cfg.name + "/state", 10));
  }
}

void MotorStatePublisher::publish(
  const std::vector<MotorState> &states, TimePoint now, const rclcpp::Time &stamp)
{
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    vesc_msgs::msg::VescStateStamped msg;
    msg.header.stamp = stamp;
    msg.state = make_vesc_state(configs_[idx], states[idx], now);
    pubs_[idx]->publish(msg);
  }
}
}  // namespace vesc_can_driver
