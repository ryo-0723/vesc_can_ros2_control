#pragma once

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vesc_can_driver/driver_core.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

namespace vesc_can_driver
{
// Shared per-motor ROS output. Call from a ROS executor, outside hardware read/write.
class MotorStatePublisher
{
public:
  MotorStatePublisher(rclcpp::Node &node, const std::vector<MotorConfig> &configs);
  void publish(const std::vector<MotorState> &states, TimePoint now, const rclcpp::Time &stamp);

private:
  std::vector<MotorConfig> configs_;
  std::vector<rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr> pubs_;
};
}  // namespace vesc_can_driver
