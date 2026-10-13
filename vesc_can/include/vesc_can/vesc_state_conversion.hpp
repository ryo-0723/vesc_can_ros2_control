#pragma once

#include "vesc_can/driver_core.hpp"
#include "vesc_msgs/msg/vesc_state.hpp"

namespace vesc_can
{
// Native STATUS frames do not include VESC fault codes or absolute tachometer counts.
constexpr std::int32_t kUnavailableVescInteger = -1;

// Convert native motor state to the upstream VescState message.
// Floating fields expire independently. Integer displacement keeps its last received
// count; consumers must check status_5_fresh in the accompanying diagnostics.
vesc_msgs::msg::VescState make_vesc_state(
  const MotorConfig &motor_config, const MotorState &motor_state, TimePoint current_time);
}  // namespace vesc_can
