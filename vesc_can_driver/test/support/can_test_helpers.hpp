#pragma once
#include <vector>

#include "vesc_can_driver/can_protocol.hpp"
#include "vesc_can_driver/driver_core.hpp"
namespace vesc_can_driver
{
template <typename Controller>
bool receive_frame(Controller &controller, const can_msgs::msg::Frame &frame, TimePoint now)
{
  const auto feedback = protocol::decode_feedback(frame);
  return feedback && controller.receive(*feedback, now);
}
inline void make_test_frames(
  DriverCore &core, TimePoint now, std::vector<can_msgs::msg::Frame> &frames)
{
  core.commands(now, frames);
}
struct MotorFrameResult
{
  can_msgs::msg::Frame frame;
  std::string_view error;
};
inline MotorFrameResult motor_frame(MotorController &motor, TimePoint now)
{
  const auto result = motor.command(now);
  return {result.command, result.error};
}
inline can_msgs::msg::Frame stop_frame(const MotorController &motor)
{
  return motor.stop_command();
}
}  // namespace vesc_can_driver
