#pragma once

#include <cstdint>
#include "can_msgs/msg/frame.hpp"

namespace vesc_can_ros2_control::protocol
{
constexpr uint32_t SET_CURRENT_ID = 1;
constexpr uint32_t SET_RPM_ID = 3;
constexpr uint32_t SET_POSITION_ID = 4;
constexpr uint32_t STATUS_1_ID = 9;
constexpr uint32_t STATUS_4_ID = 16;
constexpr uint32_t STATUS_5_ID = 27;

struct Status1
{
  uint8_t controller_id{};
  int32_t erpm{};
  double current_a{};
  double duty_cycle{};
};
struct Status4
{
  uint8_t controller_id{};
  double temperature_fet_c{};
  double temperature_motor_c{};
  double input_current_a{};
  double position_deg{};
};
struct Status5
{
  uint8_t controller_id{};
  int32_t tachometer{};
  double input_voltage_v{};
};

can_msgs::msg::Frame make_set_rpm_frame(uint8_t id, int32_t erpm);
can_msgs::msg::Frame make_set_current_frame(uint8_t id, double current_a);
can_msgs::msg::Frame make_set_position_frame(uint8_t id, double position_deg);
bool decode_status_1(const can_msgs::msg::Frame & frame, Status1 & status);
bool decode_status_4(const can_msgs::msg::Frame & frame, Status4 & status);
bool decode_status_5(const can_msgs::msg::Frame & frame, Status5 & status);
}  // namespace vesc_can_ros2_control::protocol
