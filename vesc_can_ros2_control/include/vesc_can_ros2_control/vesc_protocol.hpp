#pragma once

#include <cstdint>

#include "can_msgs/msg/frame.hpp"

namespace vesc_can_ros2_control::protocol
{
constexpr std::uint8_t kBroadcastControllerId = 255;
constexpr std::uint32_t kControllerIdMask = 0xFF;
constexpr std::uint32_t kPacketIdShift = 8;
constexpr std::uint32_t kExtendedCanIdMask = 0x1FFFFFFF;

constexpr std::uint32_t kSetCurrentId = 1;
constexpr std::uint32_t kSetRpmId = 3;
constexpr std::uint32_t kSetPositionId = 4;
constexpr std::uint32_t kStatus1Id = 9;
constexpr std::uint32_t kStatus2Id = 14;
constexpr std::uint32_t kStatus3Id = 15;
constexpr std::uint32_t kStatus4Id = 16;
constexpr std::uint32_t kStatus5Id = 27;
constexpr std::uint32_t kStatus6Id = 58;

struct Status1
{
  std::uint8_t controller_id{};
  std::int32_t erpm{};
  double current_a{};
  double duty_cycle{};
};
struct Status4
{
  std::uint8_t controller_id{};
  double temperature_fet_c{};
  double temperature_motor_c{};
  double input_current_a{};
  double position_deg{};
};
struct Status2
{
  std::uint8_t controller_id{};
  double charge_drawn_ah{};
  double charge_regen_ah{};
};
struct Status3
{
  std::uint8_t controller_id{};
  double energy_drawn_wh{};
  double energy_regen_wh{};
};
struct Status5
{
  std::uint8_t controller_id{};
  std::int32_t tachometer{};
  double input_voltage_v{};
};
struct Status6
{
  std::uint8_t controller_id{};
  double adc_1_v{};
  double adc_2_v{};
  double adc_3_v{};
  double ppm{};
};

can_msgs::msg::Frame make_set_rpm_frame(std::uint8_t controller_id, std::int32_t erpm);
can_msgs::msg::Frame make_set_current_frame(std::uint8_t controller_id, double current_a);
can_msgs::msg::Frame make_set_position_frame(std::uint8_t controller_id, double position_deg);
bool decode_status_1(const can_msgs::msg::Frame &frame, Status1 &status);
bool decode_status_2(const can_msgs::msg::Frame &frame, Status2 &status);
bool decode_status_3(const can_msgs::msg::Frame &frame, Status3 &status);
bool decode_status_4(const can_msgs::msg::Frame &frame, Status4 &status);
bool decode_status_5(const can_msgs::msg::Frame &frame, Status5 &status);
bool decode_status_6(const can_msgs::msg::Frame &frame, Status6 &status);
}  // namespace vesc_can_ros2_control::protocol
