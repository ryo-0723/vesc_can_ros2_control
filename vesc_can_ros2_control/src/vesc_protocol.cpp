#include "vesc_can_ros2_control/vesc_protocol.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace vesc_can_ros2_control::protocol
{
namespace
{
constexpr std::size_t kCommandPayloadSize = 4;
constexpr double kCurrentCommandScale = 1000.0;
constexpr double kPositionCommandScale = 1000000.0;

can_msgs::msg::Frame make_command_frame(
  std::uint8_t controller_id, std::uint32_t packet_id, std::int32_t payload)
{
  if (controller_id == kBroadcastControllerId) {
    throw std::invalid_argument("Broadcast ID 255 is not supported");
  }

  can_msgs::msg::Frame frame;
  frame.id = (packet_id << kPacketIdShift) | controller_id;
  frame.is_extended = true;
  frame.dlc = kCommandPayloadSize;
  const auto payload_bits = static_cast<std::uint32_t>(payload);
  for (std::size_t byte_index = 0; byte_index < kCommandPayloadSize; ++byte_index) {
    const auto bit_shift = 8 * (kCommandPayloadSize - byte_index - 1);
    frame.data[byte_index] = static_cast<std::uint8_t>(payload_bits >> bit_shift);
  }
  return frame;
}

std::int32_t scale_command_value(double value, double scale)
{
  const double scaled_value = value * scale;
  if (!std::isfinite(scaled_value) || scaled_value < std::numeric_limits<std::int32_t>::min() ||
      scaled_value > std::numeric_limits<std::int32_t>::max()) {
    throw std::out_of_range("CAN command is non-finite or exceeds int32 range");
  }
  return static_cast<std::int32_t>(std::llround(scaled_value));
}

bool is_valid_status_frame(
  const can_msgs::msg::Frame &frame, std::uint32_t packet_id, std::uint8_t expected_dlc)
{
  return frame.is_extended && !frame.is_rtr && !frame.is_error && frame.id <= kExtendedCanIdMask &&
         frame.dlc == expected_dlc && (frame.id >> kPacketIdShift) == packet_id;
}

std::int32_t read_signed_32(const can_msgs::msg::Frame &frame, std::size_t offset)
{
  std::uint32_t payload_bits = 0;
  for (std::size_t byte_index = 0; byte_index < 4; ++byte_index) {
    payload_bits = (payload_bits << 8) | frame.data[offset + byte_index];
  }
  return std::bit_cast<std::int32_t>(payload_bits);
}

std::int32_t read_signed_16(const can_msgs::msg::Frame &frame, std::size_t offset)
{
  const auto payload_bits = static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(frame.data[offset]) << 8) | frame.data[offset + 1]);
  return std::bit_cast<std::int16_t>(payload_bits);
}
}  // namespace

can_msgs::msg::Frame make_set_rpm_frame(std::uint8_t controller_id, std::int32_t erpm)
{
  return make_command_frame(controller_id, kSetRpmId, erpm);
}

can_msgs::msg::Frame make_set_current_frame(std::uint8_t controller_id, double current_a)
{
  return make_command_frame(
    controller_id, kSetCurrentId, scale_command_value(current_a, kCurrentCommandScale));
}

can_msgs::msg::Frame make_set_position_frame(std::uint8_t controller_id, double position_deg)
{
  return make_command_frame(
    controller_id, kSetPositionId, scale_command_value(position_deg, kPositionCommandScale));
}

bool decode_status_1(const can_msgs::msg::Frame &frame, Status1 &status)
{
  if (!is_valid_status_frame(frame, kStatus1Id, 8)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.erpm = read_signed_32(frame, 0);
  status.current_a = read_signed_16(frame, 4) / 10.0;
  status.duty_cycle = read_signed_16(frame, 6) / 1000.0;
  return true;
}

bool decode_status_4(const can_msgs::msg::Frame &frame, Status4 &status)
{
  if (!is_valid_status_frame(frame, kStatus4Id, 8)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.temperature_fet_c = read_signed_16(frame, 0) / 10.0;
  status.temperature_motor_c = read_signed_16(frame, 2) / 10.0;
  status.input_current_a = read_signed_16(frame, 4) / 10.0;
  status.position_deg = read_signed_16(frame, 6) / 50.0;
  return true;
}

bool decode_status_2(const can_msgs::msg::Frame &frame, Status2 &status)
{
  if (!is_valid_status_frame(frame, kStatus2Id, 8)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.charge_drawn_ah = read_signed_32(frame, 0) / 10000.0;
  status.charge_regen_ah = read_signed_32(frame, 4) / 10000.0;
  return true;
}

bool decode_status_3(const can_msgs::msg::Frame &frame, Status3 &status)
{
  if (!is_valid_status_frame(frame, kStatus3Id, 8)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.energy_drawn_wh = read_signed_32(frame, 0) / 10000.0;
  status.energy_regen_wh = read_signed_32(frame, 4) / 10000.0;
  return true;
}

bool decode_status_5(const can_msgs::msg::Frame &frame, Status5 &status)
{
  // Official firmware sends eight bytes (the last two are reserved).
  // Also accept the existing six-byte form containing the complete known payload.
  if (!is_valid_status_frame(frame, kStatus5Id, 8) &&
      !is_valid_status_frame(frame, kStatus5Id, 6)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.tachometer = read_signed_32(frame, 0);
  status.input_voltage_v = read_signed_16(frame, 4) / 10.0;
  return true;
}

bool decode_status_6(const can_msgs::msg::Frame &frame, Status6 &status)
{
  if (!is_valid_status_frame(frame, kStatus6Id, 8)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.adc_1_v = read_signed_16(frame, 0) / 1000.0;
  status.adc_2_v = read_signed_16(frame, 2) / 1000.0;
  status.adc_3_v = read_signed_16(frame, 4) / 1000.0;
  status.ppm = read_signed_16(frame, 6) / 1000.0;
  return true;
}
}  // namespace vesc_can_ros2_control::protocol
