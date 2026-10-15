#include "vesc_can_driver/can_protocol.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace vesc_can_driver::protocol
{
namespace
{
constexpr std::size_t kCommandPayloadSize = 4;
constexpr std::uint8_t kStatusPayloadSize = 8;
constexpr double kDutyCommandScale = 100000.0;
constexpr double kCurrentCommandScale = 1000.0;
constexpr double kPositionCommandScale = 1000000.0;

/// @brief COMM コマンドを 1 枚の拡張 CAN short buffer に格納する
can_msgs::msg::Frame make_short_command(
  std::uint8_t controller_id, std::uint8_t command, bool reply)
{
  if (controller_id >= kReplyCanId) {
    throw std::invalid_argument("Short command requires a motor CAN ID in [0, 253]");
  }
  can_msgs::msg::Frame frame;
  frame.id = (kProcessShortBufferId << kPacketIdShift) | controller_id;
  frame.is_extended = true;
  frame.dlc = 3;
  frame.data[0] = kReplyCanId;
  frame.data[1] = reply ? 0 : 2;
  frame.data[2] = command;
  return frame;
}

/// @brief int32 ペイロードを big endian の拡張 CAN 指令へ格納する
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

/// @brief 指令値に倍率を掛け、範囲を検査して int32 に丸める
std::int32_t scale_command_value(double value, double scale)
{
  const double scaled_value = value * scale;
  if (!std::isfinite(scaled_value) || scaled_value < std::numeric_limits<std::int32_t>::min() ||
      scaled_value > std::numeric_limits<std::int32_t>::max()) {
    throw std::out_of_range("CAN command is non-finite or exceeds int32 range");
  }
  return static_cast<std::int32_t>(std::llround(scaled_value));
}

/// @brief パケット ID・拡張 ID・DLC・データフレーム条件を検査する
bool is_valid_frame(const can_msgs::msg::Frame &frame, std::uint32_t packet_id)
{
  return frame.is_extended && !frame.is_rtr && !frame.is_error && frame.id <= kExtendedCanIdMask &&
         frame.dlc == kStatusPayloadSize && (frame.id >> kPacketIdShift) == packet_id;
}

/// @brief 指定バイト位置から big endian の符号付き 32 bit 値を読む
std::int32_t read_signed_32(const can_msgs::msg::Frame &frame, std::size_t offset)
{
  std::uint32_t payload_bits = 0;
  for (std::size_t byte_index = 0; byte_index < 4; ++byte_index) {
    payload_bits = (payload_bits << 8) | frame.data[offset + byte_index];
  }
  return std::bit_cast<std::int32_t>(payload_bits);
}

/// @brief 指定バイト位置から big endian の符号付き 16 bit 値を読む
std::int32_t read_signed_16(const can_msgs::msg::Frame &frame, std::size_t offset)
{
  const auto payload_bits = static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(frame.data[offset]) << 8) | frame.data[offset + 1]);
  return std::bit_cast<std::int16_t>(payload_bits);
}
}  // namespace

can_msgs::msg::Frame make_fault_request(std::uint8_t controller_id)
{
  auto frame = make_short_command(controller_id, kGetValuesSelectiveId, true);
  frame.dlc = 7;
  frame.data[5] = 0x80;  // Big-endian mask: bit 15 selects fault_code.
  return frame;
}

can_msgs::msg::Frame make_reboot_frame(std::uint8_t controller_id)
{
  return make_short_command(controller_id, kCommRebootId, false);
}

bool decode_fault_reply(const can_msgs::msg::Frame &frame, FaultCode &fault)
{
  if (!is_valid_frame(frame, kProcessShortBufferId) ||
      (frame.id & kControllerIdMask) != kReplyCanId || frame.data[0] >= kReplyCanId ||
      frame.data[1] != 1 || frame.data[2] != kGetValuesSelectiveId || frame.data[3] != 0 ||
      frame.data[4] != 0 || frame.data[5] != 0x80 || frame.data[6] != 0) {
    return false;
  }
  fault = {frame.data[0], frame.data[7]};
  return true;
}

can_msgs::msg::Frame comm_can_set_duty(std::uint8_t controller_id, double duty)
{
  if (!std::isfinite(duty) || std::abs(duty) > 1.0) {
    throw std::out_of_range("Duty command must be finite and in [-1, 1]");
  }
  return make_command_frame(
    controller_id, kSetDutyId, scale_command_value(duty, kDutyCommandScale));
}

can_msgs::msg::Frame comm_can_set_rpm(std::uint8_t controller_id, std::int32_t electrical_rpm)
{
  return make_command_frame(controller_id, kSetRpmId, electrical_rpm);
}

can_msgs::msg::Frame comm_can_set_current(std::uint8_t controller_id, double motor_current_a)
{
  return make_command_frame(
    controller_id, kSetCurrentId, scale_command_value(motor_current_a, kCurrentCommandScale));
}

can_msgs::msg::Frame comm_can_set_pos(std::uint8_t controller_id, double motor_pid_position_deg)
{
  return make_command_frame(controller_id, kSetPositionId,
    scale_command_value(motor_pid_position_deg, kPositionCommandScale));
}

bool decode_status_1(const can_msgs::msg::Frame &frame, Status1 &status)
{
  if (!is_valid_frame(frame, kStatus1Id)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.electrical_rpm = read_signed_32(frame, 0);
  status.motor_current_a = read_signed_16(frame, 4) / 10.0;
  status.duty_cycle = read_signed_16(frame, 6) / 1000.0;
  return true;
}

bool decode_status_2(const can_msgs::msg::Frame &frame, Status2 &status)
{
  if (!is_valid_frame(frame, kStatus2Id)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.charge_drawn_ah = read_signed_32(frame, 0) / 10000.0;
  status.charge_regen_ah = read_signed_32(frame, 4) / 10000.0;
  return true;
}

bool decode_status_3(const can_msgs::msg::Frame &frame, Status3 &status)
{
  if (!is_valid_frame(frame, kStatus3Id)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.energy_drawn_wh = read_signed_32(frame, 0) / 10000.0;
  status.energy_regen_wh = read_signed_32(frame, 4) / 10000.0;
  return true;
}

bool decode_status_4(const can_msgs::msg::Frame &frame, Status4 &status)
{
  if (!is_valid_frame(frame, kStatus4Id)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.temperature_fet_c = read_signed_16(frame, 0) / 10.0;
  status.temperature_motor_c = read_signed_16(frame, 2) / 10.0;
  status.input_current_a = read_signed_16(frame, 4) / 10.0;
  status.motor_pid_position_deg = read_signed_16(frame, 6) / 50.0;
  return true;
}

bool decode_status_5(const can_msgs::msg::Frame &frame, Status5 &status)
{
  // VESC 6 sends eight bytes, including two reserved bytes that are not decoded.
  if (!is_valid_frame(frame, kStatus5Id)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.tachometer_counts = read_signed_32(frame, 0);
  status.input_voltage_v = read_signed_16(frame, 4) / 10.0;
  return true;
}

bool decode_status_6(const can_msgs::msg::Frame &frame, Status6 &status)
{
  if (!is_valid_frame(frame, kStatus6Id)) {
    return false;
  }
  status.controller_id = static_cast<std::uint8_t>(frame.id & kControllerIdMask);
  status.adc_1_v = read_signed_16(frame, 0) / 1000.0;
  status.adc_2_v = read_signed_16(frame, 2) / 1000.0;
  status.adc_3_v = read_signed_16(frame, 4) / 1000.0;
  status.ppm = read_signed_16(frame, 6) / 1000.0;
  return true;
}

can_msgs::msg::Frame encode_command(const VescCommand &command)
{
  switch (command.type) {
    case VescCommandType::Duty:
      return comm_can_set_duty(command.controller_id, command.value);
    case VescCommandType::Current:
      return comm_can_set_current(command.controller_id, command.value);
    case VescCommandType::Rpm:
      if (!std::isfinite(command.value) ||
          command.value < std::numeric_limits<std::int32_t>::min() ||
          command.value > std::numeric_limits<std::int32_t>::max() ||
          std::trunc(command.value) != command.value) {
        throw std::out_of_range("RPM command must be an int32 ERPM value");
      }
      return comm_can_set_rpm(command.controller_id, static_cast<std::int32_t>(command.value));
    case VescCommandType::Position:
      return comm_can_set_pos(command.controller_id, command.value);
    case VescCommandType::FaultRequest:
      return make_fault_request(command.controller_id);
    case VescCommandType::Reboot:
      return make_reboot_frame(command.controller_id);
  }
  throw std::invalid_argument("Unsupported VESC command type");
}

std::optional<VescFeedback> decode_feedback(const can_msgs::msg::Frame &frame)
{
  Status1 s1;
  Status2 s2;
  Status3 s3;
  Status4 s4;
  Status5 s5;
  Status6 s6;
  FaultCode fault;
  if (decode_status_1(frame, s1)) {
    return s1;
  }
  if (decode_status_2(frame, s2)) {
    return s2;
  }
  if (decode_status_3(frame, s3)) {
    return s3;
  }
  if (decode_status_4(frame, s4)) {
    return s4;
  }
  if (decode_status_5(frame, s5)) {
    return s5;
  }
  if (decode_status_6(frame, s6)) {
    return s6;
  }
  if (decode_fault_reply(frame, fault)) {
    return fault;
  }
  return std::nullopt;
}
}  // namespace vesc_can_driver::protocol
