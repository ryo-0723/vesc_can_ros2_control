#include "vesc_can_ros2_control/vesc_protocol.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace vesc_can_ros2_control::protocol
{
namespace
{
can_msgs::msg::Frame command(uint8_t id, uint32_t packet, int32_t value)
{
  if (id == 255) {throw std::invalid_argument("Broadcast ID 255 is not supported");}
  can_msgs::msg::Frame frame{};
  frame.id = (packet << 8) | id;
  frame.is_extended = true;
  frame.dlc = 4;
  const auto raw = static_cast<uint32_t>(value);
  for (std::size_t i = 0; i < 4; ++i) {
    frame.data[i] = static_cast<uint8_t>(raw >> (24 - 8 * i));
  }
  return frame;
}

int32_t scaled_integer(double value, double scale)
{
  const double scaled = value * scale;
  if (!std::isfinite(scaled) || scaled < std::numeric_limits<int32_t>::min() ||
    scaled > std::numeric_limits<int32_t>::max())
  {
    throw std::out_of_range("CAN command is non-finite or exceeds int32 range");
  }
  return static_cast<int32_t>(std::llround(scaled));
}

bool valid(const can_msgs::msg::Frame & frame, uint32_t packet, uint8_t dlc)
{
  return frame.is_extended && !frame.is_rtr && !frame.is_error &&
         frame.id <= 0x1FFFFFFFU && frame.dlc == dlc && (frame.id >> 8) == packet;
}

int32_t read32(const can_msgs::msg::Frame & frame, std::size_t offset)
{
  uint32_t value = 0;
  for (std::size_t i = 0; i < 4; ++i) {value = (value << 8) | frame.data[offset + i];}
  return value <= 0x7FFFFFFFU ? static_cast<int32_t>(value) :
         -1 - static_cast<int32_t>(0xFFFFFFFFU - value);
}

int32_t read16(const can_msgs::msg::Frame & frame, std::size_t offset)
{
  const uint32_t value = (static_cast<uint32_t>(frame.data[offset]) << 8) |
    frame.data[offset + 1];
  return value <= 0x7FFFU ? static_cast<int32_t>(value) : static_cast<int32_t>(value) - 65536;
}
}  // namespace

can_msgs::msg::Frame make_set_rpm_frame(uint8_t id, int32_t erpm)
{
  return command(id, SET_RPM_ID, erpm);
}
can_msgs::msg::Frame make_set_current_frame(uint8_t id, double current_a)
{
  return command(id, SET_CURRENT_ID, scaled_integer(current_a, 1000.0));
}
can_msgs::msg::Frame make_set_position_frame(uint8_t id, double position_deg)
{
  return command(id, SET_POSITION_ID, scaled_integer(position_deg, 1000000.0));
}
bool decode_status_1(const can_msgs::msg::Frame & frame, Status1 & status)
{
  if (!valid(frame, STATUS_1_ID, 8)) {return false;}
  status = {static_cast<uint8_t>(frame.id & 0xFF), read32(frame, 0),
    read16(frame, 4) / 10.0, read16(frame, 6) / 1000.0};
  return true;
}
bool decode_status_4(const can_msgs::msg::Frame & frame, Status4 & status)
{
  if (!valid(frame, STATUS_4_ID, 8)) {return false;}
  status = {static_cast<uint8_t>(frame.id & 0xFF), read16(frame, 0) / 10.0,
    read16(frame, 2) / 10.0, read16(frame, 4) / 10.0, read16(frame, 6) / 50.0};
  return true;
}
bool decode_status_5(const can_msgs::msg::Frame & frame, Status5 & status)
{
  if (!valid(frame, STATUS_5_ID, 6)) {return false;}
  status = {static_cast<uint8_t>(frame.id & 0xFF), read32(frame, 0), read16(frame, 4) / 10.0};
  return true;
}
}  // namespace vesc_can_ros2_control::protocol
