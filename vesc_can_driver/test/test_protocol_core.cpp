#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "gtest/gtest.h"
#include "vesc_can_driver/vesc_driver.hpp"
namespace vesc_can_driver
{
namespace
{
using namespace std::chrono_literals;
MotorConfig make_motor_config(std::uint8_t controller_id = 11)
{
  MotorConfig cfg;
  cfg.name = "motor" + std::to_string(controller_id);
  cfg.motor_id = controller_id;
  cfg.controller_id = controller_id;
  cfg.command_timeout = 100ms;
  cfg.feedback_timeout = 200ms;
  return cfg;
}
can_msgs::msg::Frame make_status_1_frame(std::uint8_t controller_id = 11)
{
  can_msgs::msg::Frame frame;
  frame.id = (9u << 8) | controller_id;
  frame.is_extended = true;
  frame.dlc = 8;
  frame.data = {0, 0, 1, 164, 0, 15, 1, 244};
  return frame;  // 420 ERPM, 1.5 A, 0.5 duty
}
can_msgs::msg::Frame make_status_4_frame(std::uint8_t controller_id = 21)
{
  can_msgs::msg::Frame frame;
  frame.id = (16u << 8) | controller_id;
  frame.is_extended = true;
  frame.dlc = 8;
  frame.data = {1, 44, 1, 54, 0, 0, 17, 148};
  return frame;  // 30 C, 31 C, 90 degrees
}
can_msgs::msg::Frame make_status_5_frame(std::uint8_t controller_id = 11)
{
  can_msgs::msg::Frame frame;
  frame.id = (27u << 8) | controller_id;
  frame.is_extended = true;
  frame.dlc = 8;
  frame.data = {255, 255, 255, 214, 0, 240, 0, 0};
  return frame;  // -42 counts, 24 V
}

template <typename Status, auto Decode> bool decode_status_frame(const can_msgs::msg::Frame &frame)
{
  Status status;
  status.controller_id = 77;
  const bool decoded = Decode(frame, status);
  if (!decoded) {
    EXPECT_EQ(status.controller_id, 77);  // Rejected frames must not update the output.
  }
  return decoded;
}
TEST(Protocol, CommandsHaveNativeIdAndBigEndianSignedPayload)
{
  auto frame = protocol::comm_can_set_rpm(11, -123456);
  EXPECT_EQ(frame.id, 0x30Bu);
  EXPECT_TRUE(frame.is_extended);
  EXPECT_FALSE(frame.is_rtr);
  EXPECT_FALSE(frame.is_error);
  EXPECT_EQ(frame.dlc, 4);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{255, 254, 29, 192, 0, 0, 0, 0}));
  frame = protocol::comm_can_set_current(21, 1.5);
  EXPECT_EQ(frame.id, 0x115u);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{0, 0, 5, 220, 0, 0, 0, 0}));
  frame = protocol::comm_can_set_pos(21, 90.0);
  EXPECT_EQ(frame.id, 0x415u);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{5, 93, 74, 128, 0, 0, 0, 0}));
  EXPECT_THROW(protocol::comm_can_set_current(255, 0.0), std::invalid_argument);
  EXPECT_THROW(protocol::comm_can_set_current(11, kUnknownValue), std::out_of_range);
  EXPECT_THROW(protocol::comm_can_set_pos(11, 1e20), std::out_of_range);
}

TEST(Protocol, DutyCommandsUseVesc6SignedRatioAndRejectInvalidInputs)
{
  auto frame = protocol::comm_can_set_duty(41, 0.5);
  EXPECT_EQ(frame.id, 0x29u);
  EXPECT_TRUE(frame.is_extended);
  EXPECT_FALSE(frame.is_rtr);
  EXPECT_FALSE(frame.is_error);
  EXPECT_EQ(frame.dlc, 4);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{0, 0, 0xC3, 0x50, 0, 0, 0, 0}));
  frame = protocol::comm_can_set_duty(41, -0.25);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{0xFF, 0xFF, 0x9E, 0x58, 0, 0, 0, 0}));
  EXPECT_EQ(protocol::comm_can_set_duty(0, 1.0).data,
    (std::array<std::uint8_t, 8>{0, 1, 0x86, 0xA0, 0, 0, 0, 0}));
  EXPECT_EQ(protocol::comm_can_set_duty(254, -1.0).data,
    (std::array<std::uint8_t, 8>{0xFF, 0xFE, 0x79, 0x60, 0, 0, 0, 0}));
  EXPECT_EQ(protocol::comm_can_set_duty(41, 0.0).data, (std::array<std::uint8_t, 8>{}));
  for (double value : {1.01, -1.01, kUnknownValue, std::numeric_limits<double>::infinity(),
         -std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(protocol::comm_can_set_duty(41, value), std::out_of_range);
  }
  EXPECT_THROW(protocol::comm_can_set_duty(255, 0.5), std::invalid_argument);
}

TEST(Protocol, FaultQueryMatchesVesc6SelectiveShortBufferWireFormat)
{
  const auto request = protocol::make_fault_request(11);
  EXPECT_EQ(request.id, 0x80Bu);
  EXPECT_EQ(request.dlc, 7);
  EXPECT_TRUE(request.is_extended);
  EXPECT_FALSE(request.is_rtr);
  EXPECT_FALSE(request.is_error);
  EXPECT_EQ(request.data, (std::array<std::uint8_t, 8>{254, 0, 50, 0, 0, 128, 0, 0}));
  EXPECT_THROW(protocol::make_fault_request(255), std::invalid_argument);
  EXPECT_THROW(protocol::make_fault_request(254), std::invalid_argument);
  can_msgs::msg::Frame reply;
  reply.id = 0x8FE;  // Destination is the host; data[0] identifies the motor.
  reply.is_extended = true;
  reply.dlc = 8;
  reply.data = {11, 1, 50, 0, 0, 128, 0, 4};
  protocol::FaultCode fault;
  ASSERT_TRUE(protocol::decode_fault_reply(reply, fault));
  EXPECT_EQ(fault.controller_id, 11);
  EXPECT_EQ(fault.code, 4);
  for (std::uint8_t code : {0, 255}) {
    reply.data[7] = code;
    ASSERT_TRUE(protocol::decode_fault_reply(reply, fault));
    EXPECT_EQ(fault.code, code);  // Preserve newer firmware codes without narrowing the list.
  }
}

TEST(Protocol, RebootWrapsComm29InCanShortBufferWithoutBroadcast)
{
  const auto frame = protocol::make_reboot_frame(51);
  EXPECT_EQ(frame.id, 0x833u);
  EXPECT_TRUE(frame.is_extended);
  EXPECT_FALSE(frame.is_rtr);
  EXPECT_FALSE(frame.is_error);
  EXPECT_EQ(frame.dlc, 3);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{254, 2, 29, 0, 0, 0, 0, 0}));
  EXPECT_THROW(protocol::make_reboot_frame(255), std::invalid_argument);
  EXPECT_THROW(protocol::make_reboot_frame(254), std::invalid_argument);
}

TEST(Core, RebootResetInvalidatesSelectedFeedbackAndDropsEveryOldCommand)
{
  DriverCore core({make_motor_config(11), make_motor_config(12)});
  const auto now = TimePoint(1s);
  ASSERT_TRUE(core.receive(make_status_1_frame(11), now));
  ASSERT_TRUE(core.receive(make_status_1_frame(12), now));
  ASSERT_TRUE(core.activate(now));
  ASSERT_TRUE(core.set_command(0, 1.0, now));
  ASSERT_TRUE(core.set_command(1, 2.0, now));
  ASSERT_TRUE(core.receive_fault({12, 4}, now));
  EXPECT_FALSE(core.reset(2));
  EXPECT_TRUE(core.faulted());
  ASSERT_TRUE(core.reset(1));
  EXPECT_FALSE(core.faulted());
  EXPECT_FALSE(core.active());
  EXPECT_EQ(core.states()[1].fault_code, -1);
  EXPECT_FALSE(core.states()[1].status1_at.has_value());
  EXPECT_TRUE(core.states()[0].status1_at.has_value());
  EXPECT_FALSE(core.activate(now));
  ASSERT_TRUE(core.receive(make_status_1_frame(12), now));
  ASSERT_TRUE(core.activate(now));
  std::vector<can_msgs::msg::Frame> frames;
  core.command_frames(now, frames);
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
  }
}

TEST(Protocol, FaultReplyRejectsWrongAddressMaskCommandAndFrameFormat)
{
  can_msgs::msg::Frame valid;
  valid.id = 0x8FE;
  valid.is_extended = true;
  valid.dlc = 8;
  valid.data = {11, 1, 50, 0, 0, 128, 0, 4};
  const auto reject = [](const can_msgs::msg::Frame &frame) {
    protocol::FaultCode fault{77, 88};
    EXPECT_FALSE(protocol::decode_fault_reply(frame, fault));
    EXPECT_EQ(fault.controller_id, 77);
    EXPECT_EQ(fault.code, 88);
  };
  for (auto id : {0x8FDu, 0x8FFu}) {
    auto frame = valid;
    frame.id = id;
    reject(frame);
  }
  for (int byte : {0, 1, 2, 3, 4, 5, 6}) {
    auto frame = valid;
    frame.data[byte] ^= 0x01;
    if (byte == 0) {
      frame.data[byte] = 255;
    }
    reject(frame);
  }
  auto frame = valid;
  frame.data[0] = 254;
  reject(frame);
  for (int dlc : {0, 7, 9, 255}) {
    frame = valid;
    frame.dlc = dlc;
    reject(frame);
  }
  frame = valid;
  frame.is_extended = false;
  reject(frame);
  frame = valid;
  frame.is_rtr = true;
  reject(frame);
  frame = valid;
  frame.is_error = true;
  reject(frame);
  frame = valid;
  frame.id |= 0x20000000;
  reject(frame);
  frame = valid;
  frame.id = 0x9FE;
  reject(frame);
}

TEST(Core, DeviceFaultStopsAllAndRequiresHealthyReplyBeforeExplicitEnable)
{
  DriverCore core({make_motor_config(11), make_motor_config(12)});
  const auto start = TimePoint(1s);
  ASSERT_TRUE(core.receive(make_status_1_frame(11), start));
  ASSERT_TRUE(core.receive(make_status_1_frame(12), start));
  ASSERT_TRUE(core.activate(start));
  ASSERT_TRUE(core.set_command(0, 2.0, start));
  ASSERT_TRUE(core.set_command(1, 2.0, start));
  EXPECT_FALSE(core.receive_fault({99, 4}, start));
  EXPECT_FALSE(core.faulted());
  ASSERT_TRUE(core.receive_fault({12, 4}, start));
  EXPECT_EQ(core.states()[0].fault_code, -1);
  EXPECT_EQ(core.states()[1].fault_code, 4);
  EXPECT_TRUE(core.faulted());
  EXPECT_FALSE(core.active());
  EXPECT_FALSE(core.activate(start));
  std::vector<can_msgs::msg::Frame> frames;
  core.command_frames(start, frames);
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
  }
  const auto later = start + 1s;
  ASSERT_TRUE(core.receive(make_status_1_frame(11), later));
  ASSERT_TRUE(core.receive(make_status_1_frame(12), later));
  EXPECT_FALSE(core.activate(later));  // Expired fault evidence cannot make the motor healthy.
  ASSERT_TRUE(core.receive_fault({12, 0}, later));
  EXPECT_TRUE(core.faulted());
  EXPECT_FALSE(core.active());
  EXPECT_TRUE(core.activate(later));
  EXPECT_FALSE(core.faulted());
  core.command_frames(later, frames);
  EXPECT_EQ(frames[0].id >> 8, 1u);  // Old commands were cleared.
}
TEST(Protocol, StatusAndMalformedFrames)
{
  protocol::Status1 status_1;
  protocol::Status4 status_4;
  protocol::Status5 status_5;
  auto frame = make_status_1_frame();
  ASSERT_TRUE(protocol::decode_status_1(frame, status_1));
  EXPECT_EQ(status_1.electrical_rpm, 420);
  EXPECT_DOUBLE_EQ(status_1.motor_current_a, 1.5);
  EXPECT_DOUBLE_EQ(status_1.duty_cycle, 0.5);
  frame.data = {255, 255, 252, 24, 255, 133, 252, 24};
  ASSERT_TRUE(protocol::decode_status_1(frame, status_1));
  EXPECT_EQ(status_1.electrical_rpm, -1000);
  EXPECT_DOUBLE_EQ(status_1.motor_current_a, -12.3);
  EXPECT_DOUBLE_EQ(status_1.duty_cycle, -1.0);
  frame.data = {128, 0, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(protocol::decode_status_1(frame, status_1));
  EXPECT_EQ(status_1.electrical_rpm, std::numeric_limits<std::int32_t>::min());
  frame.is_extended = false;
  EXPECT_FALSE(protocol::decode_status_1(frame, status_1));
  frame = make_status_1_frame();
  frame.dlc = 7;
  EXPECT_FALSE(protocol::decode_status_1(frame, status_1));
  frame = make_status_1_frame();
  frame.is_error = true;
  EXPECT_FALSE(protocol::decode_status_1(frame, status_1));
  frame = make_status_1_frame();
  frame.is_rtr = true;
  EXPECT_FALSE(protocol::decode_status_1(frame, status_1));
  frame = make_status_1_frame();
  frame.id |= 0x20000000;
  EXPECT_FALSE(protocol::decode_status_1(frame, status_1));
  ASSERT_TRUE(protocol::decode_status_4(make_status_4_frame(), status_4));
  EXPECT_DOUBLE_EQ(status_4.temperature_fet_c, 30.0);
  EXPECT_DOUBLE_EQ(status_4.motor_pid_position_deg, 90.0);
  ASSERT_TRUE(protocol::decode_status_5(make_status_5_frame(), status_5));
  EXPECT_EQ(status_5.tachometer_counts, -42);
  EXPECT_DOUBLE_EQ(status_5.input_voltage_v, 24.0);
  frame = make_status_5_frame();
  frame.data[6] = 0xA5;
  frame.data[7] = 0x5A;
  ASSERT_TRUE(protocol::decode_status_5(frame, status_5));
  EXPECT_EQ(status_5.tachometer_counts, -42);  // Reserved bytes do not change known fields.
  frame.dlc = 6;
  EXPECT_FALSE(protocol::decode_status_5(frame, status_5));
  frame.dlc = 7;
  EXPECT_FALSE(protocol::decode_status_5(frame, status_5));
  frame.dlc = 9;
  EXPECT_FALSE(protocol::decode_status_5(frame, status_5));
}
TEST(Protocol, ChargeEnergyAndAnalogStatusFields)
{
  auto frame = make_status_1_frame();
  frame.id = (14u << 8) | 11;
  frame.data = {0, 0, 48, 212, 0, 0, 19, 136};  // 12500 and 5000, scale 1e4.
  protocol::Status2 status_2;
  ASSERT_TRUE(protocol::decode_status_2(frame, status_2));
  EXPECT_DOUBLE_EQ(status_2.charge_drawn_ah, 1.25);
  EXPECT_DOUBLE_EQ(status_2.charge_regen_ah, 0.5);
  frame.dlc = 7;
  EXPECT_FALSE(protocol::decode_status_2(frame, status_2));

  frame.id = (15u << 8) | 11;
  frame.dlc = 8;
  frame.data = {0, 1, 212, 192, 0, 0, 39, 16};  // 120000 and 10000, scale 1e4.
  protocol::Status3 status_3;
  ASSERT_TRUE(protocol::decode_status_3(frame, status_3));
  EXPECT_DOUBLE_EQ(status_3.energy_drawn_wh, 12.0);
  EXPECT_DOUBLE_EQ(status_3.energy_regen_wh, 1.0);
  frame.is_rtr = true;
  EXPECT_FALSE(protocol::decode_status_3(frame, status_3));

  frame.id = (58u << 8) | 11;
  frame.is_rtr = false;
  frame.data = {4, 76, 8, 152, 12, 228, 254, 12};  // 1100, 2200, 3300, -500.
  protocol::Status6 status_6;
  ASSERT_TRUE(protocol::decode_status_6(frame, status_6));
  EXPECT_DOUBLE_EQ(status_6.adc_1_v, 1.1);
  EXPECT_DOUBLE_EQ(status_6.adc_2_v, 2.2);
  EXPECT_DOUBLE_EQ(status_6.adc_3_v, 3.3);
  EXPECT_DOUBLE_EQ(status_6.ppm, -0.5);
  frame.is_extended = false;
  EXPECT_FALSE(protocol::decode_status_6(frame, status_6));
}

TEST(Protocol, Vesc6StatusesRequireEightByteExtendedDataFrames)
{
  using Decoder = bool (*)(const can_msgs::msg::Frame &);
  const std::array<std::pair<std::uint32_t, Decoder>, 6> decoders = {{
    {9u, decode_status_frame<protocol::Status1, protocol::decode_status_1>},
    {14u, decode_status_frame<protocol::Status2, protocol::decode_status_2>},
    {15u, decode_status_frame<protocol::Status3, protocol::decode_status_3>},
    {16u, decode_status_frame<protocol::Status4, protocol::decode_status_4>},
    {27u, decode_status_frame<protocol::Status5, protocol::decode_status_5>},
    {58u, decode_status_frame<protocol::Status6, protocol::decode_status_6>},
  }};
  for (const auto &[packet_id, decode] : decoders) {
    SCOPED_TRACE(packet_id);
    auto frame = make_status_1_frame(37);
    frame.id = (packet_id << 8) | 37u;
    ASSERT_TRUE(decode(frame));
    for (int dlc : {0, 4, 6, 7, 9, 255}) {
      SCOPED_TRACE(dlc);
      frame.dlc = dlc;
      EXPECT_FALSE(decode(frame));
    }
    frame.dlc = 8;
    frame.is_extended = false;
    EXPECT_FALSE(decode(frame));
    frame.is_extended = true;
    frame.is_rtr = true;
    EXPECT_FALSE(decode(frame));
    frame.is_rtr = false;
    frame.is_error = true;
    EXPECT_FALSE(decode(frame));
    frame.is_error = false;
    frame.id = ((packet_id + 1) << 8) | 37u;
    EXPECT_FALSE(decode(frame));
    frame.id = (packet_id << 8) | 37u | 0x20000000u;
    EXPECT_FALSE(decode(frame));
  }
}

TEST(Protocol, Vesc6Signed32StatusFieldsKeepTheirUnitsAndBoundaries)
{
  auto frame = make_status_1_frame(37);
  frame.data = {0x7F, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0};
  protocol::Status1 status_1;
  ASSERT_TRUE(protocol::decode_status_1(frame, status_1));
  EXPECT_EQ(status_1.controller_id, 37);
  EXPECT_EQ(status_1.electrical_rpm, std::numeric_limits<std::int32_t>::max());

  frame.data = {0x80, 0, 0, 0, 0x7F, 0xFF, 0xFF, 0xFF};
  frame.id = (14u << 8) | 37u;
  protocol::Status2 status_2;
  ASSERT_TRUE(protocol::decode_status_2(frame, status_2));
  EXPECT_EQ(status_2.controller_id, 37);
  EXPECT_DOUBLE_EQ(status_2.charge_drawn_ah, -214748.3648);
  EXPECT_DOUBLE_EQ(status_2.charge_regen_ah, 214748.3647);

  frame.id = (15u << 8) | 37u;
  protocol::Status3 status_3;
  ASSERT_TRUE(protocol::decode_status_3(frame, status_3));
  EXPECT_EQ(status_3.controller_id, 37);
  EXPECT_DOUBLE_EQ(status_3.energy_drawn_wh, -214748.3648);
  EXPECT_DOUBLE_EQ(status_3.energy_regen_wh, 214748.3647);

  frame.id = (27u << 8) | 37u;
  frame.data = {0x80, 0, 0, 0, 0xFF, 0x85, 0xA5, 0x5A};
  protocol::Status5 status_5;
  ASSERT_TRUE(protocol::decode_status_5(frame, status_5));
  EXPECT_EQ(status_5.controller_id, 37);
  EXPECT_EQ(status_5.tachometer_counts, std::numeric_limits<std::int32_t>::min());
  EXPECT_DOUBLE_EQ(status_5.input_voltage_v, -12.3);
}

TEST(Protocol, Vesc6Signed16StatusFieldsKeepTheirOrderAndScale)
{
  auto frame = make_status_1_frame(37);
  frame.id = (16u << 8) | 37u;
  frame.data = {0x80, 0, 0x7F, 0xFF, 0xFF, 0x85, 0xEE, 0x6C};
  protocol::Status4 status_4;
  ASSERT_TRUE(protocol::decode_status_4(frame, status_4));
  EXPECT_EQ(status_4.controller_id, 37);
  EXPECT_DOUBLE_EQ(status_4.temperature_fet_c, -3276.8);
  EXPECT_DOUBLE_EQ(status_4.temperature_motor_c, 3276.7);
  EXPECT_DOUBLE_EQ(status_4.input_current_a, -12.3);
  EXPECT_DOUBLE_EQ(status_4.motor_pid_position_deg, -90.0);

  frame.id = (58u << 8) | 37u;
  frame.data = {0xFC, 0x18, 0x7F, 0xFF, 0x80, 0, 0xFE, 0x0C};
  protocol::Status6 status_6;
  ASSERT_TRUE(protocol::decode_status_6(frame, status_6));
  EXPECT_EQ(status_6.controller_id, 37);
  EXPECT_DOUBLE_EQ(status_6.adc_1_v, -1.0);
  EXPECT_DOUBLE_EQ(status_6.adc_2_v, 32.767);
  EXPECT_DOUBLE_EQ(status_6.adc_3_v, -32.768);
  EXPECT_DOUBLE_EQ(status_6.ppm, -0.5);
}

TEST(Core, ConvertsJointUnitsAndUsesMeasuredTachometer)
{
  auto cfg = make_motor_config();
  cfg.gear_ratio = 2.0;
  cfg.direction = -1.0;
  EXPECT_NEAR(vel_to_erpm(cfg, kPi), -420.0, 1e-9);
  DriverCore core({cfg});
  const auto start_time = TimePoint(1s);
  ASSERT_TRUE(core.receive(make_status_1_frame(), start_time));
  EXPECT_NEAR(core.states()[0].vel_rad_s, -kPi, 1e-9);
  ASSERT_TRUE(core.receive(make_status_5_frame(), start_time));
  EXPECT_NEAR(core.states()[0].pos_rad, kPi, 1e-9);
  EXPECT_FALSE(core.receive(make_status_1_frame(99), start_time));
  cfg.mode = ControlMode::Position;
  cfg.zero_offset_rad = kPi / 2;
  EXPECT_NEAR(pos_to_deg(cfg, kPi / 4), 0.0, 1e-9);
  EXPECT_NEAR(deg_to_pos(cfg, 0.0), kPi / 4, 1e-9);
}

TEST(Core, TachometerConversionKeepsSignedMultiTurnJointAngles)
{
  auto cfg = make_motor_config();  // Seven pole pairs: 42 counts per motor revolution.
  EXPECT_DOUBLE_EQ(tacho_to_pos(cfg, 0), 0.0);
  EXPECT_NEAR(tacho_to_pos(cfg, 42), 2.0 * kPi, 1e-9);
  EXPECT_NEAR(tacho_to_pos(cfg, -84), -4.0 * kPi, 1e-9);
  cfg.gear_ratio = 2.0;
  cfg.direction = -1.0;
  cfg.zero_offset_rad = kPi / 2;  // The PID origin does not change the tachometer origin.
  EXPECT_NEAR(tacho_to_pos(cfg, -168), 4.0 * kPi, 1e-9);
  cfg.pole_pairs = 3.0;
  EXPECT_NEAR(tacho_to_pos(cfg, 72), -4.0 * kPi, 1e-9);
}

TEST(Core, NonPositionModesUseTachometerAndKeepAngleAcrossControlActivation)
{
  const auto now = TimePoint(1s);
  for (auto mode : {ControlMode::Current, ControlMode::Velocity, ControlMode::CurrentVelocity,
         ControlMode::DutyCycle}) {
    SCOPED_TRACE(command_interface(mode));
    auto cfg = make_motor_config();
    cfg.mode = mode;
    cfg.require_tachometer = true;
    if (mode == ControlMode::CurrentVelocity) {
      cfg.startup_current_a = 2.0;
    }
    DriverCore core({cfg});
    ASSERT_TRUE(core.receive(make_status_1_frame(), now));
    auto feedback = make_status_5_frame();
    feedback.data = {0, 0, 0, 84, 0, 240, 0, 0};  // Two complete motor revolutions.
    ASSERT_TRUE(core.receive(feedback, now));
    EXPECT_NEAR(core.states()[0].pos_rad, 4.0 * kPi, 1e-9);
    ASSERT_TRUE(core.activate(now));
    core.deactivate();
    ASSERT_TRUE(core.activate(now));
    EXPECT_NEAR(core.states()[0].pos_rad, 4.0 * kPi, 1e-9);
    ASSERT_TRUE(core.receive(make_status_4_frame(11), now));
    EXPECT_NEAR(core.states()[0].pos_rad, 4.0 * kPi, 1e-9);
    feedback.data = {255, 255, 255, 214, 0, 240, 0, 0};  // Reverse past the original zero.
    ASSERT_TRUE(core.receive(feedback, now));
    EXPECT_NEAR(core.states()[0].pos_rad, -2.0 * kPi, 1e-9);
  }
}

TEST(Core, ReceivesStatusWhileControlIsDisabledOrFaulted)
{
  auto cfg = make_motor_config();
  cfg.mode = ControlMode::Current;
  DriverCore core({cfg});
  const auto start = TimePoint(1s);
  EXPECT_FALSE(core.active());
  ASSERT_TRUE(core.receive(make_status_1_frame(), start));
  EXPECT_DOUBLE_EQ(core.states()[0].motor_current_a, 1.5);
  ASSERT_TRUE(core.activate(start));
  EXPECT_FALSE(core.set_command(0, kUnknownValue, start));
  ASSERT_TRUE(core.faulted());

  auto feedback = make_status_1_frame();
  feedback.data = {255, 255, 254, 92, 0, 25, 0, 0};  // -420 ERPM, 2.5 A.
  ASSERT_TRUE(core.receive(feedback, start + 1ms));
  ASSERT_TRUE(core.receive(make_status_5_frame(), start + 1ms));
  EXPECT_DOUBLE_EQ(core.states()[0].electrical_rpm, -420.0);
  EXPECT_DOUBLE_EQ(core.states()[0].motor_current_a, 2.5);
  EXPECT_DOUBLE_EQ(core.states()[0].input_voltage_v, 24.0);
  EXPECT_EQ(core.states()[0].status1_at, start + 1ms);
  EXPECT_TRUE(core.ready(start + 1ms));
  EXPECT_TRUE(core.faulted());
  EXPECT_FALSE(core.active());  // New feedback must not resume motion automatically.
}

TEST(Core, MultiMotorFeedbackAndCommandTimeoutLatchStopUntilExplicitEnable)
{
  DriverCore core({make_motor_config(11), make_motor_config(12)});
  const auto start_time = TimePoint(1s);
  core.receive(make_status_1_frame(11), start_time);
  EXPECT_FALSE(core.activate(start_time));
  core.receive(make_status_1_frame(12), start_time);
  ASSERT_TRUE(core.activate(start_time));
  ASSERT_TRUE(core.set_command(0, 1.0, start_time));
  ASSERT_TRUE(core.set_command(1, 2.0, start_time));
  std::vector<can_msgs::msg::Frame> command_frames;
  core.command_frames(start_time + 20ms, command_frames);
  ASSERT_EQ(command_frames.size(), 2u);
  EXPECT_EQ(command_frames[0].id >> 8, 3u);
  core.command_frames(start_time + 101ms, command_frames);
  EXPECT_TRUE(core.faulted());
  EXPECT_FALSE(core.active());
  for (const auto &frame : command_frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data[0] | frame.data[1] | frame.data[2] | frame.data[3], 0);
  }
  core.receive(make_status_1_frame(11), start_time + 102ms);
  core.receive(make_status_1_frame(12), start_time + 102ms);
  EXPECT_FALSE(core.set_command(0, 1.0, start_time + 102ms));
  EXPECT_TRUE(core.faulted());
  ASSERT_TRUE(core.activate(start_time + 102ms));
  EXPECT_FALSE(core.faulted());
  ASSERT_TRUE(core.set_command(0, 1.0, start_time + 102ms));
  ASSERT_TRUE(core.set_command(1, 1.0, start_time + 102ms));
  core.deactivate();
  core.command_frames(start_time + 103ms, command_frames);
  EXPECT_EQ(command_frames[1].id >> 8, 1u);
}
TEST(Core, StaleRequiredFeedbackInvalidCommandsAndStartupStopAll)
{
  auto cfg = make_motor_config();
  cfg.command_timeout = 1000ms;
  DriverCore core({cfg, make_motor_config(12)});
  const auto start_time = TimePoint(1s);
  core.receive(make_status_1_frame(11), start_time);
  core.receive(make_status_1_frame(12), start_time);
  ASSERT_TRUE(core.activate(start_time));
  core.receive(make_status_1_frame(12), start_time + 201ms);
  core.set_command(1, 1.0, start_time + 201ms);
  std::vector<can_msgs::msg::Frame> command_frames;
  core.command_frames(start_time + 201ms, command_frames);
  EXPECT_TRUE(core.faulted());
  EXPECT_EQ(command_frames[1].id >> 8, 1u);
  DriverCore invalid_command_driver({cfg});
  invalid_command_driver.receive(make_status_1_frame(), start_time);
  ASSERT_TRUE(invalid_command_driver.activate(start_time));
  EXPECT_FALSE(invalid_command_driver.set_command(0, kUnknownValue, start_time));
  EXPECT_TRUE(invalid_command_driver.faulted());
  ASSERT_TRUE(invalid_command_driver.activate(start_time));
  EXPECT_FALSE(invalid_command_driver.set_command(0, 1000.0, start_time));
  cfg.startup_current_a = 2.0;
  cfg.mode = ControlMode::CurrentVelocity;
  cfg.rpm_threshold = 100.0;
  cfg.startup_timeout = 50ms;
  DriverCore startup_driver({cfg});
  auto zero_speed_feedback = make_status_1_frame();
  zero_speed_feedback.data.fill(0);
  startup_driver.receive(zero_speed_feedback, start_time);
  ASSERT_TRUE(startup_driver.activate(start_time));
  ASSERT_TRUE(startup_driver.set_command(0, 10.0, start_time));
  startup_driver.command_frames(start_time + 10ms, command_frames);
  EXPECT_EQ(command_frames[0].id >> 8, 1u);
  EXPECT_EQ(command_frames[0].data[3], 208);
  ASSERT_TRUE(startup_driver.set_command(
    0, 10.0, start_time + 40ms));  // repeated commands must not extend startup timeout
  startup_driver.command_frames(start_time + 51ms, command_frames);
  EXPECT_TRUE(startup_driver.faulted());
  EXPECT_EQ(command_frames[0].data[3], 0);
}
TEST(Core, MixedVelocityAndPositionRequireIndependentStatusStreams)
{
  auto cfg = make_motor_config(21);
  cfg.mode = ControlMode::Position;
  DriverCore core({make_motor_config(), cfg});
  const auto start_time = TimePoint(1s);
  core.receive(make_status_1_frame(), start_time);
  core.receive(make_status_1_frame(21), start_time);
  EXPECT_FALSE(core.activate(start_time));
  core.receive(make_status_4_frame(), start_time);
  ASSERT_TRUE(core.activate(start_time));
  EXPECT_NEAR(core.states()[1].pos_rad, kPi / 2, 1e-9);
  core.set_command(0, 1.0, start_time);
  core.set_command(1, kPi / 2, start_time);
  std::vector<can_msgs::msg::Frame> command_frames;
  core.command_frames(start_time + 10ms, command_frames);
  ASSERT_EQ(command_frames.size(), 2u);
  EXPECT_EQ(command_frames[0].id >> 8, 3u);
  EXPECT_EQ(command_frames[1].id >> 8, 4u);
  cfg.gear_ratio = 20.0;
  EXPECT_THROW(DriverCore({cfg}), std::invalid_argument);
  auto duplicate_config = make_motor_config();
  EXPECT_THROW(DriverCore({duplicate_config, duplicate_config}), std::invalid_argument);
}

TEST(Core, ZeroVelocityReleasesCurrentWhileZeroPositionCommandsJointZero)
{
  auto steer = make_motor_config(21);
  steer.mode = ControlMode::Position;
  steer.zero_offset_rad = kPi / 2;
  DriverCore driver({make_motor_config(), steer});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(), start);
  driver.receive(make_status_1_frame(21), start);
  driver.receive(make_status_4_frame(), start);
  ASSERT_TRUE(driver.activate(start));
  ASSERT_TRUE(driver.set_command(0, 1.0, start));
  ASSERT_TRUE(driver.set_command(1, 0.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].id >> 8, 3u);
  EXPECT_EQ(frames[1].data, protocol::comm_can_set_pos(21, 90.0).data);
  ASSERT_TRUE(driver.set_command(0, 0.0, start + 11ms));
  driver.command_frames(start + 11ms, frames);
  EXPECT_EQ(frames[0].id, 0x10Bu);
  EXPECT_EQ(frames[0].data, (std::array<std::uint8_t, 8>{}));
  EXPECT_EQ(frames[1].id, 0x415u);
  EXPECT_TRUE(driver.active());
  driver.deactivate();
  driver.command_frames(start + 12ms, frames);
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
  }
}

TEST(Core, VelocityRampUsesJointUnitsAndMotorDirection)
{
  auto motor = make_motor_config();
  motor.gear_ratio = 2.0;
  motor.direction = -1.0;
  motor.vel_slew_rad_s2 = kPi;
  motor.command_timeout = 1000ms;
  motor.feedback_timeout = 1000ms;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(), start);
  ASSERT_TRUE(driver.activate(start));
  ASSERT_TRUE(driver.set_command(0, kPi, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 500ms, frames);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_rpm(11, -210).data);
  driver.command_frames(start + 1000ms, frames);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_rpm(11, -420).data);
}

TEST(Core, CurrentStartupTransitionsOnlyOnFeedbackInCommandDirection)
{
  auto motor = make_motor_config();
  motor.direction = -1.0;
  motor.mode = ControlMode::CurrentVelocity;
  motor.startup_current_a = 2.0;
  motor.rpm_threshold = 50.0;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  // Positive raw ERPM means negative joint velocity when direction=-1.
  driver.receive(make_status_1_frame(), start);
  ASSERT_TRUE(driver.activate(start));
  ASSERT_TRUE(driver.set_command(0, 10.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].id, 0x10Bu);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_current(11, -2.0).data);
  auto feedback = make_status_1_frame();
  feedback.data = {255, 255, 254, 92, 0, 0, 0, 0};  // -420 ERPM = +60 motor RPM.
  driver.receive(feedback, start + 20ms);
  driver.command_frames(start + 20ms, frames);
  EXPECT_EQ(frames[0].id, 0x30Bu);
  EXPECT_TRUE(driver.active());
  EXPECT_FALSE(driver.faulted());
}

TEST(Core, PositionRangeAndOptionalTachometerAreEnforced)
{
  auto motor = make_motor_config(21);
  motor.mode = ControlMode::Position;
  motor.gear_ratio = 2.0;
  motor.min_pos_rad = -1.0;
  motor.max_pos_rad = 1.0;
  EXPECT_TRUE(command_is_valid(motor, -1.0));
  EXPECT_TRUE(command_is_valid(motor, 1.0));
  EXPECT_FALSE(command_is_valid(motor, 1.01));
  EXPECT_FALSE(command_is_valid(motor, std::numeric_limits<double>::infinity()));
  auto invalid = motor;
  invalid.max_pos_rad = kPi;
  EXPECT_THROW(DriverCore({invalid}), std::invalid_argument);
  invalid = motor;
  invalid.startup_current_a = 1.0;
  EXPECT_THROW(DriverCore({invalid}), std::invalid_argument);

  auto drive = make_motor_config();
  drive.require_tachometer = true;
  DriverCore driver({drive, motor});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(), start);
  driver.receive(make_status_1_frame(21), start);
  driver.receive(make_status_4_frame(), start);  // 90 motor degrees = pi/4 joint rad.
  EXPECT_FALSE(driver.activate(start));
  driver.receive(make_status_5_frame(), start);
  ASSERT_TRUE(driver.activate(start));
  EXPECT_NEAR(driver.states()[1].pos_rad, kPi / 4, 1e-9);
  ASSERT_TRUE(driver.set_command(0, 1.0, start));
  EXPECT_FALSE(driver.set_command(1, 1.01, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  ASSERT_EQ(frames.size(), 2u);
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
  }
}

TEST(Core, ModeSelectionValidatesCommandInterfaces)
{
  EXPECT_EQ(resolve_control_mode("", "current"), ControlMode::Current);
  EXPECT_EQ(resolve_control_mode("", "velocity"), ControlMode::Velocity);
  EXPECT_EQ(resolve_control_mode("", "position"), ControlMode::Position);
  EXPECT_EQ(resolve_control_mode("", "duty_cycle"), ControlMode::DutyCycle);
  EXPECT_EQ(resolve_control_mode("duty_cycle", "duty_cycle"), ControlMode::DutyCycle);
  EXPECT_THROW(resolve_control_mode("duty_cycle", "velocity"), std::invalid_argument);
  EXPECT_EQ(resolve_control_mode("current_velocity", "velocity"), ControlMode::CurrentVelocity);
  EXPECT_THROW(resolve_control_mode("current_velocity", "current"), std::invalid_argument);
  EXPECT_THROW(resolve_control_mode("velocity", "position"), std::invalid_argument);
  EXPECT_THROW(parse_control_mode("effort"), std::invalid_argument);
  EXPECT_THROW(parse_control_mode("unknown"), std::invalid_argument);
  EXPECT_THROW(command_interface(static_cast<ControlMode>(123)), std::invalid_argument);
}

TEST(Core, CurrentCommandsUseAmpsDirectionAndSharedStops)
{
  auto current = make_motor_config(12);
  current.mode = ControlMode::Current;
  current.max_current_a = 3.0;
  current.direction = -1.0;
  current.gear_ratio = 2.0;
  current.require_tachometer = true;
  DriverCore driver({current, make_motor_config()});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(12), start);
  driver.receive(make_status_1_frame(), start);
  EXPECT_FALSE(driver.activate(start));
  driver.receive(make_status_5_frame(12), start);
  ASSERT_TRUE(driver.activate(start));
  EXPECT_NEAR(driver.states()[0].pos_rad, kPi, 1e-9);
  EXPECT_DOUBLE_EQ(apply_direction(current, driver.states()[0].motor_current_a), -1.5);
  std::vector<can_msgs::msg::Frame> frames;
  for (double command : {2.5, -2.5, 0.0}) {
    ASSERT_TRUE(driver.set_command(0, command, start));
    ASSERT_TRUE(driver.set_command(1, 1.0, start));
    driver.command_frames(start + 10ms, frames);
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].id, 0x10Cu);
    EXPECT_EQ(frames[0].data, protocol::comm_can_set_current(12, -command).data);
  }
  EXPECT_TRUE(command_is_valid(current, 3.0));
  EXPECT_TRUE(command_is_valid(current, -3.0));
  EXPECT_FALSE(command_is_valid(current, kUnknownValue));
  EXPECT_FALSE(command_is_valid(current, std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(driver.set_command(0, 3.01, start));
  driver.command_frames(start + 11ms, frames);
  EXPECT_TRUE(driver.faulted());
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
  }
}

TEST(Core, DutyUsesDirectionAndConfiguredLimitWithSharedStops)
{
  auto duty = make_motor_config(41);
  duty.mode = ControlMode::DutyCycle;
  duty.direction = -1.0;
  duty.gear_ratio = 4.0;  // Duty is unaffected by gearing or pole pairs.
  duty.max_duty = 0.5;
  auto current = make_motor_config(12);
  current.mode = ControlMode::Current;
  DriverCore driver({duty, current});
  const auto now = TimePoint(1s);
  ASSERT_TRUE(driver.receive(make_status_1_frame(41), now));
  ASSERT_TRUE(driver.receive(make_status_1_frame(12), now));
  ASSERT_TRUE(driver.activate(now));
  ASSERT_TRUE(driver.set_command(0, 0.25, now));
  ASSERT_TRUE(driver.set_command(1, 1.5, now));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(now, frames);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].id, 0x29u);
  EXPECT_EQ(frames[0].data, (std::array<std::uint8_t, 8>{0xFF, 0xFF, 0x9E, 0x58, 0, 0, 0, 0}));
  ASSERT_TRUE(driver.set_command(0, -0.5, now));
  driver.command_frames(now, frames);
  EXPECT_EQ(frames[0].data, (std::array<std::uint8_t, 8>{0, 0, 0xC3, 0x50, 0, 0, 0, 0}));
  ASSERT_TRUE(driver.set_command(0, 0.0, now));
  driver.command_frames(now, frames);
  EXPECT_EQ(frames[0].id, 0x129u);  // Zero releases current, just like velocity/current modes.
  EXPECT_EQ(frames[0].data, (std::array<std::uint8_t, 8>{}));
  EXPECT_TRUE(command_is_valid(duty, 0.5));
  EXPECT_TRUE(command_is_valid(duty, -0.5));
  EXPECT_FALSE(driver.set_command(0, 0.51, now));
  EXPECT_TRUE(driver.faulted());
  driver.command_frames(now, frames);
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
  }
}

TEST(Core, DutyLimitsRejectNonFiniteOrOutOfRangeConfiguration)
{
  auto duty = make_motor_config(41);
  duty.mode = ControlMode::DutyCycle;
  for (double limit : {0.0, -0.5, 1.01, kUnknownValue, std::numeric_limits<double>::infinity()}) {
    duty.max_duty = limit;
    EXPECT_THROW(validate_configs({duty}), std::invalid_argument);
  }
}

TEST(Core, FiveModesShareCommandTimeoutAndExplicitRecovery)
{
  auto current = make_motor_config(12);
  current.mode = ControlMode::Current;
  auto steer = make_motor_config(21);
  steer.mode = ControlMode::Position;
  auto hybrid = make_motor_config(31);
  hybrid.mode = ControlMode::CurrentVelocity;
  hybrid.startup_current_a = 2.0;
  hybrid.rpm_threshold = 100.0;
  auto duty = make_motor_config(41);
  duty.mode = ControlMode::DutyCycle;
  DriverCore driver({current, make_motor_config(), steer, hybrid, duty});
  const auto start = TimePoint(1s);
  for (auto id : {12, 11, 21, 31, 41}) {
    auto feedback = make_status_1_frame(id);
    feedback.data.fill(0);
    driver.receive(feedback, start);
  }
  driver.receive(make_status_4_frame(), start);
  ASSERT_TRUE(driver.activate(start));
  for (std::size_t i = 0; i < 5; ++i) {
    ASSERT_TRUE(driver.set_command(i, i == 3 ? 10.0 : (i == 4 ? 0.25 : 1.0), start));
  }
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  ASSERT_EQ(frames.size(), 5u);
  EXPECT_EQ(frames[0].id, 0x10Cu);
  EXPECT_EQ(frames[1].id, 0x30Bu);
  EXPECT_EQ(frames[2].id, 0x415u);
  EXPECT_EQ(frames[3].id, 0x11Fu);
  EXPECT_EQ(frames[4].id, 0x29u);
  driver.command_frames(start + 101ms, frames);
  ASSERT_TRUE(driver.faulted());
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
  }
  EXPECT_FALSE(driver.set_command(0, 1.0, start + 102ms));
  ASSERT_TRUE(driver.activate(start + 102ms));
  driver.command_frames(start + 103ms, frames);
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);  // Re-enabling does not reuse any mode's old command.
  }
}

TEST(Core, HybridRestartsOnReversalAndZeroButNotOnSpeedDrop)
{
  auto motor = make_motor_config();
  motor.mode = ControlMode::CurrentVelocity;
  motor.startup_current_a = 2.0;
  motor.rpm_threshold = 50.0;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(), start);  // Already rotating at 60 motor RPM.
  ASSERT_TRUE(driver.activate(start));
  ASSERT_TRUE(driver.set_command(0, 10.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  EXPECT_EQ(frames[0].id >> 8, 3u);
  auto stopped = make_status_1_frame();
  stopped.data.fill(0);
  driver.receive(stopped, start + 20ms);
  driver.command_frames(start + 20ms, frames);
  EXPECT_EQ(frames[0].id >> 8, 3u);
  ASSERT_TRUE(driver.set_command(0, -10.0, start + 21ms));
  driver.command_frames(start + 22ms, frames);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_current(11, -2.0).data);
  auto reversed = make_status_1_frame();
  reversed.data = {255, 255, 254, 92, 0, 0, 0, 0};
  driver.receive(reversed, start + 30ms);
  driver.command_frames(start + 31ms, frames);
  EXPECT_EQ(frames[0].id >> 8, 3u);
  ASSERT_TRUE(driver.set_command(0, 0.0, start + 32ms));
  driver.command_frames(start + 32ms, frames);
  EXPECT_EQ(frames[0].id >> 8, 1u);
  EXPECT_EQ(frames[0].data, (std::array<std::uint8_t, 8>{}));
  driver.receive(stopped, start + 33ms);
  ASSERT_TRUE(driver.set_command(0, -10.0, start + 33ms));
  driver.command_frames(start + 34ms, frames);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_current(11, -2.0).data);
}

TEST(Core, HybridTransitionNeverCommandsAboveConfiguredSpeedLimit)
{
  auto motor = make_motor_config();
  motor.mode = ControlMode::CurrentVelocity;
  motor.startup_current_a = 2.0;
  motor.rpm_threshold = 50.0;
  motor.max_vel_rad_s = 10.0;
  motor.vel_slew_rad_s2 = 1.0;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  auto feedback = make_status_1_frame();
  feedback.data = {0, 0, 17, 148, 0, 0, 0, 0};  // 4500 ERPM exceeds max_velocity.
  driver.receive(feedback, start);
  ASSERT_TRUE(driver.activate(start));
  ASSERT_TRUE(driver.set_command(0, 2.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  EXPECT_EQ(frames[0].id >> 8, 3u);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_rpm(11, 668).data);  // 9.99 rad/s.
}

TEST(Core, DirectVelocityReversalPreservesSlewRate)
{
  auto motor = make_motor_config();
  motor.vel_slew_rad_s2 = 10.0;
  motor.command_timeout = 1000ms;
  motor.feedback_timeout = 1000ms;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(), start);
  ASSERT_TRUE(driver.activate(start));
  ASSERT_TRUE(driver.set_command(0, 1.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 100ms, frames);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_rpm(11, 67).data);
  ASSERT_TRUE(driver.set_command(0, -1.0, start + 100ms));
  driver.command_frames(start + 150ms, frames);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_rpm(11, 33).data);
  driver.command_frames(start + 200ms, frames);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_rpm(11, 0).data);
  driver.command_frames(start + 250ms, frames);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_rpm(11, -33).data);
}

TEST(Core, EpochFeedbackAndCommandsHaveValidReceiveTimes)
{
  auto cfg = make_motor_config();
  cfg.mode = ControlMode::Current;
  cfg.feedback_timeout = 200ms;
  DriverCore driver({cfg});
  const auto epoch = TimePoint{};
  EXPECT_FALSE(driver.ready(epoch));
  EXPECT_FALSE(driver.states()[0].status1_at.has_value());
  auto malformed = make_status_1_frame();
  malformed.dlc = 7;
  EXPECT_FALSE(driver.receive(malformed, epoch));
  EXPECT_FALSE(driver.states()[0].status1_at.has_value());
  ASSERT_TRUE(driver.receive(make_status_1_frame(), epoch));
  EXPECT_TRUE(feedback_fresh(driver.states()[0].status1_at, epoch, 200ms));
  EXPECT_FALSE(driver.ready(epoch - 1ms));
  EXPECT_TRUE(driver.ready(epoch + 200ms));
  EXPECT_FALSE(driver.ready(epoch + 201ms));
  ASSERT_TRUE(driver.activate(epoch));
  ASSERT_TRUE(driver.set_command(0, 2.0, epoch));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(epoch + 100ms, frames);  // Timeout boundary is inclusive.
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].data, protocol::comm_can_set_current(11, 2.0).data);
  driver.command_frames(epoch + 101ms, frames);
  EXPECT_TRUE(driver.faulted());
  EXPECT_FALSE(driver.active());
  EXPECT_EQ(frames[0].data, (std::array<std::uint8_t, 8>{}));
}

TEST(Core, StartupFailureReplacesEarlierMotorsCommandsWithStops)
{
  auto hybrid = make_motor_config(31);
  hybrid.mode = ControlMode::CurrentVelocity;
  hybrid.startup_current_a = 2.0;
  hybrid.startup_timeout = 50ms;
  DriverCore driver({make_motor_config(), hybrid});
  const auto start = TimePoint(1s);
  ASSERT_TRUE(driver.receive(make_status_1_frame(), start));
  auto stopped = make_status_1_frame(31);
  stopped.data.fill(0);
  ASSERT_TRUE(driver.receive(stopped, start));
  ASSERT_TRUE(driver.activate(start));
  ASSERT_TRUE(driver.set_command(0, 1.0, start));
  ASSERT_TRUE(driver.set_command(1, 10.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].id >> 8, 3u);
  EXPECT_EQ(frames[1].data, protocol::comm_can_set_current(31, 2.0).data);
  driver.command_frames(start + 51ms, frames);
  EXPECT_TRUE(driver.faulted());
  EXPECT_EQ(driver.fault_reason(), hybrid.name + ": startup timeout");
  ASSERT_EQ(frames.size(), 2u);
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
  }
  ASSERT_TRUE(driver.activate(start + 60ms));
  ASSERT_TRUE(driver.set_command(0, 1.0, start + 60ms));
  driver.command_frames(start + 61ms, frames);
  EXPECT_EQ(frames[1].data, (std::array<std::uint8_t, 8>{}));  // No previous startup command.
}

TEST(Core, ModeSpecificCurrentAndStartupLimitsAreValidated)
{
  auto motor = make_motor_config();
  motor.startup_current_a = 2.0;
  for (auto mode : {ControlMode::Current, ControlMode::Velocity, ControlMode::Position}) {
    motor.mode = mode;
    EXPECT_THROW(DriverCore({motor}), std::invalid_argument);
  }
  motor.mode = ControlMode::CurrentVelocity;
  EXPECT_NO_THROW(DriverCore({motor}));
  auto invalid = motor;
  invalid.startup_current_a = 0.0;
  EXPECT_THROW(DriverCore({invalid}), std::invalid_argument);
  invalid = motor;
  invalid.rpm_threshold = 0.0;
  EXPECT_THROW(DriverCore({invalid}), std::invalid_argument);
  invalid = motor;
  invalid.max_current_a = 1.0;
  EXPECT_THROW(DriverCore({invalid}), std::invalid_argument);
  invalid = motor;
  invalid.max_current_a = kUnknownValue;
  EXPECT_THROW(DriverCore({invalid}), std::invalid_argument);
  invalid = motor;
  invalid.max_current_a = 3e6;
  EXPECT_THROW(DriverCore({invalid}), std::invalid_argument);
}

}  // namespace
}  // namespace vesc_can_driver
