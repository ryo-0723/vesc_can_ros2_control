#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include "gtest/gtest.h"
#include "vesc_can/driver_core.hpp"
namespace vesc_can
{
namespace
{
using namespace std::chrono_literals;
MotorConfig make_motor_config(std::uint8_t controller_id = 11)
{
  MotorConfig motor_config;
  motor_config.name = "motor" + std::to_string(controller_id);
  motor_config.controller_id = controller_id;
  motor_config.command_timeout = 100ms;
  motor_config.feedback_timeout = 200ms;
  return motor_config;
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
  auto frame = protocol::make_set_rpm_frame(11, -123456);
  EXPECT_EQ(frame.id, 0x30Bu);
  EXPECT_TRUE(frame.is_extended);
  EXPECT_FALSE(frame.is_rtr);
  EXPECT_FALSE(frame.is_error);
  EXPECT_EQ(frame.dlc, 4);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{255, 254, 29, 192, 0, 0, 0, 0}));
  frame = protocol::make_set_current_frame(21, 1.5);
  EXPECT_EQ(frame.id, 0x115u);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{0, 0, 5, 220, 0, 0, 0, 0}));
  frame = protocol::make_set_position_frame(21, 90.0);
  EXPECT_EQ(frame.id, 0x415u);
  EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{5, 93, 74, 128, 0, 0, 0, 0}));
  EXPECT_THROW(protocol::make_set_current_frame(255, 0.0), std::invalid_argument);
  EXPECT_THROW(protocol::make_set_current_frame(11, kUnknownValue), std::out_of_range);
  EXPECT_THROW(protocol::make_set_position_frame(11, 1e20), std::out_of_range);
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
  auto motor_config = make_motor_config();
  motor_config.gear_ratio = 2.0;
  motor_config.direction = -1.0;
  EXPECT_NEAR(joint_velocity_to_erpm(motor_config, kPi), -420.0, 1e-9);
  DriverCore driver_core({motor_config});
  const auto start_time = TimePoint(1s);
  EXPECT_FALSE(driver_core.enable(start_time));
  ASSERT_TRUE(driver_core.receive(make_status_1_frame(), start_time));
  EXPECT_NEAR(driver_core.states()[0].joint_velocity_rad_s, -kPi, 1e-9);
  ASSERT_TRUE(driver_core.receive(make_status_5_frame(), start_time));
  EXPECT_NEAR(driver_core.states()[0].joint_position_rad, kPi, 1e-9);
  EXPECT_FALSE(driver_core.receive(make_status_1_frame(99), start_time));
  motor_config.control_mode = ControlMode::Position;
  motor_config.zero_offset_rad = kPi / 2;
  EXPECT_NEAR(joint_position_to_degrees(motor_config, kPi / 4), 0.0, 1e-9);
  EXPECT_NEAR(degrees_to_joint_position(motor_config, 0.0), kPi / 4, 1e-9);
}
TEST(Core, MultiMotorFeedbackAndCommandTimeoutLatchStopUntilExplicitEnable)
{
  DriverCore driver_core({make_motor_config(11), make_motor_config(12)});
  const auto start_time = TimePoint(1s);
  driver_core.receive(make_status_1_frame(11), start_time);
  EXPECT_FALSE(driver_core.enable(start_time));
  driver_core.receive(make_status_1_frame(12), start_time);
  ASSERT_TRUE(driver_core.enable(start_time));
  ASSERT_TRUE(driver_core.set_command(0, 1.0, start_time));
  ASSERT_TRUE(driver_core.set_command(1, 2.0, start_time));
  std::vector<can_msgs::msg::Frame> command_frames;
  driver_core.command_frames(start_time + 20ms, command_frames);
  ASSERT_EQ(command_frames.size(), 2u);
  EXPECT_EQ(command_frames[0].id >> 8, 3u);
  driver_core.command_frames(start_time + 101ms, command_frames);
  EXPECT_TRUE(driver_core.faulted());
  EXPECT_FALSE(driver_core.enabled());
  for (const auto &frame : command_frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data[0] | frame.data[1] | frame.data[2] | frame.data[3], 0);
  }
  driver_core.receive(make_status_1_frame(11), start_time + 102ms);
  driver_core.receive(make_status_1_frame(12), start_time + 102ms);
  EXPECT_FALSE(driver_core.set_command(0, 1.0, start_time + 102ms));
  EXPECT_TRUE(driver_core.faulted());
  ASSERT_TRUE(driver_core.enable(start_time + 102ms));
  EXPECT_FALSE(driver_core.faulted());
  ASSERT_TRUE(driver_core.set_command(0, 1.0, start_time + 102ms));
  ASSERT_TRUE(driver_core.set_command(1, 1.0, start_time + 102ms));
  driver_core.disable();
  driver_core.command_frames(start_time + 103ms, command_frames);
  EXPECT_EQ(command_frames[1].id >> 8, 1u);
}
TEST(Core, StaleRequiredFeedbackInvalidCommandsAndStartupStopAll)
{
  auto motor_config = make_motor_config();
  motor_config.command_timeout = 1000ms;
  DriverCore driver_core({motor_config, make_motor_config(12)});
  const auto start_time = TimePoint(1s);
  driver_core.receive(make_status_1_frame(11), start_time);
  driver_core.receive(make_status_1_frame(12), start_time);
  ASSERT_TRUE(driver_core.enable(start_time));
  driver_core.receive(make_status_1_frame(12), start_time + 201ms);
  driver_core.set_command(1, 1.0, start_time + 201ms);
  std::vector<can_msgs::msg::Frame> command_frames;
  driver_core.command_frames(start_time + 201ms, command_frames);
  EXPECT_TRUE(driver_core.faulted());
  EXPECT_EQ(command_frames[1].id >> 8, 1u);
  DriverCore invalid_command_driver({motor_config});
  invalid_command_driver.receive(make_status_1_frame(), start_time);
  ASSERT_TRUE(invalid_command_driver.enable(start_time));
  EXPECT_FALSE(invalid_command_driver.set_command(0, kUnknownValue, start_time));
  EXPECT_TRUE(invalid_command_driver.faulted());
  ASSERT_TRUE(invalid_command_driver.enable(start_time));
  EXPECT_FALSE(invalid_command_driver.set_command(0, 1000.0, start_time));
  motor_config.startup_current_a = 2.0;
  motor_config.control_mode = ControlMode::CurrentVelocity;
  motor_config.rpm_control_threshold_rpm = 100.0;
  motor_config.startup_timeout = 50ms;
  DriverCore startup_driver({motor_config});
  auto zero_speed_feedback = make_status_1_frame();
  zero_speed_feedback.data.fill(0);
  startup_driver.receive(zero_speed_feedback, start_time);
  ASSERT_TRUE(startup_driver.enable(start_time));
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
  auto motor_config = make_motor_config(21);
  motor_config.control_mode = ControlMode::Position;
  DriverCore driver_core({make_motor_config(), motor_config});
  const auto start_time = TimePoint(1s);
  driver_core.receive(make_status_1_frame(), start_time);
  driver_core.receive(make_status_1_frame(21), start_time);
  EXPECT_FALSE(driver_core.enable(start_time));
  driver_core.receive(make_status_4_frame(), start_time);
  ASSERT_TRUE(driver_core.enable(start_time));
  EXPECT_NEAR(driver_core.states()[1].joint_position_rad, kPi / 2, 1e-9);
  driver_core.set_command(0, 1.0, start_time);
  driver_core.set_command(1, kPi / 2, start_time);
  std::vector<can_msgs::msg::Frame> command_frames;
  driver_core.command_frames(start_time + 10ms, command_frames);
  ASSERT_EQ(command_frames.size(), 2u);
  EXPECT_EQ(command_frames[0].id >> 8, 3u);
  EXPECT_EQ(command_frames[1].id >> 8, 4u);
  motor_config.gear_ratio = 20.0;
  EXPECT_THROW(DriverCore({motor_config}), std::invalid_argument);
  auto duplicate_config = make_motor_config();
  EXPECT_THROW(DriverCore({duplicate_config, duplicate_config}), std::invalid_argument);
}

TEST(Core, ZeroVelocityReleasesCurrentWhileZeroPositionCommandsJointZero)
{
  auto steer = make_motor_config(21);
  steer.control_mode = ControlMode::Position;
  steer.zero_offset_rad = kPi / 2;
  DriverCore driver({make_motor_config(), steer});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(), start);
  driver.receive(make_status_1_frame(21), start);
  driver.receive(make_status_4_frame(), start);
  ASSERT_TRUE(driver.enable(start));
  ASSERT_TRUE(driver.set_command(0, 1.0, start));
  ASSERT_TRUE(driver.set_command(1, 0.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].id >> 8, 3u);
  EXPECT_EQ(frames[1].data, protocol::make_set_position_frame(21, 90.0).data);
  ASSERT_TRUE(driver.set_command(0, 0.0, start + 11ms));
  driver.command_frames(start + 11ms, frames);
  EXPECT_EQ(frames[0].id, 0x10Bu);
  EXPECT_EQ(frames[0].data, (std::array<std::uint8_t, 8>{}));
  EXPECT_EQ(frames[1].id, 0x415u);
  EXPECT_TRUE(driver.enabled());
  driver.disable();
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
  motor.joint_velocity_slew_rate_rad_s2 = kPi;
  motor.command_timeout = 1000ms;
  motor.feedback_timeout = 1000ms;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(), start);
  ASSERT_TRUE(driver.enable(start));
  ASSERT_TRUE(driver.set_command(0, kPi, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 500ms, frames);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].data, protocol::make_set_rpm_frame(11, -210).data);
  driver.command_frames(start + 1000ms, frames);
  EXPECT_EQ(frames[0].data, protocol::make_set_rpm_frame(11, -420).data);
}

TEST(Core, CurrentStartupTransitionsOnlyOnFeedbackInCommandDirection)
{
  auto motor = make_motor_config();
  motor.direction = -1.0;
  motor.control_mode = ControlMode::CurrentVelocity;
  motor.startup_current_a = 2.0;
  motor.rpm_control_threshold_rpm = 50.0;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  // Positive raw ERPM means negative joint velocity when direction=-1.
  driver.receive(make_status_1_frame(), start);
  ASSERT_TRUE(driver.enable(start));
  ASSERT_TRUE(driver.set_command(0, 10.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].id, 0x10Bu);
  EXPECT_EQ(frames[0].data, protocol::make_set_current_frame(11, -2.0).data);
  auto feedback = make_status_1_frame();
  feedback.data = {255, 255, 254, 92, 0, 0, 0, 0};  // -420 ERPM = +60 motor RPM.
  driver.receive(feedback, start + 20ms);
  driver.command_frames(start + 20ms, frames);
  EXPECT_EQ(frames[0].id, 0x30Bu);
  EXPECT_TRUE(driver.enabled());
  EXPECT_FALSE(driver.faulted());
}

TEST(Core, PositionRangeAndOptionalTachometerAreEnforced)
{
  auto motor = make_motor_config(21);
  motor.control_mode = ControlMode::Position;
  motor.gear_ratio = 2.0;
  motor.min_joint_position_rad = -1.0;
  motor.max_joint_position_rad = 1.0;
  EXPECT_TRUE(command_is_valid(motor, -1.0));
  EXPECT_TRUE(command_is_valid(motor, 1.0));
  EXPECT_FALSE(command_is_valid(motor, 1.01));
  EXPECT_FALSE(command_is_valid(motor, std::numeric_limits<double>::infinity()));
  auto invalid = motor;
  invalid.max_joint_position_rad = kPi;
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
  EXPECT_FALSE(driver.enable(start));
  driver.receive(make_status_5_frame(), start);
  ASSERT_TRUE(driver.enable(start));
  EXPECT_NEAR(driver.states()[1].joint_position_rad, kPi / 4, 1e-9);
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
  EXPECT_EQ(resolve_control_mode("current_velocity", "velocity"), ControlMode::CurrentVelocity);
  EXPECT_THROW(resolve_control_mode("current_velocity", "current"), std::invalid_argument);
  EXPECT_THROW(resolve_control_mode("velocity", "position"), std::invalid_argument);
  EXPECT_THROW(control_mode_from_string("effort"), std::invalid_argument);
  EXPECT_THROW(control_mode_from_string("unknown"), std::invalid_argument);
  EXPECT_THROW(command_interface_for_mode(static_cast<ControlMode>(123)), std::invalid_argument);
}

TEST(Core, CurrentCommandsUseAmpsDirectionAndSharedStops)
{
  auto current = make_motor_config(12);
  current.control_mode = ControlMode::Current;
  current.max_current_a = 3.0;
  current.direction = -1.0;
  current.gear_ratio = 2.0;
  current.require_tachometer = true;
  DriverCore driver({current, make_motor_config()});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(12), start);
  driver.receive(make_status_1_frame(), start);
  EXPECT_FALSE(driver.enable(start));
  driver.receive(make_status_5_frame(12), start);
  ASSERT_TRUE(driver.enable(start));
  EXPECT_NEAR(driver.states()[0].joint_position_rad, kPi, 1e-9);
  EXPECT_DOUBLE_EQ(
    motor_current_to_joint_current(current, driver.states()[0].motor_current_a), -1.5);
  std::vector<can_msgs::msg::Frame> frames;
  for (double command : {2.5, -2.5, 0.0}) {
    ASSERT_TRUE(driver.set_command(0, command, start));
    ASSERT_TRUE(driver.set_command(1, 1.0, start));
    driver.command_frames(start + 10ms, frames);
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].id, 0x10Cu);
    EXPECT_EQ(frames[0].data, protocol::make_set_current_frame(12, -command).data);
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

TEST(Core, FourModesShareFeedbackTimeoutAndExplicitRecovery)
{
  auto current = make_motor_config(12);
  current.control_mode = ControlMode::Current;
  auto steer = make_motor_config(21);
  steer.control_mode = ControlMode::Position;
  auto hybrid = make_motor_config(31);
  hybrid.control_mode = ControlMode::CurrentVelocity;
  hybrid.startup_current_a = 2.0;
  hybrid.rpm_control_threshold_rpm = 100.0;
  DriverCore driver({current, make_motor_config(), steer, hybrid});
  const auto start = TimePoint(1s);
  for (auto id : {12, 11, 21, 31}) {
    auto feedback = make_status_1_frame(id);
    feedback.data.fill(0);
    driver.receive(feedback, start);
  }
  driver.receive(make_status_4_frame(), start);
  ASSERT_TRUE(driver.enable(start));
  for (std::size_t i = 0; i < 4; ++i) {
    ASSERT_TRUE(driver.set_command(i, i == 3 ? 10.0 : 1.0, start));
  }
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  ASSERT_EQ(frames.size(), 4u);
  EXPECT_EQ(frames[0].id, 0x10Cu);
  EXPECT_EQ(frames[1].id, 0x30Bu);
  EXPECT_EQ(frames[2].id, 0x415u);
  EXPECT_EQ(frames[3].id, 0x11Fu);
  driver.command_frames(start + 101ms, frames);
  ASSERT_TRUE(driver.faulted());
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);
    EXPECT_EQ(frame.data, (std::array<std::uint8_t, 8>{}));
  }
  EXPECT_FALSE(driver.set_command(0, 1.0, start + 102ms));
  ASSERT_TRUE(driver.enable(start + 102ms));
  driver.command_frames(start + 103ms, frames);
  for (const auto &frame : frames) {
    EXPECT_EQ(frame.id >> 8, 1u);  // Re-enabling does not reuse any mode's old command.
  }
}

TEST(Core, HybridRestartsOnReversalAndZeroButNotOnSpeedDrop)
{
  auto motor = make_motor_config();
  motor.control_mode = ControlMode::CurrentVelocity;
  motor.startup_current_a = 2.0;
  motor.rpm_control_threshold_rpm = 50.0;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(), start);  // Already rotating at 60 motor RPM.
  ASSERT_TRUE(driver.enable(start));
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
  EXPECT_EQ(frames[0].data, protocol::make_set_current_frame(11, -2.0).data);
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
  EXPECT_EQ(frames[0].data, protocol::make_set_current_frame(11, -2.0).data);
}

TEST(Core, HybridTransitionNeverCommandsAboveConfiguredSpeedLimit)
{
  auto motor = make_motor_config();
  motor.control_mode = ControlMode::CurrentVelocity;
  motor.startup_current_a = 2.0;
  motor.rpm_control_threshold_rpm = 50.0;
  motor.max_joint_velocity_rad_s = 10.0;
  motor.joint_velocity_slew_rate_rad_s2 = 1.0;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  auto feedback = make_status_1_frame();
  feedback.data = {0, 0, 17, 148, 0, 0, 0, 0};  // 4500 ERPM exceeds max_velocity.
  driver.receive(feedback, start);
  ASSERT_TRUE(driver.enable(start));
  ASSERT_TRUE(driver.set_command(0, 2.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 10ms, frames);
  EXPECT_EQ(frames[0].id >> 8, 3u);
  EXPECT_EQ(frames[0].data, protocol::make_set_rpm_frame(11, 668).data);  // 9.99 rad/s.
}

TEST(Core, DirectVelocityReversalPreservesSlewRate)
{
  auto motor = make_motor_config();
  motor.joint_velocity_slew_rate_rad_s2 = 10.0;
  motor.command_timeout = 1000ms;
  motor.feedback_timeout = 1000ms;
  DriverCore driver({motor});
  const auto start = TimePoint(1s);
  driver.receive(make_status_1_frame(), start);
  ASSERT_TRUE(driver.enable(start));
  ASSERT_TRUE(driver.set_command(0, 1.0, start));
  std::vector<can_msgs::msg::Frame> frames;
  driver.command_frames(start + 100ms, frames);
  EXPECT_EQ(frames[0].data, protocol::make_set_rpm_frame(11, 67).data);
  ASSERT_TRUE(driver.set_command(0, -1.0, start + 100ms));
  driver.command_frames(start + 150ms, frames);
  EXPECT_EQ(frames[0].data, protocol::make_set_rpm_frame(11, 33).data);
  driver.command_frames(start + 200ms, frames);
  EXPECT_EQ(frames[0].data, protocol::make_set_rpm_frame(11, 0).data);
  driver.command_frames(start + 250ms, frames);
  EXPECT_EQ(frames[0].data, protocol::make_set_rpm_frame(11, -33).data);
}

TEST(Core, ModeSpecificCurrentAndStartupLimitsAreValidated)
{
  auto motor = make_motor_config();
  motor.startup_current_a = 2.0;
  for (auto control_mode : {ControlMode::Current, ControlMode::Velocity, ControlMode::Position}) {
    motor.control_mode = control_mode;
    EXPECT_THROW(DriverCore({motor}), std::invalid_argument);
  }
  motor.control_mode = ControlMode::CurrentVelocity;
  EXPECT_NO_THROW(DriverCore({motor}));
  auto invalid = motor;
  invalid.startup_current_a = 0.0;
  EXPECT_THROW(DriverCore({invalid}), std::invalid_argument);
  invalid = motor;
  invalid.rpm_control_threshold_rpm = 0.0;
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
}  // namespace vesc_can
