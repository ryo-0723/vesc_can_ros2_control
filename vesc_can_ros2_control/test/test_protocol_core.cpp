#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "vesc_can_ros2_control/driver_core.hpp"
namespace vesc_can_ros2_control
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
  EXPECT_EQ(status_1.erpm, 420);
  EXPECT_DOUBLE_EQ(status_1.current_a, 1.5);
  EXPECT_DOUBLE_EQ(status_1.duty_cycle, 0.5);
  frame.data = {255, 255, 252, 24, 255, 133, 252, 24};
  ASSERT_TRUE(protocol::decode_status_1(frame, status_1));
  EXPECT_EQ(status_1.erpm, -1000);
  EXPECT_DOUBLE_EQ(status_1.current_a, -12.3);
  EXPECT_DOUBLE_EQ(status_1.duty_cycle, -1.0);
  frame.data = {128, 0, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(protocol::decode_status_1(frame, status_1));
  EXPECT_EQ(status_1.erpm, std::numeric_limits<std::int32_t>::min());
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
  EXPECT_DOUBLE_EQ(status_4.position_deg, 90.0);
  ASSERT_TRUE(protocol::decode_status_5(make_status_5_frame(), status_5));
  EXPECT_EQ(status_5.tachometer, -42);
  EXPECT_DOUBLE_EQ(status_5.input_voltage_v, 24.0);
  frame = make_status_5_frame();
  frame.data[6] = 0xA5;
  frame.data[7] = 0x5A;
  ASSERT_TRUE(protocol::decode_status_5(frame, status_5));
  EXPECT_EQ(status_5.tachometer, -42);  // Reserved bytes do not change known fields.
  frame.dlc = 6;
  EXPECT_TRUE(protocol::decode_status_5(frame, status_5));
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
  EXPECT_NEAR(driver_core.states()[0].velocity, -kPi, 1e-9);
  ASSERT_TRUE(driver_core.receive(make_status_5_frame(), start_time));
  EXPECT_NEAR(driver_core.states()[0].position, kPi, 1e-9);
  EXPECT_FALSE(driver_core.receive(make_status_1_frame(99), start_time));
  motor_config.mode = ControlMode::Position;
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
  motor_config.mode = ControlMode::Position;
  DriverCore driver_core({make_motor_config(), motor_config});
  const auto start_time = TimePoint(1s);
  driver_core.receive(make_status_1_frame(), start_time);
  driver_core.receive(make_status_1_frame(21), start_time);
  EXPECT_FALSE(driver_core.enable(start_time));
  driver_core.receive(make_status_4_frame(), start_time);
  ASSERT_TRUE(driver_core.enable(start_time));
  EXPECT_NEAR(driver_core.states()[1].position, kPi / 2, 1e-9);
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

}  // namespace
}  // namespace vesc_can_ros2_control
