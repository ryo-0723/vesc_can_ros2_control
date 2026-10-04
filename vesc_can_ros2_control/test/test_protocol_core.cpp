#include <cmath>
#include <limits>
#include <stdexcept>
#include "gtest/gtest.h"
#include "vesc_can_ros2_control/driver_core.hpp"
using namespace vesc_can_ros2_control;
using namespace std::chrono_literals;
namespace
{
MotorConfig config(uint8_t id = 11)
{
  MotorConfig c; c.name = "motor" + std::to_string(id); c.logical_id = id; c.controller_id = id;
  c.command_timeout = 100ms; c.feedback_timeout = 200ms; return c;
}
can_msgs::msg::Frame status1(uint8_t id = 11)
{
  can_msgs::msg::Frame f; f.id = (9u << 8) | id; f.is_extended = true; f.dlc = 8;
  f.data = {0, 0, 1, 164, 0, 15, 1, 244}; return f;  // 420 ERPM, 1.5 A, 0.5 duty
}
can_msgs::msg::Frame status4(uint8_t id = 21)
{
  can_msgs::msg::Frame f; f.id = (16u << 8) | id; f.is_extended = true; f.dlc = 8;
  f.data = {1, 44, 1, 54, 0, 0, 17, 148}; return f;  // 30 C, 31 C, 90 degrees
}
can_msgs::msg::Frame status5(uint8_t id = 11)
{
  can_msgs::msg::Frame f; f.id = (27u << 8) | id; f.is_extended = true; f.dlc = 6;
  f.data = {255, 255, 255, 214, 0, 240, 0, 0}; return f;  // -42 counts, 24 V
}
}
TEST(Protocol, CommandsHaveNativeIdAndBigEndianSignedPayload)
{
  auto f = protocol::make_set_rpm_frame(11, -123456);
  EXPECT_EQ(f.id, 0x30Bu); EXPECT_TRUE(f.is_extended); EXPECT_FALSE(f.is_rtr); EXPECT_FALSE(f.is_error);
  EXPECT_EQ(f.dlc, 4); EXPECT_EQ(f.data, (std::array<uint8_t, 8>{255, 254, 29, 192, 0, 0, 0, 0}));
  f = protocol::make_set_current_frame(21, 1.5);
  EXPECT_EQ(f.id, 0x115u); EXPECT_EQ(f.data, (std::array<uint8_t, 8>{0, 0, 5, 220, 0, 0, 0, 0}));
  f = protocol::make_set_position_frame(21, 90.0);
  EXPECT_EQ(f.id, 0x415u); EXPECT_EQ(f.data, (std::array<uint8_t, 8>{5, 93, 74, 128, 0, 0, 0, 0}));
  EXPECT_THROW(protocol::make_set_current_frame(255, 0.0), std::invalid_argument);
  EXPECT_THROW(protocol::make_set_current_frame(11, NAN_VALUE), std::out_of_range);
  EXPECT_THROW(protocol::make_set_position_frame(11, 1e20), std::out_of_range);
}
TEST(Protocol, StatusAndMalformedFrames)
{
  protocol::Status1 one; protocol::Status4 four; protocol::Status5 five;
  auto f = status1(); ASSERT_TRUE(protocol::decode_status_1(f, one));
  EXPECT_EQ(one.erpm, 420); EXPECT_DOUBLE_EQ(one.current_a, 1.5); EXPECT_DOUBLE_EQ(one.duty_cycle, 0.5);
  f.data = {255, 255, 252, 24, 255, 133, 252, 24};
  ASSERT_TRUE(protocol::decode_status_1(f, one)); EXPECT_EQ(one.erpm, -1000);
  EXPECT_DOUBLE_EQ(one.current_a, -12.3); EXPECT_DOUBLE_EQ(one.duty_cycle, -1.0);
  f.is_extended = false; EXPECT_FALSE(protocol::decode_status_1(f, one));
  f = status1(); f.dlc = 7; EXPECT_FALSE(protocol::decode_status_1(f, one));
  f = status1(); f.is_error = true; EXPECT_FALSE(protocol::decode_status_1(f, one));
  f = status1(); f.is_rtr = true; EXPECT_FALSE(protocol::decode_status_1(f, one));
  f = status1(); f.id |= 0x20000000; EXPECT_FALSE(protocol::decode_status_1(f, one));
  ASSERT_TRUE(protocol::decode_status_4(status4(), four));
  EXPECT_DOUBLE_EQ(four.temperature_fet_c, 30.0); EXPECT_DOUBLE_EQ(four.position_deg, 90.0);
  ASSERT_TRUE(protocol::decode_status_5(status5(), five));
  EXPECT_EQ(five.tachometer, -42); EXPECT_DOUBLE_EQ(five.input_voltage_v, 24.0);
  f = status5(); f.dlc = 8; EXPECT_FALSE(protocol::decode_status_5(f, five));
}
TEST(Core, ConvertsJointUnitsAndUsesMeasuredTachometer)
{
  auto c = config(); c.gear_ratio = 2.0; c.direction = -1.0;
  EXPECT_NEAR(joint_velocity_to_erpm(c, PI), -420.0, 1e-9);
  DriverCore core({c}); const auto t = TimePoint(1s);
  EXPECT_FALSE(core.enable(t)); ASSERT_TRUE(core.receive(status1(), t));
  EXPECT_NEAR(core.states()[0].velocity, -PI, 1e-9);
  ASSERT_TRUE(core.receive(status5(), t)); EXPECT_NEAR(core.states()[0].position, PI, 1e-9);
  EXPECT_FALSE(core.receive(status1(99), t));
  c.mode = ControlMode::POSITION; c.zero_offset_rad = PI / 2;
  EXPECT_NEAR(joint_position_to_degrees(c, PI / 4), 0.0, 1e-9);
  EXPECT_NEAR(degrees_to_joint_position(c, 0.0), PI / 4, 1e-9);
}
TEST(Core, MultiMotorFeedbackAndCommandTimeoutLatchStopUntilExplicitEnable)
{
  DriverCore core({config(11), config(12)}); const auto t = TimePoint(1s);
  core.receive(status1(11), t); EXPECT_FALSE(core.enable(t));
  core.receive(status1(12), t); ASSERT_TRUE(core.enable(t));
  ASSERT_TRUE(core.set_command(0, 1.0, t)); ASSERT_TRUE(core.set_command(1, 2.0, t));
  std::vector<can_msgs::msg::Frame> frames;
  core.command_frames(t + 20ms, frames); ASSERT_EQ(frames.size(), 2u); EXPECT_EQ(frames[0].id >> 8, 3u);
  core.command_frames(t + 101ms, frames); EXPECT_TRUE(core.faulted()); EXPECT_FALSE(core.enabled());
  for (const auto & f : frames) {EXPECT_EQ(f.id >> 8, 1u); EXPECT_EQ(f.data[0] | f.data[1] | f.data[2] | f.data[3], 0);}
  core.receive(status1(11), t + 102ms); core.receive(status1(12), t + 102ms);
  EXPECT_FALSE(core.set_command(0, 1.0, t + 102ms)); EXPECT_TRUE(core.faulted());
  ASSERT_TRUE(core.enable(t + 102ms)); EXPECT_FALSE(core.faulted());
  ASSERT_TRUE(core.set_command(0, 1.0, t + 102ms)); ASSERT_TRUE(core.set_command(1, 1.0, t + 102ms));
  core.disable(); core.command_frames(t + 103ms, frames); EXPECT_EQ(frames[1].id >> 8, 1u);
}
TEST(Core, StaleRequiredFeedbackInvalidCommandsAndStartupStopAll)
{
  auto c = config(); c.command_timeout = 1000ms;
  DriverCore core({c, config(12)}); const auto t = TimePoint(1s);
  core.receive(status1(11), t); core.receive(status1(12), t); ASSERT_TRUE(core.enable(t));
  core.receive(status1(12), t + 201ms); core.set_command(1, 1.0, t + 201ms);
  std::vector<can_msgs::msg::Frame> frames; core.command_frames(t + 201ms, frames);
  EXPECT_TRUE(core.faulted()); EXPECT_EQ(frames[1].id >> 8, 1u);
  DriverCore invalid({c}); invalid.receive(status1(), t); ASSERT_TRUE(invalid.enable(t));
  EXPECT_FALSE(invalid.set_command(0, NAN_VALUE, t)); EXPECT_TRUE(invalid.faulted());
  ASSERT_TRUE(invalid.enable(t)); EXPECT_FALSE(invalid.set_command(0, 1000.0, t));
  c.startup_current_a = 2.0; c.rpm_control_threshold_rpm = 100.0; c.startup_timeout = 50ms;
  DriverCore startup({c}); auto stopped = status1(); stopped.data.fill(0);
  startup.receive(stopped, t); ASSERT_TRUE(startup.enable(t)); ASSERT_TRUE(startup.set_command(0, 10.0, t));
  startup.command_frames(t + 10ms, frames); EXPECT_EQ(frames[0].id >> 8, 1u); EXPECT_EQ(frames[0].data[3], 208);
  ASSERT_TRUE(startup.set_command(0, 10.0, t + 40ms));  // repeated commands must not extend startup timeout
  startup.command_frames(t + 51ms, frames); EXPECT_TRUE(startup.faulted()); EXPECT_EQ(frames[0].data[3], 0);
}
TEST(Core, MixedVelocityAndPositionRequireIndependentStatusStreams)
{
  auto c = config(21); c.mode = ControlMode::POSITION;
  DriverCore core({config(), c}); const auto t = TimePoint(1s);
  core.receive(status1(), t); core.receive(status1(21), t); EXPECT_FALSE(core.enable(t));
  core.receive(status4(), t); ASSERT_TRUE(core.enable(t));
  EXPECT_NEAR(core.states()[1].position, PI / 2, 1e-9);
  core.set_command(0, 1.0, t); core.set_command(1, PI / 2, t);
  std::vector<can_msgs::msg::Frame> frames; core.command_frames(t + 10ms, frames);
  ASSERT_EQ(frames.size(), 2u); EXPECT_EQ(frames[0].id >> 8, 3u); EXPECT_EQ(frames[1].id >> 8, 4u);
  c.gear_ratio = 20.0; EXPECT_THROW(DriverCore({c}), std::invalid_argument);
  auto duplicate = config(); EXPECT_THROW(DriverCore({duplicate, duplicate}), std::invalid_argument);
}
