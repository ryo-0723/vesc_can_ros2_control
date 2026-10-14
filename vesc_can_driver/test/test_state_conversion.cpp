#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "vesc_can_driver/vesc_driver.hpp"
#include "vesc_can_driver/vesc_state.hpp"

namespace vesc_can_driver
{
namespace
{
using namespace std::chrono_literals;

can_msgs::msg::Frame status_frame(
  std::uint32_t packet_id, const std::array<std::uint8_t, 8> &payload)
{
  can_msgs::msg::Frame frame;
  frame.id = (packet_id << 8) | 11;
  frame.is_extended = true;
  frame.dlc = 8;
  frame.data = payload;
  return frame;
}

MotorConfig state_conversion_config()
{
  MotorConfig cfg;
  cfg.name = "drive_joint";
  cfg.controller_id = 11;
  cfg.gear_ratio = 2.0;
  cfg.direction = -1.0;
  cfg.feedback_timeout = 200ms;
  return cfg;
}

void receive_all_statuses(DriverCore &core, TimePoint received_at)
{
  // Known native wire values, independent of the fake node's encoder.
  ASSERT_TRUE(core.receive(status_frame(9, {0, 0, 1, 164, 0, 15, 1, 244}), received_at));
  ASSERT_TRUE(core.receive(status_frame(14, {0, 0, 48, 212, 0, 0, 19, 136}), received_at));
  ASSERT_TRUE(core.receive(status_frame(15, {0, 1, 212, 192, 0, 0, 39, 16}), received_at));
  ASSERT_TRUE(core.receive(status_frame(16, {1, 44, 1, 54, 0, 12, 17, 148}), received_at));
  ASSERT_TRUE(core.receive(status_frame(27, {255, 255, 255, 214, 0, 240, 0, 0}), received_at));
  ASSERT_TRUE(core.receive(status_frame(58, {4, 76, 8, 152, 12, 228, 254, 12}), received_at));
}

TEST(StateConversion, NativeValuesKeepUpstreamUnitsAndControllerIdentity)
{
  const auto cfg = state_conversion_config();
  DriverCore core({cfg});
  const auto received_at = TimePoint(1s);
  receive_all_statuses(core, received_at);
  const auto &state = core.states().front();
  const auto msg = make_vesc_state(cfg, state, received_at);
  EXPECT_EQ(msg.controller_id, 11);
  EXPECT_DOUBLE_EQ(msg.speed, 420.0);  // Raw ERPM, despite reversed gearing.
  EXPECT_NEAR(state.vel_rad_s, -kPi, 1e-9);
  EXPECT_DOUBLE_EQ(msg.current_motor, 1.5);
  EXPECT_DOUBLE_EQ(msg.duty_cycle, 0.5);
  EXPECT_DOUBLE_EQ(msg.current_input, 1.2);
  EXPECT_DOUBLE_EQ(msg.temp_fet, 30.0);
  EXPECT_DOUBLE_EQ(msg.temp_motor, 31.0);
  EXPECT_DOUBLE_EQ(msg.pid_pos_now, 90.0);  // Native motor PID angle in degrees.
  EXPECT_DOUBLE_EQ(msg.voltage_input, 24.0);
  EXPECT_EQ(msg.displacement, -42);
  EXPECT_DOUBLE_EQ(msg.charge_drawn, 1.25);
  EXPECT_DOUBLE_EQ(msg.charge_regen, 0.5);
  EXPECT_DOUBLE_EQ(msg.energy_drawn, 12.0);
  EXPECT_DOUBLE_EQ(msg.energy_regen, 1.0);
  EXPECT_DOUBLE_EQ(state.adc_1_v, 1.1);
  EXPECT_DOUBLE_EQ(state.ppm, -0.5);
}

TEST(StateConversion, MissingValuesAndDeviceFaultAreNeverReportedAsHealthyZeros)
{
  const auto cfg = state_conversion_config();
  const auto msg = make_vesc_state(cfg, MotorState{}, TimePoint(1s));
  const std::array unavailable_values = {msg.speed, msg.current_motor, msg.current_input,
    msg.temp_fet, msg.temp_motor, msg.duty_cycle, msg.voltage_input, msg.charge_drawn,
    msg.charge_regen, msg.energy_drawn, msg.energy_regen, msg.pid_pos_now, msg.avg_id, msg.avg_iq,
    msg.ntc_temp_mos1, msg.ntc_temp_mos2, msg.ntc_temp_mos3, msg.avg_vd, msg.avg_vq};
  for (double value : unavailable_values) {
    EXPECT_TRUE(std::isnan(value));
  }
  EXPECT_EQ(msg.fault_code, kUnavailableVescInteger);
  EXPECT_NE(msg.fault_code, vesc_msgs::msg::VescState::FAULT_CODE_NONE);
  EXPECT_EQ(msg.distance_traveled, kUnavailableVescInteger);
}

TEST(StateConversion, DeviceFaultCodesRequireFreshRepliesAndKeepUnknownCodes)
{
  const auto cfg = state_conversion_config();
  DriverCore core({cfg});
  const auto received_at = TimePoint{};
  for (std::uint8_t code : {0, 4, 255}) {
    ASSERT_TRUE(core.receive_fault({11, code}, received_at));
    EXPECT_EQ(make_vesc_state(cfg, core.states()[0], received_at).fault_code, code);
    EXPECT_EQ(make_vesc_state(cfg, core.states()[0], received_at + 200ms).fault_code, code);
    EXPECT_EQ(make_vesc_state(cfg, core.states()[0], received_at + 201ms).fault_code, -1);
    EXPECT_EQ(make_vesc_state(cfg, core.states()[0], received_at - 1ms).fault_code, -1);
  }
}

TEST(StateConversion, FeedbackExpiresByPacketAndOptionalStatusIsNotRequiredForControl)
{
  const auto cfg = state_conversion_config();
  DriverCore core({cfg});
  const auto received_at = TimePoint(1s);
  const auto rpm_frame = status_frame(9, {0, 0, 1, 164, 0, 15, 1, 244});
  ASSERT_TRUE(core.receive(rpm_frame, received_at));
  EXPECT_TRUE(
    state_ready(cfg, core.states().front(), received_at));  // STATUS2/3/6 are optional telemetry.
  receive_all_statuses(core, received_at);
  const auto steady_now = received_at + 201ms;
  ASSERT_TRUE(core.receive(rpm_frame, steady_now));
  const auto msg = make_vesc_state(cfg, core.states().front(), steady_now);
  EXPECT_DOUBLE_EQ(msg.speed, 420.0);
  EXPECT_TRUE(std::isnan(msg.temp_fet));
  EXPECT_TRUE(std::isnan(msg.voltage_input));
  EXPECT_TRUE(std::isnan(msg.charge_drawn));
  EXPECT_TRUE(std::isnan(msg.energy_drawn));
  EXPECT_EQ(msg.displacement, -42);  // Last count; status_5_fresh becomes false.
}
}  // namespace
}  // namespace vesc_can_driver
