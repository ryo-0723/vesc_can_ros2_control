#include <array>
#include <cmath>
#include <limits>

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

TEST(StateConversion, NativeValuesKeepMotorUnitsAndControllerIdentity)
{
  const auto cfg = state_conversion_config();
  DriverCore core({cfg});
  const auto received_at = TimePoint(1s);
  receive_all_statuses(core, received_at);
  const auto &state = core.states().front();
  const auto msg = make_vesc_state(cfg, state, received_at);
  EXPECT_EQ(msg.controller_id, 11);
  EXPECT_EQ(msg.motor_name, "drive_joint");
  EXPECT_DOUBLE_EQ(msg.erpm, 420.0);  // Raw ERPM, despite reversed gearing.
  EXPECT_NEAR(state.vel_rad_s, -kPi, 1e-9);
  EXPECT_DOUBLE_EQ(msg.motor_current_a, 1.5);
  EXPECT_DOUBLE_EQ(msg.duty_cycle, 0.5);
  EXPECT_DOUBLE_EQ(msg.input_current_a, 1.2);
  EXPECT_DOUBLE_EQ(msg.temp_fet_c, 30.0);
  EXPECT_DOUBLE_EQ(msg.temp_motor_c, 31.0);
  EXPECT_DOUBLE_EQ(msg.pid_pos_deg, 90.0);  // Native motor PID angle in degrees.
  EXPECT_DOUBLE_EQ(msg.input_voltage_v, 24.0);
  EXPECT_EQ(msg.tachometer_counts, -42);
  EXPECT_TRUE(msg.tachometer_fresh);
  EXPECT_NEAR(msg.position_rad, kPi, 1e-9);  // Reversed 2:1 gearing, unlike native fields.
  EXPECT_DOUBLE_EQ(msg.charge_drawn_ah, 1.25);
  EXPECT_DOUBLE_EQ(msg.charge_regen_ah, 0.5);
  EXPECT_DOUBLE_EQ(msg.energy_drawn_wh, 12.0);
  EXPECT_DOUBLE_EQ(msg.energy_regen_wh, 1.0);
  EXPECT_DOUBLE_EQ(state.adc_1_v, 1.1);
  EXPECT_DOUBLE_EQ(state.ppm, -0.5);
}

TEST(StateConversion, MissingValuesAndDeviceFaultAreNeverReportedAsHealthyZeros)
{
  const auto cfg = state_conversion_config();
  const auto msg = make_vesc_state(cfg, MotorState{}, TimePoint(1s));
  const std::array unavailable_values = {msg.erpm, msg.motor_current_a, msg.input_current_a,
    msg.temp_fet_c, msg.temp_motor_c, msg.duty_cycle, msg.input_voltage_v, msg.charge_drawn_ah,
    msg.charge_regen_ah, msg.energy_drawn_wh, msg.energy_regen_wh, msg.pid_pos_deg,
    msg.position_rad};
  for (double value : unavailable_values) {
    EXPECT_TRUE(std::isnan(value));
  }
  EXPECT_EQ(msg.fault_code, kUnknownFaultCode);
  EXPECT_NE(msg.fault_code, 0);
  EXPECT_FALSE(msg.tachometer_fresh);
  EXPECT_EQ(msg.tachometer_counts, 0);
}

TEST(StateConversion, NativeSignedCountersAndErpmRemainExact)
{
  const auto cfg = state_conversion_config();
  DriverCore core({cfg});
  const auto now = TimePoint(1s);
  ASSERT_TRUE(core.receive(status_frame(9, {0x7F, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0}), now));
  ASSERT_TRUE(core.receive(status_frame(27, {0x80, 0, 0, 0, 0, 240, 0, 0}), now));
  const auto msg = make_vesc_state(cfg, core.states()[0], now);
  EXPECT_DOUBLE_EQ(msg.erpm, std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(msg.tachometer_counts, std::numeric_limits<std::int32_t>::min());
  EXPECT_TRUE(msg.tachometer_fresh);
  EXPECT_TRUE(std::isfinite(msg.position_rad));
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
  EXPECT_DOUBLE_EQ(msg.erpm, 420.0);
  EXPECT_TRUE(std::isnan(msg.temp_fet_c));
  EXPECT_TRUE(std::isnan(msg.input_voltage_v));
  EXPECT_TRUE(std::isnan(msg.charge_drawn_ah));
  EXPECT_TRUE(std::isnan(msg.energy_drawn_wh));
  EXPECT_EQ(msg.tachometer_counts, -42);  // Last count is retained with explicit freshness.
  EXPECT_FALSE(msg.tachometer_fresh);
  EXPECT_TRUE(std::isnan(msg.position_rad));
}

TEST(StateConversion, AccumulatedAngleUsesStatus5InEveryModeAndExpiresIndependently)
{
  const auto now = TimePoint(1s);
  for (auto mode : {ControlMode::Current, ControlMode::Velocity, ControlMode::Position,
         ControlMode::DutyCycle, ControlMode::CurrentVelocity}) {
    SCOPED_TRACE(command_interface(mode));
    auto cfg = state_conversion_config();
    cfg.mode = mode;
    cfg.min_pos_rad = -1.0;
    cfg.max_pos_rad = 1.0;
    cfg.zero_offset_rad = kPi / 2;
    if (mode == ControlMode::CurrentVelocity) {
      cfg.startup_current_a = 2.0;
    }
    DriverCore core({cfg});
    // No STATUS1/4 is needed to decode and publish the measured accumulated angle.
    ASSERT_TRUE(core.receive(status_frame(27, {255, 255, 255, 88, 0, 240, 0, 0}), now));
    EXPECT_NEAR(make_vesc_state(cfg, core.states()[0], now).position_rad, 4.0 * kPi, 1e-9);
    ASSERT_TRUE(core.receive(status_frame(16, {1, 44, 1, 54, 0, 12, 17, 148}), now + 1ms));
    const auto msg = make_vesc_state(cfg, core.states()[0], now + 200ms);
    EXPECT_NEAR(msg.position_rad, 4.0 * kPi, 1e-9);
    EXPECT_DOUBLE_EQ(msg.pid_pos_deg, 90.0);
    if (mode == ControlMode::Position) {
      EXPECT_NEAR(core.states()[0].pos_rad, 0.0, 1e-9);  // PID feedback keeps its own origin.
    }
    const auto stale = make_vesc_state(cfg, core.states()[0], now + 201ms);
    EXPECT_TRUE(std::isnan(stale.position_rad));
    EXPECT_FALSE(stale.tachometer_fresh);
    EXPECT_DOUBLE_EQ(stale.pid_pos_deg, 90.0);  // A fresh STATUS4 cannot refresh STATUS5.
    EXPECT_TRUE(std::isnan(make_vesc_state(cfg, core.states()[0], now - 1ms).position_rad));
  }
}
}  // namespace
}  // namespace vesc_can_driver
