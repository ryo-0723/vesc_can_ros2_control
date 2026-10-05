#include <array>
#include <cmath>

#include "gtest/gtest.h"
#include "vesc_can_ros2_control/vesc_telemetry.hpp"

namespace vesc_can_ros2_control
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

MotorConfig telemetry_config()
{
  MotorConfig motor_config;
  motor_config.name = "drive_joint";
  motor_config.controller_id = 11;
  motor_config.gear_ratio = 2.0;
  motor_config.direction = -1.0;
  motor_config.feedback_timeout = 200ms;
  return motor_config;
}

void receive_all_statuses(DriverCore &driver_core, TimePoint received_at)
{
  // Known native wire values, independent of the fake node's encoder.
  ASSERT_TRUE(driver_core.receive(status_frame(9, {0, 0, 1, 164, 0, 15, 1, 244}), received_at));
  ASSERT_TRUE(driver_core.receive(status_frame(14, {0, 0, 48, 212, 0, 0, 19, 136}), received_at));
  ASSERT_TRUE(driver_core.receive(status_frame(15, {0, 1, 212, 192, 0, 0, 39, 16}), received_at));
  ASSERT_TRUE(driver_core.receive(status_frame(16, {1, 44, 1, 54, 0, 12, 17, 148}), received_at));
  ASSERT_TRUE(
    driver_core.receive(status_frame(27, {255, 255, 255, 214, 0, 240, 0, 0}), received_at));
  ASSERT_TRUE(
    driver_core.receive(status_frame(58, {4, 76, 8, 152, 12, 228, 254, 12}), received_at));
}

TEST(Telemetry, NativeValuesKeepUpstreamUnitsAndControllerIdentity)
{
  const auto motor_config = telemetry_config();
  DriverCore driver_core({motor_config});
  const auto received_at = TimePoint(1s);
  receive_all_statuses(driver_core, received_at);
  const auto &motor_state = driver_core.states().front();
  const auto message = make_vesc_state(motor_config, motor_state, received_at);
  EXPECT_EQ(message.controller_id, 11);
  EXPECT_DOUBLE_EQ(message.speed, 420.0);  // Raw ERPM, despite reversed gearing.
  EXPECT_NEAR(motor_state.velocity, -kPi, 1e-9);
  EXPECT_DOUBLE_EQ(message.current_motor, 1.5);
  EXPECT_DOUBLE_EQ(message.duty_cycle, 0.5);
  EXPECT_DOUBLE_EQ(message.current_input, 1.2);
  EXPECT_DOUBLE_EQ(message.temp_fet, 30.0);
  EXPECT_DOUBLE_EQ(message.temp_motor, 31.0);
  EXPECT_DOUBLE_EQ(message.pid_pos_now, 90.0);  // Native motor PID angle in degrees.
  EXPECT_DOUBLE_EQ(message.voltage_input, 24.0);
  EXPECT_EQ(message.displacement, -42);
  EXPECT_DOUBLE_EQ(message.charge_drawn, 1.25);
  EXPECT_DOUBLE_EQ(message.charge_regen, 0.5);
  EXPECT_DOUBLE_EQ(message.energy_drawn, 12.0);
  EXPECT_DOUBLE_EQ(message.energy_regen, 1.0);
  EXPECT_DOUBLE_EQ(motor_state.adc_1_v, 1.1);
  EXPECT_DOUBLE_EQ(motor_state.ppm, -0.5);
}

TEST(Telemetry, MissingValuesAndDeviceFaultAreNeverReportedAsHealthyZeros)
{
  const auto motor_config = telemetry_config();
  const auto message = make_vesc_state(motor_config, MotorState{}, TimePoint(1s));
  const std::array unavailable_values = {message.speed, message.current_motor,
    message.current_input, message.temp_fet, message.temp_motor, message.duty_cycle,
    message.voltage_input, message.charge_drawn, message.charge_regen, message.energy_drawn,
    message.energy_regen, message.pid_pos_now, message.avg_id, message.avg_iq,
    message.ntc_temp_mos1, message.ntc_temp_mos2, message.ntc_temp_mos3, message.avg_vd,
    message.avg_vq};
  for (double value : unavailable_values) {
    EXPECT_TRUE(std::isnan(value));
  }
  EXPECT_EQ(message.fault_code, kUnavailableVescInteger);
  EXPECT_NE(message.fault_code, vesc_msgs::msg::VescState::FAULT_CODE_NONE);
  EXPECT_EQ(message.distance_traveled, kUnavailableVescInteger);
}

TEST(Telemetry, FeedbackExpiresByPacketAndOptionalStatusIsNotRequiredForControl)
{
  const auto motor_config = telemetry_config();
  DriverCore driver_core({motor_config});
  const auto received_at = TimePoint(1s);
  const auto rpm_frame = status_frame(9, {0, 0, 1, 164, 0, 15, 1, 244});
  ASSERT_TRUE(driver_core.receive(rpm_frame, received_at));
  EXPECT_TRUE(driver_core.ready(received_at));  // STATUS2/3/6 are optional telemetry.
  receive_all_statuses(driver_core, received_at);
  const auto current_time = received_at + 201ms;
  ASSERT_TRUE(driver_core.receive(rpm_frame, current_time));
  const auto message = make_vesc_state(motor_config, driver_core.states().front(), current_time);
  EXPECT_DOUBLE_EQ(message.speed, 420.0);
  EXPECT_TRUE(std::isnan(message.temp_fet));
  EXPECT_TRUE(std::isnan(message.voltage_input));
  EXPECT_TRUE(std::isnan(message.charge_drawn));
  EXPECT_TRUE(std::isnan(message.energy_drawn));
  EXPECT_EQ(message.displacement, -42);  // Last count; status_5_fresh becomes false.
}
}  // namespace
}  // namespace vesc_can_ros2_control
