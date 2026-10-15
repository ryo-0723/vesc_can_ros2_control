#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <thread>

#include "gtest/gtest.h"
#include "support/can_test_helpers.hpp"
#include "vesc_can_driver/driver_core.hpp"
#include "vesc_can_driver/vesc_node.hpp"

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
  cfg.logical_id = 1;
  cfg.motor_id = 11;
  cfg.gear_ratio = 2.0;
  cfg.direction = -1.0;
  cfg.feedback_timeout = 200ms;
  return cfg;
}

void receive_all_statuses(DriverCore &core, TimePoint received_at)
{
  // Known native wire values, independent of the fake node's encoder.
  ASSERT_TRUE(receive_frame(core, status_frame(9, {0, 0, 1, 164, 0, 15, 1, 244}), received_at));
  ASSERT_TRUE(receive_frame(core, status_frame(14, {0, 0, 48, 212, 0, 0, 19, 136}), received_at));
  ASSERT_TRUE(receive_frame(core, status_frame(15, {0, 1, 212, 192, 0, 0, 39, 16}), received_at));
  ASSERT_TRUE(receive_frame(core, status_frame(16, {1, 44, 1, 54, 0, 12, 17, 148}), received_at));
  ASSERT_TRUE(
    receive_frame(core, status_frame(27, {255, 255, 255, 214, 0, 240, 0, 0}), received_at));
  ASSERT_TRUE(
    receive_frame(core, status_frame(58, {4, 76, 8, 152, 12, 228, 254, 12}), received_at));
}

TEST(StateConversion, ConvertsFeedbackAndKeepsLogicalIdentityWithCompactMonitoring)
{
  const auto cfg = state_conversion_config();
  DriverCore core({cfg});
  const auto received_at = TimePoint(1s);
  receive_all_statuses(core, received_at);
  const auto &state = core.motors().front().state();
  const auto msg = make_vesc_state(cfg, state, received_at);
  EXPECT_EQ(msg.logical_id, 1);  // Different from physical CAN ID 11.
  EXPECT_NEAR(msg.velocity_rad_s, -kPi, 1e-9);
  EXPECT_NEAR(state.vel_rad_s, -kPi, 1e-9);
  EXPECT_FLOAT_EQ(msg.motor_current_a, 1.5f);
  EXPECT_FLOAT_EQ(msg.duty_cycle, 0.5f);
  EXPECT_FLOAT_EQ(msg.input_current_a, 1.2f);
  EXPECT_FLOAT_EQ(msg.temp_fet_c, 30.0f);
  EXPECT_FLOAT_EQ(msg.temp_motor_c, 31.0f);
  EXPECT_NEAR(msg.pid_pos_rad, -kPi / 4, 1e-9);  // Joint angle after gearing and direction.
  EXPECT_FLOAT_EQ(msg.input_voltage_v, 24.0f);
  EXPECT_NEAR(msg.position_rad, kPi, 1e-9);  // Reversed 2:1 gearing, unlike native fields.
  EXPECT_FLOAT_EQ(msg.charge_drawn_ah, 1.25f);
  EXPECT_FLOAT_EQ(msg.charge_regen_ah, 0.5f);
  EXPECT_FLOAT_EQ(msg.energy_drawn_wh, 12.0f);
  EXPECT_FLOAT_EQ(msg.energy_regen_wh, 1.0f);
  EXPECT_DOUBLE_EQ(state.adc_1_v, 1.1);
  EXPECT_DOUBLE_EQ(state.ppm, -0.5);
}

TEST(StateConversion, MissingValuesAndDeviceFaultAreNeverReportedAsHealthyZeros)
{
  const auto cfg = state_conversion_config();
  const auto msg = make_vesc_state(cfg, MotorState{}, TimePoint(1s));
  const std::array<double, 13> unavailable_values = {msg.velocity_rad_s, msg.motor_current_a,
    msg.input_current_a, msg.temp_fet_c, msg.temp_motor_c, msg.duty_cycle, msg.input_voltage_v,
    msg.charge_drawn_ah, msg.charge_regen_ah, msg.energy_drawn_wh, msg.energy_regen_wh,
    msg.pid_pos_rad, msg.position_rad};
  for (double value : unavailable_values) {
    EXPECT_TRUE(std::isnan(value));
  }
  EXPECT_EQ(msg.fault_code, kUnknownFaultCode);
  EXPECT_NE(msg.fault_code, 0);
}

TEST(StateConversion, VelocityAndAccumulatedAngleRetainPrecisionAtInt32Limits)
{
  const auto cfg = state_conversion_config();
  DriverCore core({cfg});
  const auto now = TimePoint(1s);
  ASSERT_TRUE(receive_frame(core, status_frame(9, {0x7F, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0}), now));
  ASSERT_TRUE(receive_frame(core, status_frame(27, {0x80, 0, 0, 0, 0, 240, 0, 0}), now));
  const auto msg = make_vesc_state(cfg, core.motors()[0].state(), now);
  EXPECT_DOUBLE_EQ(msg.velocity_rad_s, erpm_to_vel(cfg, std::numeric_limits<std::int32_t>::max()));
  EXPECT_DOUBLE_EQ(msg.position_rad, tacho_to_pos(cfg, std::numeric_limits<std::int32_t>::min()));
  EXPECT_TRUE(std::isfinite(msg.position_rad));
}

TEST(StateConversion, DeviceFaultCodesRequireFreshRepliesAndKeepUnknownCodes)
{
  const auto cfg = state_conversion_config();
  DriverCore core({cfg});
  const auto received_at = TimePoint{};
  for (std::uint8_t code : {0, 4, 255}) {
    ASSERT_TRUE(core.receive_fault({11, code}, received_at));
    EXPECT_EQ(make_vesc_state(cfg, core.motors()[0].state(), received_at).fault_code, code);
    EXPECT_EQ(make_vesc_state(cfg, core.motors()[0].state(), received_at + 200ms).fault_code, code);
    EXPECT_EQ(make_vesc_state(cfg, core.motors()[0].state(), received_at + 201ms).fault_code, -1);
    EXPECT_EQ(make_vesc_state(cfg, core.motors()[0].state(), received_at - 1ms).fault_code, -1);
  }
}

TEST(StateConversion, FeedbackExpiresByPacketAndOptionalStatusIsNotRequiredForControl)
{
  const auto cfg = state_conversion_config();
  DriverCore core({cfg});
  const auto received_at = TimePoint(1s);
  const auto rpm_frame = status_frame(9, {0, 0, 1, 164, 0, 15, 1, 244});
  ASSERT_TRUE(receive_frame(core, rpm_frame, received_at));
  EXPECT_TRUE(state_ready(
    cfg, core.motors().front().state(), received_at));  // STATUS2/3/6 are optional telemetry.
  receive_all_statuses(core, received_at);
  const auto steady_now = received_at + 201ms;
  ASSERT_TRUE(receive_frame(core, rpm_frame, steady_now));
  const auto msg = make_vesc_state(cfg, core.motors().front().state(), steady_now);
  EXPECT_NEAR(msg.velocity_rad_s, -kPi, 1e-9);
  EXPECT_TRUE(std::isnan(msg.temp_fet_c));
  EXPECT_TRUE(std::isnan(msg.input_voltage_v));
  EXPECT_TRUE(std::isnan(msg.charge_drawn_ah));
  EXPECT_TRUE(std::isnan(msg.energy_drawn_wh));
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
    ASSERT_TRUE(receive_frame(core, status_frame(27, {255, 255, 255, 88, 0, 240, 0, 0}), now));
    EXPECT_NEAR(make_vesc_state(cfg, core.motors()[0].state(), now).position_rad, 4.0 * kPi, 1e-9);
    ASSERT_TRUE(receive_frame(core, status_frame(16, {1, 44, 1, 54, 0, 12, 17, 148}), now + 1ms));
    const auto msg = make_vesc_state(cfg, core.motors()[0].state(), now + 200ms);
    EXPECT_NEAR(msg.position_rad, 4.0 * kPi, 1e-9);
    EXPECT_NEAR(msg.pid_pos_rad, 0.0, 1e-9);
    if (mode == ControlMode::Position) {
      EXPECT_NEAR(
        core.motors()[0].state().pos_rad, 0.0, 1e-9);  // PID feedback keeps its own origin.
    }
    const auto stale = make_vesc_state(cfg, core.motors()[0].state(), now + 201ms);
    EXPECT_TRUE(std::isnan(stale.position_rad));
    EXPECT_NEAR(stale.pid_pos_rad, 0.0, 1e-9);  // A fresh STATUS4 cannot refresh STATUS5.
    EXPECT_TRUE(std::isnan(make_vesc_state(cfg, core.motors()[0].state(), now - 1ms).position_rad));
  }
}

class StatePublisherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(StatePublisherTest, EventsContainLogicalIdsAndFaultNamesAndRetainAllStoppedMotors)
{
  auto first = state_conversion_config();
  auto second = first;
  second.name = "steer_joint";
  second.logical_id = 2;
  second.motor_id = 21;
  auto node = std::make_shared<rclcpp::Node>("compact_event_test");
  std::vector<vesc_can_msgs::msg::VescEvent> events;
  const auto qos = rclcpp::QoS(2).reliable().transient_local();
  auto sub = node->create_subscription<vesc_can_msgs::msg::VescEvent>("/compact_test/event", qos,
    [&](vesc_can_msgs::msg::VescEvent::ConstSharedPtr msg) { events.push_back(*msg); });
  MotorStatePublisher publisher(
    *node, {first, second}, "/compact_test/event", "/compact_test/state");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  const auto wait_for = [&](auto condition) {
    const auto deadline = SteadyClock::now() + 1s;
    while (SteadyClock::now() < deadline) {
      exec.spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return condition();
  };
  ASSERT_TRUE(wait_for([&]() { return sub->get_publisher_count() > 0; }));
  for (const auto &[code, name] : {std::pair{4, "ABS_OVER_CURRENT"}, {5, "OVER_TEMP_FET"},
         {6, "OVER_TEMP_MOTOR"}, {29, "LV_OUTPUT_FAULT"}, {255, "UNKNOWN"}}) {
    const auto count = events.size();
    publisher.publish_error(
      {"raw driver reason", protocol::FaultCode{21, static_cast<std::uint8_t>(code)}});
    ASSERT_TRUE(wait_for([&]() { return events.size() == count + 1; }));
    EXPECT_EQ(events.back().logical_id, 2);
    EXPECT_EQ(events.back().message, "VESC fault_code=" + std::to_string(code) + " (" + name + ")");
  }
  const auto count = events.size();
  publisher.publish_error({"healthy reply", protocol::FaultCode{21, 0}});
  exec.spin_some();
  EXPECT_EQ(events.size(), count);
  publisher.publish_error({"CAN error frame", std::nullopt});
  ASSERT_TRUE(wait_for([&]() { return events.size() == count + 2; }));
  EXPECT_EQ(events[count].logical_id, 1);
  EXPECT_EQ(events[count + 1].logical_id, 2);
  EXPECT_EQ(events[count].message, "CAN error frame");
  std::vector<vesc_can_msgs::msg::VescEvent> retained;
  auto late_sub = node->create_subscription<vesc_can_msgs::msg::VescEvent>("/compact_test/event",
    qos, [&](vesc_can_msgs::msg::VescEvent::ConstSharedPtr msg) { retained.push_back(*msg); });
  ASSERT_TRUE(wait_for([&]() { return retained.size() == 2; }));
  EXPECT_EQ((std::set<std::uint8_t>{retained[0].logical_id, retained[1].logical_id}),
    (std::set<std::uint8_t>{1, 2}));
  EXPECT_EQ(retained[0].message, "CAN error frame");
  EXPECT_EQ(retained[1].message, "CAN error frame");
}
}  // namespace
}  // namespace vesc_can_driver
