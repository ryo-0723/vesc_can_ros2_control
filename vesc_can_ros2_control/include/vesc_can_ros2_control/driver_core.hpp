#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include "vesc_can_ros2_control/vesc_protocol.hpp"

namespace vesc_can_ros2_control
{
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kUnknownValue = std::numeric_limits<double>::quiet_NaN();
enum class ControlMode
{
  Velocity,
  Position
};

struct MotorConfig
{
  std::string name;
  std::uint8_t controller_id{};
  ControlMode mode{ControlMode::Velocity};
  double pole_pairs{7.0};
  double gear_ratio{1.0};  // motor revolutions / joint revolution
  double direction{1.0};
  double zero_offset_rad{0.0};  // motor PID angle at joint position zero
  double max_velocity{20.0};    // joint rad/s
  double min_position{-kPi};
  double max_position{kPi};
  double velocity_slew_rate{100.0};       // joint rad/s^2
  double startup_current_a{0.0};          // zero disables legacy current-start mode
  double rpm_control_threshold_rpm{0.0};  // motor mechanical RPM
  std::chrono::milliseconds startup_timeout{1000};
  std::chrono::milliseconds feedback_timeout{500};
  std::chrono::milliseconds command_timeout{500};
  bool require_tachometer{false};
};

struct MotorState
{
  double position{kUnknownValue};
  double velocity{kUnknownValue};
  double current_a{kUnknownValue};
  double temperature_fet_c{kUnknownValue};
  double temperature_motor_c{kUnknownValue};
  double input_voltage_v{kUnknownValue};
  double duty_cycle{kUnknownValue};
  double electrical_rpm{kUnknownValue};
  double input_current_a{kUnknownValue};
  double pid_position_deg{kUnknownValue};
  double charge_drawn_ah{kUnknownValue};
  double charge_regen_ah{kUnknownValue};
  double energy_drawn_wh{kUnknownValue};
  double energy_regen_wh{kUnknownValue};
  std::int32_t tachometer_counts{};
  double adc_1_v{kUnknownValue};
  double adc_2_v{kUnknownValue};
  double adc_3_v{kUnknownValue};
  double ppm{kUnknownValue};
  TimePoint status1_time{};
  TimePoint status2_time{};
  TimePoint status3_time{};
  TimePoint status4_time{};
  TimePoint status5_time{};
  TimePoint status6_time{};
  bool have_status1{false};
  bool have_status2{false};
  bool have_status3{false};
  bool have_status4{false};
  bool have_status5{false};
  bool have_status6{false};
};

double joint_velocity_to_erpm(const MotorConfig &motor_config, double velocity);
double erpm_to_joint_velocity(const MotorConfig &motor_config, double erpm);
double joint_position_to_degrees(const MotorConfig &motor_config, double position);
double degrees_to_joint_position(const MotorConfig &motor_config, double degrees);
void validate_configs(const std::vector<MotorConfig> &motor_configs);
bool feedback_is_fresh(
  bool has_feedback, TimePoint received_at, TimePoint now, std::chrono::milliseconds timeout);
bool command_is_valid(const MotorConfig &motor_config, double command);
bool state_ready(const MotorConfig &motor_config, const MotorState &motor_state, TimePoint now);

// ROS-free state machine (apart from the frame data type). Callers serialize access.
class DriverCore
{
public:
  explicit DriverCore(std::vector<MotorConfig> motor_configs);
  const std::vector<MotorConfig> &motor_configs() const
  {
    return motor_configs_;
  }
  const std::vector<MotorState> &states() const
  {
    return motor_states_;
  }
  bool receive(const can_msgs::msg::Frame &frame, TimePoint now);
  bool ready(TimePoint now) const;
  bool enable(TimePoint now);
  void disable();
  void stop_on_error(const std::string &reason);
  bool enabled() const
  {
    return enabled_;
  }
  bool faulted() const
  {
    return faulted_;
  }
  const std::string &fault_reason() const
  {
    return fault_reason_;
  }
  bool set_command(std::size_t motor_index, double value, TimePoint now);
  void command_frames(TimePoint now, std::vector<can_msgs::msg::Frame> &frames);
  void stop_frames(std::vector<can_msgs::msg::Frame> &frames) const;

private:
  struct Command
  {
    double target{};
    double ramped_velocity{};
    TimePoint received_at{};
    TimePoint last_ramp_update{};
    TimePoint startup_started_at{};
    bool has_command{false};
    bool rpm_control_active{false};
  };
  std::optional<can_msgs::msg::Frame> make_velocity_command(std::size_t motor_index, TimePoint now);
  std::vector<MotorConfig> motor_configs_;
  std::vector<MotorState> motor_states_;
  std::vector<Command> motor_commands_;
  bool enabled_{false};
  bool faulted_{false};
  TimePoint enabled_time_{};
  std::string fault_reason_;
};
}  // namespace vesc_can_ros2_control
