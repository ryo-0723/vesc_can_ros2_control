#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "vesc_can/vesc_protocol.hpp"

namespace vesc_can
{
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kUnknownValue = std::numeric_limits<double>::quiet_NaN();
enum class ControlMode
{
  Current,
  Velocity,
  Position,
  CurrentVelocity
};

ControlMode control_mode_from_string(std::string_view control_mode_name);
std::string_view command_interface_for_mode(ControlMode control_mode);
ControlMode resolve_control_mode(
  std::string_view control_mode_name, std::string_view command_interface_name);
bool uses_velocity_command(ControlMode control_mode);

struct MotorConfig
{
  std::string name;
  std::uint8_t controller_id{};
  ControlMode control_mode{ControlMode::Velocity};
  double pole_pairs{7.0};
  double gear_ratio{1.0};  // motor revolutions / joint revolution
  double direction{1.0};
  double zero_offset_rad{0.0};            // motor PID angle at joint position zero
  double max_joint_velocity_rad_s{20.0};  // joint rad/s
  double max_current_a{10.0};             // motor A, signed in the joint direction
  double min_joint_position_rad{-kPi};
  double max_joint_position_rad{kPi};
  double joint_velocity_slew_rate_rad_s2{100.0};  // joint rad/s^2
  double startup_current_a{0.0};                  // required for CurrentVelocity
  double rpm_control_threshold_rpm{1000.0};       // motor mechanical RPM
  std::chrono::milliseconds startup_timeout{1000};
  std::chrono::milliseconds feedback_timeout{500};
  std::chrono::milliseconds command_timeout{500};
  bool require_tachometer{false};
};

struct MotorState
{
  // Joint values include gear ratio/direction; native motor values retain VESC units/sign.
  double joint_position_rad{kUnknownValue};
  double joint_velocity_rad_s{kUnknownValue};
  double motor_current_a{kUnknownValue};
  double temperature_fet_c{kUnknownValue};
  double temperature_motor_c{kUnknownValue};
  double input_voltage_v{kUnknownValue};
  double duty_cycle{kUnknownValue};
  double electrical_rpm{kUnknownValue};
  double input_current_a{kUnknownValue};
  double motor_pid_position_deg{kUnknownValue};
  double charge_drawn_ah{kUnknownValue};
  double charge_regen_ah{kUnknownValue};
  double energy_drawn_wh{kUnknownValue};
  double energy_regen_wh{kUnknownValue};
  std::int32_t tachometer_counts{};
  double adc_1_v{kUnknownValue};
  double adc_2_v{kUnknownValue};
  double adc_3_v{kUnknownValue};
  double ppm{kUnknownValue};  // normalized PPM input, dimensionless
  TimePoint status_1_received_at{};
  TimePoint status_2_received_at{};
  TimePoint status_3_received_at{};
  TimePoint status_4_received_at{};
  TimePoint status_5_received_at{};
  TimePoint status_6_received_at{};
  bool has_status_1{false};
  bool has_status_2{false};
  bool has_status_3{false};
  bool has_status_4{false};
  bool has_status_5{false};
  bool has_status_6{false};
};

double joint_velocity_to_erpm(const MotorConfig &motor_config, double joint_velocity_rad_s);
double erpm_to_joint_velocity(const MotorConfig &motor_config, double electrical_rpm);
double joint_position_to_degrees(const MotorConfig &motor_config, double joint_position_rad);
double degrees_to_joint_position(const MotorConfig &motor_config, double motor_pid_position_deg);
double joint_current_to_motor_current(const MotorConfig &motor_config, double joint_current_a);
double motor_current_to_joint_current(const MotorConfig &motor_config, double motor_current_a);
void validate_configs(const std::vector<MotorConfig> &motor_configs);
bool feedback_is_fresh(bool has_feedback, TimePoint received_at, TimePoint current_time,
  std::chrono::milliseconds timeout);
bool command_is_valid(const MotorConfig &motor_config, double command_value);
bool state_ready(
  const MotorConfig &motor_config, const MotorState &motor_state, TimePoint current_time);

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
  bool receive(const can_msgs::msg::Frame &frame, TimePoint current_time);
  bool ready(TimePoint current_time) const;
  bool enable(TimePoint current_time);
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
  bool set_command(std::size_t motor_index, double command_value, TimePoint current_time);
  void command_frames(TimePoint current_time, std::vector<can_msgs::msg::Frame> &frames);
  void stop_frames(std::vector<can_msgs::msg::Frame> &frames) const;

private:
  struct Command
  {
    double target_value{};  // A signed in joint direction, joint rad/s or joint rad by control_mode
    double ramped_joint_velocity_rad_s{};
    TimePoint received_at{};
    TimePoint last_ramp_update_at{};
    TimePoint startup_started_at{};
    bool has_command{false};
    bool rpm_control_active{false};
  };
  std::optional<can_msgs::msg::Frame> make_command(std::size_t motor_index, TimePoint current_time);
  can_msgs::msg::Frame make_current_command(std::size_t motor_index, double joint_current_a) const;
  can_msgs::msg::Frame make_velocity_command(std::size_t motor_index, TimePoint current_time);
  std::optional<can_msgs::msg::Frame> make_current_velocity_command(
    std::size_t motor_index, TimePoint current_time);
  std::vector<MotorConfig> motor_configs_;
  std::vector<MotorState> motor_states_;
  std::vector<Command> motor_commands_;
  bool enabled_{false};
  bool faulted_{false};
  TimePoint enabled_at_{};
  std::string fault_reason_;
};
}  // namespace vesc_can
