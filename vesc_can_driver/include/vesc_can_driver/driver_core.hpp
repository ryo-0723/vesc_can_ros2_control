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

#include "vesc_can_driver/vesc_protocol.hpp"

namespace vesc_can_driver
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

// Shared mode selection for the standalone node and ros2_control.
ControlMode parse_control_mode(std::string_view mode_name);
std::string_view command_interface(ControlMode mode);
ControlMode resolve_control_mode(std::string_view mode_name, std::string_view interface_name);
bool is_velocity_mode(ControlMode mode);

struct MotorConfig
{
  std::string name;
  std::uint8_t controller_id{};
  ControlMode mode{ControlMode::Velocity};
  double pole_pairs{7.0};
  double gear_ratio{1.0};  // motor revolutions / joint revolution
  double direction{1.0};
  double zero_offset_rad{0.0};  // motor PID angle at joint position zero
  double max_vel_rad_s{20.0};   // joint rad/s
  double max_current_a{10.0};   // motor A, signed in the joint direction
  double min_pos_rad{-kPi};
  double max_pos_rad{kPi};
  double vel_slew_rad_s2{100.0};  // joint rad/s^2
  double startup_current_a{0.0};  // required for CurrentVelocity
  double rpm_threshold{1000.0};   // motor mechanical RPM
  std::chrono::milliseconds startup_timeout{1000};
  std::chrono::milliseconds feedback_timeout{500};
  std::chrono::milliseconds command_timeout{500};
  bool require_tachometer{false};
};

struct MotorState
{
  // Joint values include gear ratio/direction; native motor values retain VESC units/sign.
  double pos_rad{kUnknownValue};
  double vel_rad_s{kUnknownValue};
  double motor_current_a{kUnknownValue};
  double temperature_fet_c{kUnknownValue};
  double temperature_motor_c{kUnknownValue};
  double input_voltage_v{kUnknownValue};
  double duty_cycle{kUnknownValue};
  double electrical_rpm{kUnknownValue};
  double input_current_a{kUnknownValue};
  double pid_pos_deg{kUnknownValue};
  double charge_drawn_ah{kUnknownValue};
  double charge_regen_ah{kUnknownValue};
  double energy_drawn_wh{kUnknownValue};
  double energy_regen_wh{kUnknownValue};
  std::int32_t tachometer_counts{};
  double adc_1_v{kUnknownValue};
  double adc_2_v{kUnknownValue};
  double adc_3_v{kUnknownValue};
  double ppm{kUnknownValue};  // normalized PPM input, dimensionless
  // nullopt means not received; a received timestamp may legitimately be TimePoint{}.
  std::optional<TimePoint> status1_at;
  std::optional<TimePoint> status2_at;
  std::optional<TimePoint> status3_at;
  std::optional<TimePoint> status4_at;
  std::optional<TimePoint> status5_at;
  std::optional<TimePoint> status6_at;
};

// Joint position/velocity <-> native VESC units. Current conversion changes only its sign.
double vel_to_erpm(const MotorConfig &cfg, double vel_rad_s);
double erpm_to_vel(const MotorConfig &cfg, double electrical_rpm);
double pos_to_deg(const MotorConfig &cfg, double pos_rad);
double deg_to_pos(const MotorConfig &cfg, double pid_pos_deg);
// direction is +/-1, so current [A] uses the same sign conversion in both directions.
double apply_direction(const MotorConfig &cfg, double current_a);
// Checks shared by the core, transport and state publishers.
void validate_configs(const std::vector<MotorConfig> &configs);
bool feedback_fresh(
  const std::optional<TimePoint> &received_at, TimePoint now, std::chrono::milliseconds timeout);
bool command_is_valid(const MotorConfig &cfg, double value);
bool state_ready(const MotorConfig &cfg, const MotorState &state, TimePoint now);

// ROS-free state machine (apart from the frame data type). Callers serialize access.
class DriverCore
{
public:
  explicit DriverCore(std::vector<MotorConfig> configs);
  const std::vector<MotorConfig> &motor_configs() const
  {
    return configs_;
  }
  const std::vector<MotorState> &states() const
  {
    return states_;
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
  bool set_command(std::size_t idx, double value, TimePoint now);
  void command_frames(TimePoint now, std::vector<can_msgs::msg::Frame> &frames);
  void stop_frames(std::vector<can_msgs::msg::Frame> &frames) const;

private:
  struct Command
  {
    double target{};  // A signed in joint direction, joint rad/s or joint rad by mode
    double ramp_vel{};
    std::optional<TimePoint> received_at;
    TimePoint ramp_at{};
    TimePoint startup_at{};
    bool rpm_control_active{false};
  };
  bool check_commands(TimePoint now);
  std::optional<can_msgs::msg::Frame> make_cmd(std::size_t idx, TimePoint now);
  can_msgs::msg::Frame make_current_cmd(std::size_t idx, double joint_current_a) const;
  can_msgs::msg::Frame make_velocity_cmd(std::size_t idx, TimePoint now);
  std::optional<can_msgs::msg::Frame> make_hybrid_cmd(std::size_t idx, TimePoint now);
  std::vector<MotorConfig> configs_;
  std::vector<MotorState> states_;
  std::vector<Command> commands_;
  bool enabled_{false};
  bool faulted_{false};
  TimePoint enabled_at_{};
  std::string fault_reason_;
};
}  // namespace vesc_can_driver
