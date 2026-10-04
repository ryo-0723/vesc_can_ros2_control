#pragma once

#include <chrono>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>
#include "vesc_can_ros2_control/vesc_protocol.hpp"

namespace vesc_can_ros2_control
{
using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;
constexpr double PI = 3.14159265358979323846;
constexpr double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();
enum class ControlMode {VELOCITY, POSITION};

struct MotorConfig
{
  std::string name;
  uint16_t logical_id{};
  uint8_t controller_id{};
  ControlMode mode{ControlMode::VELOCITY};
  double pole_pairs{7.0};
  double gear_ratio{1.0};  // motor revolutions / joint revolution
  double direction{1.0};
  double zero_offset_rad{0.0};  // motor PID angle at joint position zero
  double max_velocity{20.0};  // joint rad/s
  double min_position{-PI};
  double max_position{PI};
  double velocity_slew_rate{100.0};  // joint rad/s^2
  double startup_current_a{0.0};  // zero disables legacy current-start mode
  double rpm_control_threshold_rpm{0.0};  // motor mechanical RPM
  std::chrono::milliseconds startup_timeout{1000};
  std::chrono::milliseconds feedback_timeout{500};
  std::chrono::milliseconds command_timeout{500};
  bool require_tachometer{false};
};

struct MotorState
{
  double position{NAN_VALUE};
  double velocity{NAN_VALUE};
  double current_a{NAN_VALUE};
  double temperature_fet_c{NAN_VALUE};
  double temperature_motor_c{NAN_VALUE};
  double input_voltage_v{NAN_VALUE};
  double duty_cycle{NAN_VALUE};
  TimePoint status1_time{};
  TimePoint status4_time{};
  TimePoint status5_time{};
  bool have_status1{false};
  bool have_status4{false};
  bool have_status5{false};
};

double joint_velocity_to_erpm(const MotorConfig & config, double velocity);
double erpm_to_joint_velocity(const MotorConfig & config, double erpm);
double joint_position_to_degrees(const MotorConfig & config, double position);
double degrees_to_joint_position(const MotorConfig & config, double degrees);
void validate_configs(const std::vector<MotorConfig> & configs);
bool state_ready(const MotorConfig & config, const MotorState & state, TimePoint now);

// ROS-free state machine (apart from the frame data type). Callers serialize access.
class DriverCore
{
public:
  explicit DriverCore(std::vector<MotorConfig> configs);
  const std::vector<MotorConfig> & configs() const {return configs_;}
  const std::vector<MotorState> & states() const {return states_;}
  bool receive(const can_msgs::msg::Frame & frame, TimePoint now);
  bool ready(TimePoint now) const;
  bool enable(TimePoint now);
  void disable();
  void stop_on_error(const std::string & reason) {fault(reason);}
  bool enabled() const {return enabled_;}
  bool faulted() const {return faulted_;}
  const std::string & fault_reason() const {return fault_reason_;}
  bool set_command(std::size_t index, double value, TimePoint now);
  void command_frames(TimePoint now, std::vector<can_msgs::msg::Frame> & frames);
  void stop_frames(std::vector<can_msgs::msg::Frame> & frames) const;

private:
  struct Command
  {
    double target{};
    double ramped_velocity{};
    TimePoint received{};
    TimePoint ramp_time{};
    TimePoint startup_time{};
    bool have_command{false};
    bool rpm_active{false};
  };
  void fault(const std::string & reason);
  std::vector<MotorConfig> configs_;
  std::vector<MotorState> states_;
  std::vector<Command> commands_;
  bool enabled_{false};
  bool faulted_{false};
  TimePoint enabled_time_{};
  std::string fault_reason_;
};
}  // namespace vesc_can_ros2_control
