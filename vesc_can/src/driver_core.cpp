#include "vesc_can/driver_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace vesc_can
{
ControlMode control_mode_from_string(std::string_view control_mode_name)
{
  if (control_mode_name == "current") {
    return ControlMode::Current;
  }
  if (control_mode_name == "velocity") {
    return ControlMode::Velocity;
  }
  if (control_mode_name == "position") {
    return ControlMode::Position;
  }
  if (control_mode_name == "current_velocity") {
    return ControlMode::CurrentVelocity;
  }
  throw std::invalid_argument("Unsupported control mode: " + std::string(control_mode_name));
}

std::string_view command_interface_for_mode(ControlMode control_mode)
{
  switch (control_mode) {
    case ControlMode::Current:
      return "current";
    case ControlMode::Velocity:
    case ControlMode::CurrentVelocity:
      return "velocity";
    case ControlMode::Position:
      return "position";
  }
  throw std::invalid_argument("Unsupported control mode");
}

ControlMode resolve_control_mode(
  std::string_view control_mode_name, std::string_view command_interface_name)
{
  const auto resolved_mode = control_mode_from_string(
    control_mode_name.empty() ? command_interface_name : control_mode_name);
  if (command_interface_for_mode(resolved_mode) != command_interface_name) {
    throw std::invalid_argument("Control mode and command interface do not match");
  }
  return resolved_mode;
}

bool uses_velocity_command(ControlMode control_mode)
{
  return control_mode == ControlMode::Velocity || control_mode == ControlMode::CurrentVelocity;
}

double joint_velocity_to_erpm(const MotorConfig &motor_config, double joint_velocity_rad_s)
{
  return joint_velocity_rad_s * 60.0 / (2.0 * kPi) * motor_config.gear_ratio *
         motor_config.pole_pairs * motor_config.direction;
}

double erpm_to_joint_velocity(const MotorConfig &motor_config, double electrical_rpm)
{
  return electrical_rpm * (2.0 * kPi) / (60.0 * motor_config.gear_ratio * motor_config.pole_pairs) *
         motor_config.direction;
}

double joint_position_to_degrees(const MotorConfig &motor_config, double joint_position_rad)
{
  double motor_pid_position_deg =
    std::fmod((motor_config.zero_offset_rad +
                joint_position_rad * motor_config.gear_ratio * motor_config.direction) *
                180.0 / kPi,
      360.0);
  return motor_pid_position_deg < 0.0 ? motor_pid_position_deg + 360.0 : motor_pid_position_deg;
}

double degrees_to_joint_position(const MotorConfig &motor_config, double motor_pid_position_deg)
{
  return std::remainder(
           motor_pid_position_deg * kPi / 180.0 - motor_config.zero_offset_rad, 2.0 * kPi) *
         motor_config.direction / motor_config.gear_ratio;
}

double joint_current_to_motor_current(const MotorConfig &motor_config, double joint_current_a)
{
  return joint_current_a * motor_config.direction;
}

double motor_current_to_joint_current(const MotorConfig &motor_config, double motor_current_a)
{
  return joint_current_to_motor_current(motor_config, motor_current_a);
}

void validate_configs(const std::vector<MotorConfig> &motor_configs)
{
  if (motor_configs.empty()) {
    throw std::invalid_argument("At least one motor is required");
  }
  std::set<std::string> motor_names;
  std::set<std::uint8_t> controller_ids;
  for (const auto &motor_config : motor_configs) {
    command_interface_for_mode(motor_config.control_mode);  // Reject invalid enum values as well.
    const std::array numeric_settings = {motor_config.pole_pairs, motor_config.gear_ratio,
      motor_config.direction, motor_config.zero_offset_rad, motor_config.max_joint_velocity_rad_s,
      motor_config.max_current_a, motor_config.min_joint_position_rad,
      motor_config.max_joint_position_rad, motor_config.joint_velocity_slew_rate_rad_s2,
      motor_config.startup_current_a, motor_config.rpm_control_threshold_rpm};
    for (double setting_value : numeric_settings) {
      if (!std::isfinite(setting_value)) {
        throw std::invalid_argument(motor_config.name + ": non-finite setting");
      }
    }
    if (motor_config.name.empty() || !motor_names.insert(motor_config.name).second ||
        !controller_ids.insert(motor_config.controller_id).second) {
      throw std::invalid_argument("Motor names and controller IDs must be unique");
    }
    if (motor_config.controller_id == 255 || motor_config.pole_pairs <= 0.0 ||
        std::floor(motor_config.pole_pairs) != motor_config.pole_pairs ||
        motor_config.gear_ratio <= 0.0 || std::abs(motor_config.direction) != 1.0 ||
        motor_config.max_joint_velocity_rad_s <= 0.0 || motor_config.max_current_a <= 0.0 ||
        motor_config.joint_velocity_slew_rate_rad_s2 <= 0.0 ||
        motor_config.startup_current_a < 0.0 || motor_config.rpm_control_threshold_rpm < 0.0 ||
        motor_config.feedback_timeout.count() <= 0 || motor_config.command_timeout.count() <= 0 ||
        motor_config.startup_timeout.count() <= 0) {
      throw std::invalid_argument(motor_config.name + ": invalid motor limits or timing");
    }
    const double max_erpm =
      std::abs(joint_velocity_to_erpm(motor_config, motor_config.max_joint_velocity_rad_s));
    if (uses_velocity_command(motor_config.control_mode) &&
        (!std::isfinite(max_erpm) || max_erpm > std::numeric_limits<std::int32_t>::max())) {
      throw std::invalid_argument(
        motor_config.name + ": speed limit exceeds VESC int32 ERPM range");
    }
    if (motor_config.control_mode == ControlMode::Position &&
        (motor_config.min_joint_position_rad >= motor_config.max_joint_position_rad ||
          motor_config.min_joint_position_rad < -kPi / motor_config.gear_ratio ||
          motor_config.max_joint_position_rad > kPi / motor_config.gear_ratio)) {
      throw std::invalid_argument(
        motor_config.name + ": PID position requires a single-turn range");
    }
    if (motor_config.max_current_a > std::numeric_limits<std::int32_t>::max() / 1000.0 ||
        motor_config.startup_current_a > motor_config.max_current_a) {
      throw std::invalid_argument(motor_config.name + ": invalid current limit or startup current");
    }
    if (motor_config.control_mode == ControlMode::CurrentVelocity) {
      if (motor_config.startup_current_a <= 0.0 || motor_config.rpm_control_threshold_rpm <= 0.0) {
        throw std::invalid_argument(
          motor_config.name +
          ": current_velocity requires positive startup current and RPM threshold");
      }
    } else if (motor_config.startup_current_a != 0.0) {
      throw std::invalid_argument(
        motor_config.name + ": startup current is only supported in current_velocity mode");
    }
  }
}

bool feedback_is_fresh(bool has_feedback, TimePoint received_at, TimePoint current_time,
  std::chrono::milliseconds timeout)
{
  return has_feedback && current_time >= received_at && current_time - received_at <= timeout;
}

bool command_is_valid(const MotorConfig &motor_config, double command_value)
{
  if (!std::isfinite(command_value)) {
    return false;
  }
  switch (motor_config.control_mode) {
    case ControlMode::Current:
      return std::abs(command_value) <= motor_config.max_current_a;
    case ControlMode::Velocity:
    case ControlMode::CurrentVelocity:
      return std::abs(command_value) <= motor_config.max_joint_velocity_rad_s;
    case ControlMode::Position:
      return command_value >= motor_config.min_joint_position_rad &&
             command_value <= motor_config.max_joint_position_rad;
  }
  return false;
}

bool state_ready(
  const MotorConfig &motor_config, const MotorState &motor_state, TimePoint current_time)
{
  constexpr double kPositionToleranceRad = 1e-3;
  const auto timeout = motor_config.feedback_timeout;
  if (!feedback_is_fresh(
        motor_state.has_status_1, motor_state.status_1_received_at, current_time, timeout)) {
    return false;
  }
  if (motor_config.control_mode == ControlMode::Position) {
    if (!feedback_is_fresh(
          motor_state.has_status_4, motor_state.status_4_received_at, current_time, timeout) ||
        !std::isfinite(motor_state.joint_position_rad) ||
        motor_state.joint_position_rad <
          motor_config.min_joint_position_rad - kPositionToleranceRad ||
        motor_state.joint_position_rad >
          motor_config.max_joint_position_rad + kPositionToleranceRad) {
      return false;
    }
  }
  return !motor_config.require_tachometer ||
         feedback_is_fresh(
           motor_state.has_status_5, motor_state.status_5_received_at, current_time, timeout);
}

DriverCore::DriverCore(std::vector<MotorConfig> motor_configs)
  : motor_configs_(std::move(motor_configs)), motor_states_(motor_configs_.size()),
    motor_commands_(motor_configs_.size())
{
  validate_configs(motor_configs_);
}

bool DriverCore::receive(const can_msgs::msg::Frame &frame, TimePoint current_time)
{
  const auto controller_id = static_cast<std::uint8_t>(frame.id & protocol::kControllerIdMask);
  const auto motor_config_it = std::find_if(
    motor_configs_.begin(), motor_configs_.end(), [controller_id](const MotorConfig &motor_config) {
      return motor_config.controller_id == controller_id;
    });
  if (motor_config_it == motor_configs_.end()) {
    return false;
  }
  const auto motor_index = static_cast<std::size_t>(motor_config_it - motor_configs_.begin());
  auto &motor_state = motor_states_[motor_index];
  protocol::Status1 status_1;
  protocol::Status2 status_2;
  protocol::Status3 status_3;
  protocol::Status4 status_4;
  protocol::Status5 status_5;
  protocol::Status6 status_6;
  if (protocol::decode_status_1(frame, status_1)) {
    motor_state.joint_velocity_rad_s =
      erpm_to_joint_velocity(*motor_config_it, status_1.electrical_rpm);
    motor_state.electrical_rpm = status_1.electrical_rpm;
    motor_state.motor_current_a = status_1.motor_current_a;
    motor_state.duty_cycle = status_1.duty_cycle;
    motor_state.status_1_received_at = current_time;
    motor_state.has_status_1 = true;
  } else if (protocol::decode_status_2(frame, status_2)) {
    motor_state.charge_drawn_ah = status_2.charge_drawn_ah;
    motor_state.charge_regen_ah = status_2.charge_regen_ah;
    motor_state.status_2_received_at = current_time;
    motor_state.has_status_2 = true;
  } else if (protocol::decode_status_3(frame, status_3)) {
    motor_state.energy_drawn_wh = status_3.energy_drawn_wh;
    motor_state.energy_regen_wh = status_3.energy_regen_wh;
    motor_state.status_3_received_at = current_time;
    motor_state.has_status_3 = true;
  } else if (protocol::decode_status_4(frame, status_4)) {
    motor_state.temperature_fet_c = status_4.temperature_fet_c;
    motor_state.temperature_motor_c = status_4.temperature_motor_c;
    motor_state.input_current_a = status_4.input_current_a;
    motor_state.motor_pid_position_deg = status_4.motor_pid_position_deg;
    if (motor_config_it->control_mode == ControlMode::Position) {
      motor_state.joint_position_rad =
        degrees_to_joint_position(*motor_config_it, status_4.motor_pid_position_deg);
    }
    motor_state.status_4_received_at = current_time;
    motor_state.has_status_4 = true;
  } else if (protocol::decode_status_5(frame, status_5)) {
    motor_state.tachometer_counts = status_5.tachometer_counts;
    if (motor_config_it->control_mode != ControlMode::Position) {
      motor_state.joint_position_rad =
        status_5.tachometer_counts * (2.0 * kPi) /
        (6.0 * motor_config_it->pole_pairs * motor_config_it->gear_ratio) *
        motor_config_it->direction;
    }
    motor_state.input_voltage_v = status_5.input_voltage_v;
    motor_state.status_5_received_at = current_time;
    motor_state.has_status_5 = true;
  } else if (protocol::decode_status_6(frame, status_6)) {
    motor_state.adc_1_v = status_6.adc_1_v;
    motor_state.adc_2_v = status_6.adc_2_v;
    motor_state.adc_3_v = status_6.adc_3_v;
    motor_state.ppm = status_6.ppm;
    motor_state.status_6_received_at = current_time;
    motor_state.has_status_6 = true;
  } else {
    return false;
  }
  return true;
}

bool DriverCore::ready(TimePoint current_time) const
{
  for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
    if (!state_ready(motor_configs_[motor_index], motor_states_[motor_index], current_time)) {
      return false;
    }
  }
  return true;
}

bool DriverCore::enable(TimePoint current_time)
{
  if (!ready(current_time)) {
    return false;
  }
  enabled_ = true;
  faulted_ = false;
  fault_reason_.clear();
  enabled_at_ = current_time;
  std::fill(motor_commands_.begin(), motor_commands_.end(), Command{});
  return true;
}

void DriverCore::disable()
{
  enabled_ = false;
  std::fill(motor_commands_.begin(), motor_commands_.end(), Command{});
}

void DriverCore::stop_on_error(const std::string &reason)
{
  faulted_ = true;
  enabled_ = false;
  fault_reason_ = reason;
}

bool DriverCore::set_command(std::size_t motor_index, double command_value, TimePoint current_time)
{
  if (motor_index >= motor_configs_.size() || !enabled_ || faulted_) {
    return false;
  }
  const auto &motor_config = motor_configs_[motor_index];
  if (!command_is_valid(motor_config, command_value)) {
    stop_on_error(motor_config.name + ": invalid command");
    return false;
  }
  auto &motor_command = motor_commands_[motor_index];
  if (motor_config.control_mode == ControlMode::CurrentVelocity &&
      (!motor_command.has_command || motor_command.target_value * command_value <= 0.0)) {
    motor_command.rpm_control_active = false;
    motor_command.startup_started_at = current_time;
  }
  motor_command.target_value = command_value;
  motor_command.received_at = current_time;
  if (!motor_command.has_command) {
    motor_command.last_ramp_update_at = current_time;
  }
  motor_command.has_command = true;
  return true;
}

void DriverCore::stop_frames(std::vector<can_msgs::msg::Frame> &frames) const
{
  frames.clear();
  for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
    frames.push_back(make_current_command(motor_index, 0.0));
  }
}

void DriverCore::command_frames(TimePoint current_time, std::vector<can_msgs::msg::Frame> &frames)
{
  frames.clear();
  if (enabled_) {
    for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
      const auto &motor_config = motor_configs_[motor_index];
      const auto &motor_command = motor_commands_[motor_index];
      if (!state_ready(motor_config, motor_states_[motor_index], current_time)) {
        stop_on_error(motor_config.name + ": feedback stale or position outside limits");
        break;
      }
      if (current_time - (motor_command.has_command ? motor_command.received_at : enabled_at_) >
          motor_config.command_timeout) {
        stop_on_error(motor_config.name + ": command timeout");
        break;
      }
    }
  }
  if (!enabled_ || faulted_) {
    stop_frames(frames);
    return;
  }

  for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
    const auto frame = make_command(motor_index, current_time);
    if (!frame) {
      stop_frames(frames);
      return;
    }
    frames.push_back(*frame);
    motor_commands_[motor_index].last_ramp_update_at = current_time;
  }
}

std::optional<can_msgs::msg::Frame> DriverCore::make_command(
  std::size_t motor_index, TimePoint current_time)
{
  const auto &motor_config = motor_configs_[motor_index];
  auto &motor_command = motor_commands_[motor_index];
  if (!motor_command.has_command ||
      (motor_config.control_mode != ControlMode::Position && motor_command.target_value == 0.0)) {
    motor_command.ramped_joint_velocity_rad_s = 0.0;
    motor_command.rpm_control_active = false;
    return make_current_command(motor_index, 0.0);
  }
  switch (motor_config.control_mode) {
    case ControlMode::Current:
      return make_current_command(motor_index, motor_command.target_value);
    case ControlMode::Velocity:
      return make_velocity_command(motor_index, current_time);
    case ControlMode::Position:
      return protocol::make_set_position_frame(motor_config.controller_id,
        joint_position_to_degrees(motor_config, motor_command.target_value));
    case ControlMode::CurrentVelocity:
      return make_current_velocity_command(motor_index, current_time);
  }
  stop_on_error(motor_config.name + ": unsupported control mode");
  return std::nullopt;
}

can_msgs::msg::Frame DriverCore::make_current_command(
  std::size_t motor_index, double joint_current_a) const
{
  const auto &motor_config = motor_configs_[motor_index];
  return protocol::make_set_current_frame(
    motor_config.controller_id, joint_current_to_motor_current(motor_config, joint_current_a));
}

std::optional<can_msgs::msg::Frame> DriverCore::make_current_velocity_command(
  std::size_t motor_index, TimePoint current_time)
{
  const auto &motor_config = motor_configs_[motor_index];
  const auto &motor_state = motor_states_[motor_index];
  auto &motor_command = motor_commands_[motor_index];
  const double measured_motor_rpm =
    std::abs(motor_state.joint_velocity_rad_s) * motor_config.gear_ratio * 60.0 / (2.0 * kPi);
  const double target_motor_rpm =
    std::abs(motor_command.target_value) * motor_config.gear_ratio * 60.0 / (2.0 * kPi);
  const bool reached_rpm_threshold =
    motor_command.target_value * motor_state.joint_velocity_rad_s > 0.0 &&
    measured_motor_rpm >= std::min(target_motor_rpm, motor_config.rpm_control_threshold_rpm);

  if (!motor_command.rpm_control_active && reached_rpm_threshold) {
    motor_command.rpm_control_active = true;
    motor_command.ramped_joint_velocity_rad_s = std::clamp(motor_state.joint_velocity_rad_s,
      -motor_config.max_joint_velocity_rad_s, motor_config.max_joint_velocity_rad_s);
  }
  if (!motor_command.rpm_control_active) {
    if (current_time - motor_command.startup_started_at > motor_config.startup_timeout) {
      stop_on_error(motor_config.name + ": startup timeout");
      return std::nullopt;
    }
    return make_current_command(
      motor_index, std::copysign(motor_config.startup_current_a, motor_command.target_value));
  }
  return make_velocity_command(motor_index, current_time);
}

can_msgs::msg::Frame DriverCore::make_velocity_command(
  std::size_t motor_index, TimePoint current_time)
{
  const auto &motor_config = motor_configs_[motor_index];
  auto &motor_command = motor_commands_[motor_index];
  const double elapsed_seconds = std::max(
    0.0, std::chrono::duration<double>(current_time - motor_command.last_ramp_update_at).count());
  const double max_joint_velocity_change_rad_s =
    motor_config.joint_velocity_slew_rate_rad_s2 * elapsed_seconds;
  motor_command.ramped_joint_velocity_rad_s +=
    std::clamp(motor_command.target_value - motor_command.ramped_joint_velocity_rad_s,
      -max_joint_velocity_change_rad_s, max_joint_velocity_change_rad_s);
  const auto electrical_rpm = static_cast<std::int32_t>(
    std::llround(joint_velocity_to_erpm(motor_config, motor_command.ramped_joint_velocity_rad_s)));
  return protocol::make_set_rpm_frame(motor_config.controller_id, electrical_rpm);
}
}  // namespace vesc_can
