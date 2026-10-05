#include "vesc_can_ros2_control/driver_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace vesc_can_ros2_control
{
double joint_velocity_to_erpm(const MotorConfig &motor_config, double velocity)
{
  return velocity * 60.0 / (2.0 * kPi) * motor_config.gear_ratio * motor_config.pole_pairs *
         motor_config.direction;
}

double erpm_to_joint_velocity(const MotorConfig &motor_config, double erpm)
{
  return erpm * (2.0 * kPi) / (60.0 * motor_config.gear_ratio * motor_config.pole_pairs) *
         motor_config.direction;
}

double joint_position_to_degrees(const MotorConfig &motor_config, double position)
{
  double angle = std::fmod(
    (motor_config.zero_offset_rad + position * motor_config.gear_ratio * motor_config.direction) *
      180.0 / kPi,
    360.0);
  return angle < 0.0 ? angle + 360.0 : angle;
}

double degrees_to_joint_position(const MotorConfig &motor_config, double degrees)
{
  return std::remainder(degrees * kPi / 180.0 - motor_config.zero_offset_rad, 2.0 * kPi) *
         motor_config.direction / motor_config.gear_ratio;
}

void validate_configs(const std::vector<MotorConfig> &motor_configs)
{
  if (motor_configs.empty()) {
    throw std::invalid_argument("At least one motor is required");
  }
  std::set<std::string> names;
  std::set<std::uint8_t> controller_ids;
  for (const auto &motor_config : motor_configs) {
    const std::array numeric_settings = {motor_config.pole_pairs, motor_config.gear_ratio,
      motor_config.direction, motor_config.zero_offset_rad, motor_config.max_velocity,
      motor_config.min_position, motor_config.max_position, motor_config.velocity_slew_rate,
      motor_config.startup_current_a, motor_config.rpm_control_threshold_rpm};
    for (double setting : numeric_settings) {
      if (!std::isfinite(setting)) {
        throw std::invalid_argument(motor_config.name + ": non-finite setting");
      }
    }
    if (motor_config.name.empty() || !names.insert(motor_config.name).second ||
        !controller_ids.insert(motor_config.controller_id).second) {
      throw std::invalid_argument("Motor names and controller IDs must be unique");
    }
    if (motor_config.controller_id == 255 || motor_config.pole_pairs <= 0.0 ||
        std::floor(motor_config.pole_pairs) != motor_config.pole_pairs ||
        motor_config.gear_ratio <= 0.0 || std::abs(motor_config.direction) != 1.0 ||
        motor_config.max_velocity <= 0.0 || motor_config.velocity_slew_rate <= 0.0 ||
        motor_config.startup_current_a < 0.0 || motor_config.rpm_control_threshold_rpm < 0.0 ||
        motor_config.feedback_timeout.count() <= 0 || motor_config.command_timeout.count() <= 0 ||
        motor_config.startup_timeout.count() <= 0) {
      throw std::invalid_argument(motor_config.name + ": invalid motor limits or timing");
    }
    const double max_erpm =
      std::abs(joint_velocity_to_erpm(motor_config, motor_config.max_velocity));
    if (!std::isfinite(max_erpm) || max_erpm > std::numeric_limits<std::int32_t>::max()) {
      throw std::invalid_argument(
        motor_config.name + ": speed limit exceeds VESC int32 ERPM range");
    }
    if (motor_config.mode == ControlMode::Position &&
        (motor_config.min_position >= motor_config.max_position ||
          motor_config.min_position < -kPi / motor_config.gear_ratio ||
          motor_config.max_position > kPi / motor_config.gear_ratio ||
          motor_config.startup_current_a != 0.0)) {
      throw std::invalid_argument(
        motor_config.name + ": PID position requires a single-turn range");
    }
    if (motor_config.startup_current_a > std::numeric_limits<std::int32_t>::max() / 1000.0) {
      throw std::invalid_argument(motor_config.name + ": current exceeds VESC int32 range");
    }
  }
}

bool feedback_is_fresh(
  bool has_feedback, TimePoint received_at, TimePoint now, std::chrono::milliseconds timeout)
{
  return has_feedback && now >= received_at && now - received_at <= timeout;
}

bool command_is_valid(const MotorConfig &motor_config, double command)
{
  if (!std::isfinite(command)) {
    return false;
  }
  if (motor_config.mode == ControlMode::Velocity) {
    return std::abs(command) <= motor_config.max_velocity;
  }
  return command >= motor_config.min_position && command <= motor_config.max_position;
}

bool state_ready(const MotorConfig &motor_config, const MotorState &motor_state, TimePoint now)
{
  constexpr double kPositionToleranceRad = 1e-3;
  const auto timeout = motor_config.feedback_timeout;
  if (!feedback_is_fresh(motor_state.have_status1, motor_state.status1_time, now, timeout)) {
    return false;
  }
  if (motor_config.mode == ControlMode::Position) {
    if (!feedback_is_fresh(motor_state.have_status4, motor_state.status4_time, now, timeout) ||
        !std::isfinite(motor_state.position) ||
        motor_state.position < motor_config.min_position - kPositionToleranceRad ||
        motor_state.position > motor_config.max_position + kPositionToleranceRad) {
      return false;
    }
  }
  return !motor_config.require_tachometer ||
         feedback_is_fresh(motor_state.have_status5, motor_state.status5_time, now, timeout);
}

DriverCore::DriverCore(std::vector<MotorConfig> motor_configs)
  : motor_configs_(std::move(motor_configs)), motor_states_(motor_configs_.size()),
    motor_commands_(motor_configs_.size())
{
  validate_configs(motor_configs_);
}

bool DriverCore::receive(const can_msgs::msg::Frame &frame, TimePoint now)
{
  const auto controller_id = static_cast<std::uint8_t>(frame.id & protocol::kControllerIdMask);
  const auto config_iterator = std::find_if(
    motor_configs_.begin(), motor_configs_.end(), [controller_id](const MotorConfig &motor_config) {
      return motor_config.controller_id == controller_id;
    });
  if (config_iterator == motor_configs_.end()) {
    return false;
  }
  const auto motor_index = static_cast<std::size_t>(config_iterator - motor_configs_.begin());
  auto &motor_state = motor_states_[motor_index];
  protocol::Status1 status_1;
  protocol::Status2 status_2;
  protocol::Status3 status_3;
  protocol::Status4 status_4;
  protocol::Status5 status_5;
  protocol::Status6 status_6;
  if (protocol::decode_status_1(frame, status_1)) {
    motor_state.velocity = erpm_to_joint_velocity(*config_iterator, status_1.erpm);
    motor_state.electrical_rpm = status_1.erpm;
    motor_state.current_a = status_1.current_a;
    motor_state.duty_cycle = status_1.duty_cycle;
    motor_state.status1_time = now;
    motor_state.have_status1 = true;
  } else if (protocol::decode_status_2(frame, status_2)) {
    motor_state.charge_drawn_ah = status_2.charge_drawn_ah;
    motor_state.charge_regen_ah = status_2.charge_regen_ah;
    motor_state.status2_time = now;
    motor_state.have_status2 = true;
  } else if (protocol::decode_status_3(frame, status_3)) {
    motor_state.energy_drawn_wh = status_3.energy_drawn_wh;
    motor_state.energy_regen_wh = status_3.energy_regen_wh;
    motor_state.status3_time = now;
    motor_state.have_status3 = true;
  } else if (protocol::decode_status_4(frame, status_4)) {
    motor_state.temperature_fet_c = status_4.temperature_fet_c;
    motor_state.temperature_motor_c = status_4.temperature_motor_c;
    motor_state.input_current_a = status_4.input_current_a;
    motor_state.pid_position_deg = status_4.position_deg;
    if (config_iterator->mode == ControlMode::Position) {
      motor_state.position = degrees_to_joint_position(*config_iterator, status_4.position_deg);
    }
    motor_state.status4_time = now;
    motor_state.have_status4 = true;
  } else if (protocol::decode_status_5(frame, status_5)) {
    motor_state.tachometer_counts = status_5.tachometer;
    if (config_iterator->mode == ControlMode::Velocity) {
      motor_state.position = status_5.tachometer * (2.0 * kPi) /
                             (6.0 * config_iterator->pole_pairs * config_iterator->gear_ratio) *
                             config_iterator->direction;
    }
    motor_state.input_voltage_v = status_5.input_voltage_v;
    motor_state.status5_time = now;
    motor_state.have_status5 = true;
  } else if (protocol::decode_status_6(frame, status_6)) {
    motor_state.adc_1_v = status_6.adc_1_v;
    motor_state.adc_2_v = status_6.adc_2_v;
    motor_state.adc_3_v = status_6.adc_3_v;
    motor_state.ppm = status_6.ppm;
    motor_state.status6_time = now;
    motor_state.have_status6 = true;
  } else {
    return false;
  }
  return true;
}

bool DriverCore::ready(TimePoint now) const
{
  for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
    if (!state_ready(motor_configs_[motor_index], motor_states_[motor_index], now)) {
      return false;
    }
  }
  return true;
}

bool DriverCore::enable(TimePoint now)
{
  if (!ready(now)) {
    return false;
  }
  enabled_ = true;
  faulted_ = false;
  fault_reason_.clear();
  enabled_time_ = now;
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

bool DriverCore::set_command(std::size_t motor_index, double value, TimePoint now)
{
  if (motor_index >= motor_configs_.size() || !enabled_ || faulted_) {
    return false;
  }
  const auto &motor_config = motor_configs_[motor_index];
  if (!command_is_valid(motor_config, value)) {
    stop_on_error(motor_config.name + ": invalid command");
    return false;
  }
  auto &motor_command = motor_commands_[motor_index];
  if (!motor_command.has_command || motor_command.target * value <= 0.0) {
    motor_command.rpm_control_active = false;
    motor_command.startup_started_at = now;
  }
  motor_command.target = value;
  motor_command.received_at = now;
  if (!motor_command.has_command) {
    motor_command.last_ramp_update = now;
  }
  motor_command.has_command = true;
  return true;
}

void DriverCore::stop_frames(std::vector<can_msgs::msg::Frame> &frames) const
{
  frames.clear();
  for (const auto &motor_config : motor_configs_) {
    frames.push_back(protocol::make_set_current_frame(motor_config.controller_id, 0.0));
  }
}

void DriverCore::command_frames(TimePoint now, std::vector<can_msgs::msg::Frame> &frames)
{
  frames.clear();
  if (enabled_) {
    for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
      const auto &motor_config = motor_configs_[motor_index];
      const auto &motor_command = motor_commands_[motor_index];
      if (!state_ready(motor_config, motor_states_[motor_index], now)) {
        stop_on_error(motor_config.name + ": feedback stale or position outside limits");
        break;
      }
      if (now - (motor_command.has_command ? motor_command.received_at : enabled_time_) >
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
    const auto &motor_config = motor_configs_[motor_index];
    auto &motor_command = motor_commands_[motor_index];
    if (!motor_command.has_command) {
      frames.push_back(protocol::make_set_current_frame(motor_config.controller_id, 0.0));
    } else if (motor_config.mode == ControlMode::Position) {
      frames.push_back(protocol::make_set_position_frame(
        motor_config.controller_id, joint_position_to_degrees(motor_config, motor_command.target)));
    } else if (motor_command.target == 0.0) {
      motor_command.ramped_velocity = 0.0;
      motor_command.rpm_control_active = false;
      frames.push_back(protocol::make_set_current_frame(motor_config.controller_id, 0.0));
    } else {
      const auto velocity_frame = make_velocity_command(motor_index, now);
      if (!velocity_frame) {
        stop_frames(frames);
        return;
      }
      frames.push_back(*velocity_frame);
    }
    motor_command.last_ramp_update = now;
  }
}

std::optional<can_msgs::msg::Frame> DriverCore::make_velocity_command(
  std::size_t motor_index, TimePoint now)
{
  const auto &motor_config = motor_configs_[motor_index];
  const auto &motor_state = motor_states_[motor_index];
  auto &motor_command = motor_commands_[motor_index];
  const double measured_motor_rpm =
    std::abs(motor_state.velocity) * motor_config.gear_ratio * 60.0 / (2.0 * kPi);
  const double target_motor_rpm =
    std::abs(motor_command.target) * motor_config.gear_ratio * 60.0 / (2.0 * kPi);
  const bool use_current_start =
    motor_config.startup_current_a > 0.0 && motor_config.rpm_control_threshold_rpm > 0.0;
  const bool reached_rpm_threshold =
    motor_command.target * motor_state.velocity > 0.0 &&
    measured_motor_rpm >= std::min(target_motor_rpm, motor_config.rpm_control_threshold_rpm);

  if (!motor_command.rpm_control_active && (!use_current_start || reached_rpm_threshold)) {
    motor_command.rpm_control_active = true;
    motor_command.ramped_velocity = use_current_start ? motor_state.velocity : 0.0;
  }
  if (!motor_command.rpm_control_active) {
    if (now - motor_command.startup_started_at > motor_config.startup_timeout) {
      stop_on_error(motor_config.name + ": startup timeout");
      return std::nullopt;
    }
    const double startup_current =
      std::copysign(motor_config.startup_current_a, motor_command.target * motor_config.direction);
    return protocol::make_set_current_frame(motor_config.controller_id, startup_current);
  }

  const double elapsed_seconds =
    std::max(0.0, std::chrono::duration<double>(now - motor_command.last_ramp_update).count());
  const double max_velocity_change = motor_config.velocity_slew_rate * elapsed_seconds;
  motor_command.ramped_velocity += std::clamp(motor_command.target - motor_command.ramped_velocity,
    -max_velocity_change, max_velocity_change);
  const auto erpm = static_cast<std::int32_t>(
    std::llround(joint_velocity_to_erpm(motor_config, motor_command.ramped_velocity)));
  return protocol::make_set_rpm_frame(motor_config.controller_id, erpm);
}
}  // namespace vesc_can_ros2_control
