#include "vesc_can_driver/driver_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace vesc_can_driver
{
ControlMode parse_control_mode(std::string_view mode_name)
{
  if (mode_name == "current") {
    return ControlMode::Current;
  }
  if (mode_name == "velocity") {
    return ControlMode::Velocity;
  }
  if (mode_name == "position") {
    return ControlMode::Position;
  }
  if (mode_name == "current_velocity") {
    return ControlMode::CurrentVelocity;
  }
  throw std::invalid_argument("Unsupported control mode: " + std::string(mode_name));
}

std::string_view command_interface(ControlMode mode)
{
  switch (mode) {
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

ControlMode resolve_control_mode(std::string_view mode_name, std::string_view interface_name)
{
  const auto resolved_mode = parse_control_mode(mode_name.empty() ? interface_name : mode_name);
  if (command_interface(resolved_mode) != interface_name) {
    throw std::invalid_argument("Control mode and command interface do not match");
  }
  return resolved_mode;
}

bool is_velocity_mode(ControlMode mode)
{
  return mode == ControlMode::Velocity || mode == ControlMode::CurrentVelocity;
}

double vel_to_erpm(const MotorConfig &cfg, double vel_rad_s)
{
  return vel_rad_s * 60.0 / (2.0 * kPi) * cfg.gear_ratio * cfg.pole_pairs * cfg.direction;
}

double erpm_to_vel(const MotorConfig &cfg, double electrical_rpm)
{
  return electrical_rpm * (2.0 * kPi) / (60.0 * cfg.gear_ratio * cfg.pole_pairs) * cfg.direction;
}

double pos_to_deg(const MotorConfig &cfg, double pos_rad)
{
  double pid_pos_deg = std::fmod(
    (cfg.zero_offset_rad + pos_rad * cfg.gear_ratio * cfg.direction) * 180.0 / kPi, 360.0);
  return pid_pos_deg < 0.0 ? pid_pos_deg + 360.0 : pid_pos_deg;
}

double deg_to_pos(const MotorConfig &cfg, double pid_pos_deg)
{
  return std::remainder(pid_pos_deg * kPi / 180.0 - cfg.zero_offset_rad, 2.0 * kPi) *
         cfg.direction / cfg.gear_ratio;
}

double apply_direction(const MotorConfig &cfg, double current_a)
{
  return current_a * cfg.direction;
}

namespace
{
void validate_config(const MotorConfig &cfg)
{
  command_interface(cfg.mode);  // Reject invalid enum values as well.
  const std::array numeric_settings = {cfg.pole_pairs, cfg.gear_ratio, cfg.direction,
    cfg.zero_offset_rad, cfg.max_vel_rad_s, cfg.max_current_a, cfg.min_pos_rad, cfg.max_pos_rad,
    cfg.vel_slew_rad_s2, cfg.startup_current_a, cfg.rpm_threshold};
  for (double setting_value : numeric_settings) {
    if (!std::isfinite(setting_value)) {
      throw std::invalid_argument(cfg.name + ": non-finite setting");
    }
  }
  if (cfg.controller_id == 255 || cfg.pole_pairs <= 0.0 ||
      std::floor(cfg.pole_pairs) != cfg.pole_pairs || cfg.gear_ratio <= 0.0 ||
      std::abs(cfg.direction) != 1.0 || cfg.max_vel_rad_s <= 0.0 || cfg.max_current_a <= 0.0 ||
      cfg.vel_slew_rad_s2 <= 0.0 || cfg.startup_current_a < 0.0 || cfg.rpm_threshold < 0.0 ||
      cfg.feedback_timeout.count() <= 0 || cfg.command_timeout.count() <= 0 ||
      cfg.startup_timeout.count() <= 0) {
    throw std::invalid_argument(cfg.name + ": invalid motor limits or timing");
  }
  const double max_erpm = std::abs(vel_to_erpm(cfg, cfg.max_vel_rad_s));
  if (is_velocity_mode(cfg.mode) &&
      (!std::isfinite(max_erpm) || max_erpm > std::numeric_limits<std::int32_t>::max())) {
    throw std::invalid_argument(cfg.name + ": speed limit exceeds VESC int32 ERPM range");
  }
  if (cfg.mode == ControlMode::Position &&
      (cfg.min_pos_rad >= cfg.max_pos_rad || cfg.min_pos_rad < -kPi / cfg.gear_ratio ||
        cfg.max_pos_rad > kPi / cfg.gear_ratio)) {
    throw std::invalid_argument(cfg.name + ": PID position requires a single-turn range");
  }
  if (cfg.max_current_a > std::numeric_limits<std::int32_t>::max() / 1000.0 ||
      cfg.startup_current_a > cfg.max_current_a) {
    throw std::invalid_argument(cfg.name + ": invalid current limit or startup current");
  }
  if (cfg.mode == ControlMode::CurrentVelocity) {
    if (cfg.startup_current_a <= 0.0 || cfg.rpm_threshold <= 0.0) {
      throw std::invalid_argument(
        cfg.name + ": current_velocity requires positive startup current and RPM threshold");
    }
  } else if (cfg.startup_current_a != 0.0) {
    throw std::invalid_argument(
      cfg.name + ": startup current is only supported in current_velocity mode");
  }
}
}  // namespace

void validate_configs(const std::vector<MotorConfig> &configs)
{
  if (configs.empty()) {
    throw std::invalid_argument("At least one motor is required");
  }
  std::set<std::string> names;
  std::set<std::uint8_t> ids;
  for (const auto &cfg : configs) {
    validate_config(cfg);
    if (cfg.name.empty() || !names.insert(cfg.name).second ||
        !ids.insert(cfg.controller_id).second) {
      throw std::invalid_argument("Motor names and controller IDs must be unique");
    }
  }
}

bool feedback_fresh(
  const std::optional<TimePoint> &received_at, TimePoint now, std::chrono::milliseconds timeout)
{
  return received_at && now >= *received_at && now - *received_at <= timeout;
}

bool command_is_valid(const MotorConfig &cfg, double value)
{
  if (!std::isfinite(value)) {
    return false;
  }
  switch (cfg.mode) {
    case ControlMode::Current:
      return std::abs(value) <= cfg.max_current_a;
    case ControlMode::Velocity:
    case ControlMode::CurrentVelocity:
      return std::abs(value) <= cfg.max_vel_rad_s;
    case ControlMode::Position:
      return value >= cfg.min_pos_rad && value <= cfg.max_pos_rad;
  }
  return false;
}

bool state_ready(const MotorConfig &cfg, const MotorState &state, TimePoint now)
{
  constexpr double kPositionToleranceRad = 1e-3;
  const auto timeout = cfg.feedback_timeout;
  if (!feedback_fresh(state.status1_at, now, timeout)) {
    return false;
  }
  if (cfg.mode == ControlMode::Position) {
    if (!feedback_fresh(state.status4_at, now, timeout) || !std::isfinite(state.pos_rad) ||
        state.pos_rad < cfg.min_pos_rad - kPositionToleranceRad ||
        state.pos_rad > cfg.max_pos_rad + kPositionToleranceRad) {
      return false;
    }
  }
  return !cfg.require_tachometer || feedback_fresh(state.status5_at, now, timeout);
}

DriverCore::DriverCore(std::vector<MotorConfig> configs)
  : configs_(std::move(configs)), states_(configs_.size()), commands_(configs_.size())
{
  validate_configs(configs_);
}

bool DriverCore::receive(const can_msgs::msg::Frame &frame, TimePoint now)
{
  const auto controller_id = static_cast<std::uint8_t>(frame.id & protocol::kControllerIdMask);
  const auto config_it = std::find_if(configs_.begin(), configs_.end(),
    [controller_id](const MotorConfig &cfg) { return cfg.controller_id == controller_id; });
  if (config_it == configs_.end()) {
    return false;
  }
  const auto idx = static_cast<std::size_t>(config_it - configs_.begin());
  auto &state = states_[idx];
  protocol::Status1 status_1;
  protocol::Status2 status_2;
  protocol::Status3 status_3;
  protocol::Status4 status_4;
  protocol::Status5 status_5;
  protocol::Status6 status_6;
  if (protocol::decode_status_1(frame, status_1)) {
    state.vel_rad_s = erpm_to_vel(*config_it, status_1.electrical_rpm);
    state.electrical_rpm = status_1.electrical_rpm;
    state.motor_current_a = status_1.motor_current_a;
    state.duty_cycle = status_1.duty_cycle;
    state.status1_at = now;
  } else if (protocol::decode_status_2(frame, status_2)) {
    state.charge_drawn_ah = status_2.charge_drawn_ah;
    state.charge_regen_ah = status_2.charge_regen_ah;
    state.status2_at = now;
  } else if (protocol::decode_status_3(frame, status_3)) {
    state.energy_drawn_wh = status_3.energy_drawn_wh;
    state.energy_regen_wh = status_3.energy_regen_wh;
    state.status3_at = now;
  } else if (protocol::decode_status_4(frame, status_4)) {
    state.temperature_fet_c = status_4.temperature_fet_c;
    state.temperature_motor_c = status_4.temperature_motor_c;
    state.input_current_a = status_4.input_current_a;
    state.pid_pos_deg = status_4.motor_pid_position_deg;
    if (config_it->mode == ControlMode::Position) {
      state.pos_rad = deg_to_pos(*config_it, status_4.motor_pid_position_deg);
    }
    state.status4_at = now;
  } else if (protocol::decode_status_5(frame, status_5)) {
    state.tachometer_counts = status_5.tachometer_counts;
    if (config_it->mode != ControlMode::Position) {
      state.pos_rad = status_5.tachometer_counts * (2.0 * kPi) /
                      (6.0 * config_it->pole_pairs * config_it->gear_ratio) * config_it->direction;
    }
    state.input_voltage_v = status_5.input_voltage_v;
    state.status5_at = now;
  } else if (protocol::decode_status_6(frame, status_6)) {
    state.adc_1_v = status_6.adc_1_v;
    state.adc_2_v = status_6.adc_2_v;
    state.adc_3_v = status_6.adc_3_v;
    state.ppm = status_6.ppm;
    state.status6_at = now;
  } else {
    return false;
  }
  return true;
}

bool DriverCore::ready(TimePoint now) const
{
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    if (!state_ready(configs_[idx], states_[idx], now)) {
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
  enabled_at_ = now;
  std::fill(commands_.begin(), commands_.end(), Command{});
  return true;
}

void DriverCore::disable()
{
  enabled_ = false;
  std::fill(commands_.begin(), commands_.end(), Command{});
}

void DriverCore::stop_on_error(const std::string &reason)
{
  disable();
  faulted_ = true;
  fault_reason_ = reason;
}

bool DriverCore::set_command(std::size_t idx, double value, TimePoint now)
{
  if (idx >= configs_.size() || !enabled_) {
    return false;
  }
  const auto &cfg = configs_[idx];
  if (!command_is_valid(cfg, value)) {
    stop_on_error(cfg.name + ": invalid command");
    return false;
  }
  auto &cmd = commands_[idx];
  if (cfg.mode == ControlMode::CurrentVelocity && (!cmd.received_at || cmd.target * value <= 0.0)) {
    cmd.rpm_control_active = false;
    cmd.startup_at = now;
  }
  if (!cmd.received_at) {
    cmd.ramp_at = now;
  }
  cmd.target = value;
  cmd.received_at = now;
  return true;
}

void DriverCore::stop_frames(std::vector<can_msgs::msg::Frame> &frames) const
{
  frames.clear();
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    frames.push_back(make_current_cmd(idx, 0.0));
  }
}

bool DriverCore::check_commands(TimePoint now)
{
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    const auto &cfg = configs_[idx];
    const auto &cmd = commands_[idx];
    if (!state_ready(cfg, states_[idx], now)) {
      stop_on_error(cfg.name + ": feedback stale or position outside limits");
      return false;
    }
    if (now - cmd.received_at.value_or(enabled_at_) > cfg.command_timeout) {
      stop_on_error(cfg.name + ": command timeout");
      return false;
    }
  }
  return true;
}

void DriverCore::command_frames(TimePoint now, std::vector<can_msgs::msg::Frame> &frames)
{
  if (!enabled_ || !check_commands(now)) {
    stop_frames(frames);
    return;
  }
  frames.clear();
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    const auto frame = make_cmd(idx, now);
    if (!frame) {
      stop_frames(frames);
      return;
    }
    frames.push_back(*frame);
    commands_[idx].ramp_at = now;
  }
}

std::optional<can_msgs::msg::Frame> DriverCore::make_cmd(std::size_t idx, TimePoint now)
{
  const auto &cfg = configs_[idx];
  auto &cmd = commands_[idx];
  if (!cmd.received_at || (cfg.mode != ControlMode::Position && cmd.target == 0.0)) {
    cmd.ramp_vel = 0.0;
    cmd.rpm_control_active = false;
    return make_current_cmd(idx, 0.0);
  }
  switch (cfg.mode) {
    case ControlMode::Current:
      return make_current_cmd(idx, cmd.target);
    case ControlMode::Velocity:
      return make_velocity_cmd(idx, now);
    case ControlMode::Position:
      return protocol::make_set_position_frame(cfg.controller_id, pos_to_deg(cfg, cmd.target));
    case ControlMode::CurrentVelocity:
      return make_hybrid_cmd(idx, now);
  }
  stop_on_error(cfg.name + ": unsupported control mode");
  return std::nullopt;
}

can_msgs::msg::Frame DriverCore::make_current_cmd(std::size_t idx, double joint_current_a) const
{
  const auto &cfg = configs_[idx];
  return protocol::make_set_current_frame(cfg.controller_id, apply_direction(cfg, joint_current_a));
}

std::optional<can_msgs::msg::Frame> DriverCore::make_hybrid_cmd(std::size_t idx, TimePoint now)
{
  const auto &cfg = configs_[idx];
  const auto &state = states_[idx];
  auto &cmd = commands_[idx];
  const double measured_rpm = std::abs(state.vel_rad_s) * cfg.gear_ratio * 60.0 / (2.0 * kPi);
  const double target_rpm = std::abs(cmd.target) * cfg.gear_ratio * 60.0 / (2.0 * kPi);
  const bool reached_threshold =
    cmd.target * state.vel_rad_s > 0.0 && measured_rpm >= std::min(target_rpm, cfg.rpm_threshold);

  if (!cmd.rpm_control_active && reached_threshold) {
    cmd.rpm_control_active = true;
    cmd.ramp_vel = std::clamp(state.vel_rad_s, -cfg.max_vel_rad_s, cfg.max_vel_rad_s);
  }
  if (!cmd.rpm_control_active) {
    if (now - cmd.startup_at > cfg.startup_timeout) {
      stop_on_error(cfg.name + ": startup timeout");
      return std::nullopt;
    }
    return make_current_cmd(idx, std::copysign(cfg.startup_current_a, cmd.target));
  }
  return make_velocity_cmd(idx, now);
}

can_msgs::msg::Frame DriverCore::make_velocity_cmd(std::size_t idx, TimePoint now)
{
  const auto &cfg = configs_[idx];
  auto &cmd = commands_[idx];
  const double dt_s = std::max(0.0, std::chrono::duration<double>(now - cmd.ramp_at).count());
  const double max_step = cfg.vel_slew_rad_s2 * dt_s;
  cmd.ramp_vel += std::clamp(cmd.target - cmd.ramp_vel, -max_step, max_step);
  const auto electrical_rpm =
    static_cast<std::int32_t>(std::llround(vel_to_erpm(cfg, cmd.ramp_vel)));
  return protocol::make_set_rpm_frame(cfg.controller_id, electrical_rpm);
}
}  // namespace vesc_can_driver
