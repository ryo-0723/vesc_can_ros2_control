#include "vesc_can_driver/motor_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <type_traits>
#include <utility>
namespace vesc_can_driver
{
// Mode selection and unit conversion.
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
  if (mode_name == "duty_cycle") {
    return ControlMode::DutyCycle;
  }
  throw std::invalid_argument("Unsupported control mode: " + std::string(mode_name));
}

std::string_view command_interface(ControlMode mode)
{
  switch (mode) {
    case ControlMode::Current:
      return "current";
    case ControlMode::Velocity:
      return "velocity";
    case ControlMode::CurrentVelocity:
      return "current_velocity";
    case ControlMode::Position:
      return "position";
    case ControlMode::DutyCycle:
      return "duty_cycle";
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

double tacho_to_pos(const MotorConfig &cfg, std::int32_t counts)
{
  return counts * (2.0 * kPi) / (6.0 * cfg.pole_pairs * cfg.gear_ratio) * cfg.direction;
}

double pos_to_deg(const MotorConfig &cfg, double pos_rad)
{
  double pid_pos_deg = std::fmod(
    (cfg.zero_offset_rad + pos_rad * cfg.gear_ratio * cfg.direction) * 180.0 / kPi, 360.0);
  return pid_pos_deg < 0.0 ? pid_pos_deg + 360.0 : pid_pos_deg;
}

double deg_to_pos(const MotorConfig &cfg, double pid_pos_deg)
{
  // Pick the equivalent single-turn angle nearest the configured position range.
  // A reversed motor at +pi must not appear at -pi when the allowed range is [0, pi].
  const double center =
    cfg.mode == ControlMode::Position
      ? std::midpoint(cfg.min_pos_rad, cfg.max_pos_rad) * cfg.gear_ratio * cfg.direction
      : 0.0;
  return (center +
           std::remainder(pid_pos_deg * kPi / 180.0 - cfg.zero_offset_rad - center, 2.0 * kPi)) *
         cfg.direction / cfg.gear_ratio;
}

double apply_direction(const MotorConfig &cfg, double value)
{
  return value * cfg.direction;
}

// Configuration validation.
/// @brief 1 台の設定・時間・モード固有の指令範囲を検査する
void MotorController::validate_config(const MotorConfig &cfg)
{
  command_interface(cfg.mode);  // Reject invalid enum values as well.
  if (cfg.name.empty() || cfg.motor_id >= protocol::kReplyCanId) {
    throw std::invalid_argument("Motor name is required and CAN ID must be in [0, 253]");
  }
  const std::array numeric_settings = {cfg.pole_pairs, cfg.gear_ratio, cfg.direction,
    cfg.zero_offset_rad, cfg.max_vel_rad_s, cfg.max_current_a, cfg.max_duty, cfg.min_pos_rad,
    cfg.max_pos_rad, cfg.vel_slew_rad_s2, cfg.startup_current_a, cfg.rpm_threshold};
  for (double setting_value : numeric_settings) {
    if (!std::isfinite(setting_value)) {
      throw std::invalid_argument(cfg.name + ": non-finite setting");
    }
  }
  if (cfg.pole_pairs <= 0.0 || std::floor(cfg.pole_pairs) != cfg.pole_pairs ||
      cfg.gear_ratio <= 0.0 || std::abs(cfg.direction) != 1.0 || cfg.max_vel_rad_s <= 0.0 ||
      cfg.max_current_a <= 0.0 || cfg.vel_slew_rad_s2 <= 0.0 || cfg.startup_current_a < 0.0 ||
      cfg.rpm_threshold < 0.0 || cfg.feedback_timeout.count() <= 0 ||
      cfg.command_timeout.count() <= 0 || cfg.startup_timeout.count() <= 0) {
    throw std::invalid_argument(cfg.name + ": invalid motor limits or timing");
  }
  if (cfg.max_duty <= 0.0 || cfg.max_duty > 1.0) {
    throw std::invalid_argument(cfg.name + ": max_duty must be in (0, 1]");
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
// Shared feedback freshness and command limits.
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
    case ControlMode::DutyCycle:
      return std::abs(value) <= cfg.max_duty;
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
  // A known device fault remains unsafe until a healthy reply is received, even if stale.
  if (state.fault_code > 0 || !feedback_fresh(state.status1_at, now, timeout)) {
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

// One motor: native feedback, command state and mode-specific frame generation.
MotorController::MotorController(MotorConfig config) : config_(std::move(config))
{
  validate_config(config_);
}

bool MotorController::receive(const VescFeedback &feedback, TimePoint now)
{
  return std::visit(
    [&](const auto &value) {
      using Value = std::decay_t<decltype(value)>;
      if (value.motor_id != config_.motor_id) {
        return false;
      }
      auto &state = state_;
      if constexpr (std::is_same_v<Value, MotionFeedback>) {
        state.vel_rad_s = erpm_to_vel(config_, value.electrical_rpm);
        state.electrical_rpm = value.electrical_rpm;
        state.motor_current_a = value.motor_current_a;
        state.duty_cycle = value.duty_cycle;
        state.status1_at = now;
      } else if constexpr (std::is_same_v<Value, ChargeFeedback>) {
        state.charge_drawn_ah = value.charge_drawn_ah;
        state.charge_regen_ah = value.charge_regen_ah;
        state.status2_at = now;
      } else if constexpr (std::is_same_v<Value, EnergyFeedback>) {
        state.energy_drawn_wh = value.energy_drawn_wh;
        state.energy_regen_wh = value.energy_regen_wh;
        state.status3_at = now;
      } else if constexpr (std::is_same_v<Value, ControlFeedback>) {
        state.temperature_fet_c = value.temperature_fet_c;
        state.temperature_motor_c = value.temperature_motor_c;
        state.input_current_a = value.input_current_a;
        state.pid_pos_deg = value.motor_pid_position_deg;
        if (config_.mode == ControlMode::Position) {
          state.pos_rad = deg_to_pos(config_, value.motor_pid_position_deg);
        }
        state.status4_at = now;
      } else if constexpr (std::is_same_v<Value, PositionFeedback>) {
        state.tachometer_counts = value.tachometer_counts;
        if (config_.mode != ControlMode::Position) {
          state.pos_rad = tacho_to_pos(config_, value.tachometer_counts);
        }
        state.input_voltage_v = value.input_voltage_v;
        state.status5_at = now;
      } else if constexpr (std::is_same_v<Value, AnalogFeedback>) {
        state.adc_1_v = value.adc_1_v;
        state.adc_2_v = value.adc_2_v;
        state.adc_3_v = value.adc_3_v;
        state.ppm = value.ppm;
        state.status6_at = now;
      } else {
        return receive_fault(value, now);
      }
      return true;
    },
    feedback);
}

bool MotorController::receive_fault(const FaultCode &fault, TimePoint now)
{
  if (fault.motor_id != config_.motor_id) {
    return false;
  }
  state_.fault_code = fault.code;
  state_.fault_at = now;
  if (fault.code != 0) {
    cmd_ = Command{};
    fault_reason_ = "VESC fault";
  }
  return true;
}

void MotorController::reset()
{
  state_ = MotorState{};
  reset_command();
}

void MotorController::reset_command(TimePoint now)
{
  cmd_ = Command{};
  command_started_at_ = now;
  fault_reason_ = {};
}

bool MotorController::set_command(double value, TimePoint now)
{
  const auto &cfg = config_;
  if (!fault_reason_.empty()) {
    return false;
  }
  if (!command_is_valid(cfg, value)) {
    cmd_ = Command{};
    fault_reason_ = "invalid command";
    return false;
  }
  auto &cmd = cmd_;
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

std::string_view MotorController::command_error(TimePoint now) const
{
  if (!fault_reason_.empty()) {
    return fault_reason_;
  }
  if (!state_ready(config_, state_, now)) {
    return "feedback stale or position outside limits";
  }
  if (now - cmd_.received_at.value_or(command_started_at_) > config_.command_timeout) {
    return "command timeout";
  }
  return {};
}

can_msgs::msg::Frame MotorController::stop_command() const
{
  return protocol::comm_can_set_current(config_.motor_id, 0.0);
}

MotorController::CommandResult MotorController::command(TimePoint now)
{
  const auto error = command_error(now);
  if (!error.empty()) {
    fault_reason_ = error;
    cmd_ = Command{};
    return {stop_command(), error};
  }
  const auto &cfg = config_;
  auto &cmd = cmd_;
  const auto previous_ramp_at = cmd.ramp_at;
  cmd.ramp_at = now;
  if (!cmd.received_at || (cfg.mode != ControlMode::Position && cmd.target == 0.0)) {
    cmd.ramp_vel = 0.0;
    cmd.rpm_control_active = false;
    return {stop_command(), {}};
  }
  switch (cfg.mode) {
    case ControlMode::Current:
      return {protocol::comm_can_set_current(cfg.motor_id, apply_direction(cfg, cmd.target)), {}};
    case ControlMode::DutyCycle:
      return {protocol::comm_can_set_duty(cfg.motor_id, apply_direction(cfg, cmd.target)), {}};
    case ControlMode::Position:
      return {protocol::comm_can_set_pos(cfg.motor_id, pos_to_deg(cfg, cmd.target)), {}};
    case ControlMode::CurrentVelocity: {
      const auto &state = state_;
      const double rpm_per_vel = cfg.gear_ratio * 60.0 / (2.0 * kPi);
      const double measured_rpm = std::abs(state.vel_rad_s) * rpm_per_vel;
      const double target_rpm = std::abs(cmd.target) * rpm_per_vel;
      const bool reached_threshold = cmd.target * state.vel_rad_s > 0.0 &&
                                     measured_rpm >= std::min(target_rpm, cfg.rpm_threshold);
      if (!cmd.rpm_control_active && reached_threshold) {
        cmd.rpm_control_active = true;
        cmd.ramp_vel = std::clamp(state.vel_rad_s, -cfg.max_vel_rad_s, cfg.max_vel_rad_s);
      }
      if (!cmd.rpm_control_active) {
        if (now - cmd.startup_at > cfg.startup_timeout) {
          fault_reason_ = "startup timeout";
          cmd_ = Command{};
          return {stop_command(), fault_reason_};
        }
        return {protocol::comm_can_set_current(cfg.motor_id,
                  apply_direction(cfg, std::copysign(cfg.startup_current_a, cmd.target))),
          {}};
      }
      // Once startup finishes, use the same velocity ramp as direct velocity mode.
      [[fallthrough]];
    }
    case ControlMode::Velocity: {
      const double dt_s =
        std::max(0.0, std::chrono::duration<double>(now - previous_ramp_at).count());
      const double max_step = cfg.vel_slew_rad_s2 * dt_s;
      cmd.ramp_vel += std::clamp(cmd.target - cmd.ramp_vel, -max_step, max_step);
      const auto electrical_rpm =
        static_cast<std::int32_t>(std::llround(vel_to_erpm(cfg, cmd.ramp_vel)));
      return {protocol::comm_can_set_rpm(cfg.motor_id, electrical_rpm), {}};
    }
  }
  fault_reason_ = "unsupported control mode";
  cmd_ = Command{};
  return {stop_command(), fault_reason_};
}
}  // namespace vesc_can_driver
