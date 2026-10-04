#include "vesc_can_ros2_control/driver_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace vesc_can_ros2_control
{
double joint_velocity_to_erpm(const MotorConfig & c, double velocity)
{
  return velocity * 60.0 / (2.0 * PI) * c.gear_ratio * c.pole_pairs * c.direction;
}

double erpm_to_joint_velocity(const MotorConfig & c, double erpm)
{
  return erpm * (2.0 * PI) / (60.0 * c.gear_ratio * c.pole_pairs) * c.direction;
}

double joint_position_to_degrees(const MotorConfig & c, double position)
{
  double angle = std::fmod(
    (c.zero_offset_rad + position * c.gear_ratio * c.direction) * 180.0 / PI, 360.0);
  return angle < 0.0 ? angle + 360.0 : angle;
}

double degrees_to_joint_position(const MotorConfig & c, double degrees)
{
  return std::remainder(degrees * PI / 180.0 - c.zero_offset_rad, 2.0 * PI) *
         c.direction / c.gear_ratio;
}

void validate_configs(const std::vector<MotorConfig> & configs)
{
  if (configs.empty()) {throw std::invalid_argument("At least one motor is required");}
  std::set<std::string> names;
  std::set<uint16_t> logical_ids;
  std::set<uint8_t> controller_ids;
  for (const auto & c : configs) {
    const double numbers[] = {c.pole_pairs, c.gear_ratio, c.direction, c.zero_offset_rad,
      c.max_velocity, c.min_position, c.max_position, c.velocity_slew_rate,
      c.startup_current_a, c.rpm_control_threshold_rpm};
    for (double number : numbers) {
      if (!std::isfinite(number)) {throw std::invalid_argument(c.name + ": non-finite setting");}
    }
    if (c.name.empty() || !names.insert(c.name).second ||
      !logical_ids.insert(c.logical_id).second || !controller_ids.insert(c.controller_id).second)
    {
      throw std::invalid_argument("Motor names, logical IDs and controller IDs must be unique");
    }
    if (c.controller_id == 255 || c.pole_pairs <= 0.0 ||
      std::floor(c.pole_pairs) != c.pole_pairs || c.gear_ratio <= 0.0 ||
      std::abs(c.direction) != 1.0 || c.max_velocity <= 0.0 ||
      c.velocity_slew_rate <= 0.0 || c.startup_current_a < 0.0 ||
      c.rpm_control_threshold_rpm < 0.0 || c.feedback_timeout.count() <= 0 ||
      c.command_timeout.count() <= 0 || c.startup_timeout.count() <= 0)
    {
      throw std::invalid_argument(c.name + ": invalid motor limits or timing");
    }
    const double max_erpm = std::abs(joint_velocity_to_erpm(c, c.max_velocity));
    if (!std::isfinite(max_erpm) || max_erpm > std::numeric_limits<int32_t>::max()) {
      throw std::invalid_argument(c.name + ": speed limit exceeds VESC int32 ERPM range");
    }
    if (c.mode == ControlMode::POSITION &&
      (c.min_position >= c.max_position || c.min_position < -PI / c.gear_ratio ||
      c.max_position > PI / c.gear_ratio || c.startup_current_a != 0.0))
    {
      throw std::invalid_argument(c.name + ": PID position requires a single-turn range");
    }
    if (c.startup_current_a > std::numeric_limits<int32_t>::max() / 1000.0) {
      throw std::invalid_argument(c.name + ": current exceeds VESC int32 range");
    }
  }
}

bool state_ready(const MotorConfig & c, const MotorState & s, TimePoint now)
{
  const auto fresh = [&](bool have, TimePoint time) {
      return have && now >= time && now - time <= c.feedback_timeout;
    };
  const bool valid_position = c.mode != ControlMode::POSITION ||
    (std::isfinite(s.position) && s.position >= c.min_position - 1e-3 &&
    s.position <= c.max_position + 1e-3);
  return fresh(s.have_status1, s.status1_time) && valid_position &&
         (c.mode != ControlMode::POSITION || fresh(s.have_status4, s.status4_time)) &&
         (!c.require_tachometer || fresh(s.have_status5, s.status5_time));
}

DriverCore::DriverCore(std::vector<MotorConfig> configs)
: configs_(std::move(configs)), states_(configs_.size()), commands_(configs_.size())
{
  validate_configs(configs_);
}

bool DriverCore::receive(const can_msgs::msg::Frame & frame, TimePoint now)
{
  const auto id = static_cast<uint8_t>(frame.id & 0xFF);
  const auto it = std::find_if(configs_.begin(), configs_.end(), [id](const MotorConfig & c) {
      return c.controller_id == id;
    });
  if (it == configs_.end()) {return false;}
  const auto index = static_cast<std::size_t>(it - configs_.begin());
  auto & s = states_[index];
  protocol::Status1 one;
  protocol::Status4 four;
  protocol::Status5 five;
  if (protocol::decode_status_1(frame, one)) {
    s.velocity = erpm_to_joint_velocity(*it, one.erpm);
    s.current_a = one.current_a;
    s.duty_cycle = one.duty_cycle;
    s.status1_time = now;
    s.have_status1 = true;
  } else if (protocol::decode_status_4(frame, four)) {
    s.temperature_fet_c = four.temperature_fet_c;
    s.temperature_motor_c = four.temperature_motor_c;
    if (it->mode == ControlMode::POSITION) {
      s.position = degrees_to_joint_position(*it, four.position_deg);
    }
    s.status4_time = now;
    s.have_status4 = true;
  } else if (protocol::decode_status_5(frame, five)) {
    if (it->mode == ControlMode::VELOCITY) {
      s.position = five.tachometer * (2.0 * PI) /
        (6.0 * it->pole_pairs * it->gear_ratio) * it->direction;
    }
    s.input_voltage_v = five.input_voltage_v;
    s.status5_time = now;
    s.have_status5 = true;
  } else {return false;}
  return true;
}

bool DriverCore::ready(TimePoint now) const
{
  for (std::size_t i = 0; i < configs_.size(); ++i) {
    if (!state_ready(configs_[i], states_[i], now)) {return false;}
  }
  return true;
}

bool DriverCore::enable(TimePoint now)
{
  if (!ready(now)) {return false;}
  enabled_ = true;
  faulted_ = false;
  fault_reason_.clear();
  enabled_time_ = now;
  std::fill(commands_.begin(), commands_.end(), Command{});
  return true;
}

void DriverCore::disable()
{
  enabled_ = false;
  std::fill(commands_.begin(), commands_.end(), Command{});
}

void DriverCore::fault(const std::string & reason)
{
  faulted_ = true;
  enabled_ = false;
  fault_reason_ = reason;
}

bool DriverCore::set_command(std::size_t index, double value, TimePoint now)
{
  if (index >= configs_.size() || !enabled_ || faulted_) {return false;}
  const auto & c = configs_[index];
  if (!std::isfinite(value) ||
    (c.mode == ControlMode::VELOCITY && std::abs(value) > c.max_velocity) ||
    (c.mode == ControlMode::POSITION && (value < c.min_position || value > c.max_position)))
  {
    fault(c.name + ": invalid command");
    return false;
  }
  auto & cmd = commands_[index];
  if (!cmd.have_command || cmd.target * value <= 0.0) {
    cmd.rpm_active = false;
    cmd.startup_time = now;
  }
  cmd.target = value;
  cmd.received = now;
  if (!cmd.have_command) {cmd.ramp_time = now;}
  cmd.have_command = true;
  return true;
}

void DriverCore::stop_frames(std::vector<can_msgs::msg::Frame> & frames) const
{
  frames.clear();
  for (const auto & c : configs_) {
    frames.push_back(protocol::make_set_current_frame(c.controller_id, 0.0));
  }
}

void DriverCore::command_frames(TimePoint now, std::vector<can_msgs::msg::Frame> & frames)
{
  frames.clear();
  if (enabled_) {
    for (std::size_t i = 0; i < configs_.size(); ++i) {
      const auto & c = configs_[i];
      const auto & cmd = commands_[i];
      if (!state_ready(c, states_[i], now)) {
        fault(c.name + ": feedback stale or position outside limits");
        break;
      }
      if (now - (cmd.have_command ? cmd.received : enabled_time_) > c.command_timeout) {
        fault(c.name + ": command timeout");
        break;
      }
    }
  }
  if (!enabled_ || faulted_) {stop_frames(frames); return;}

  for (std::size_t i = 0; i < configs_.size(); ++i) {
    const auto & c = configs_[i];
    auto & cmd = commands_[i];
    if (!cmd.have_command) {
      frames.push_back(protocol::make_set_current_frame(c.controller_id, 0.0));
    } else if (c.mode == ControlMode::POSITION) {
      frames.push_back(protocol::make_set_position_frame(
          c.controller_id, joint_position_to_degrees(c, cmd.target)));
    } else if (cmd.target == 0.0) {
      cmd.ramped_velocity = 0.0;
      cmd.rpm_active = false;
      frames.push_back(protocol::make_set_current_frame(c.controller_id, 0.0));
    } else {
      const double measured_rpm = std::abs(states_[i].velocity) * c.gear_ratio * 60.0 / (2.0 * PI);
      const double target_rpm = std::abs(cmd.target) * c.gear_ratio * 60.0 / (2.0 * PI);
      const bool current_start = c.startup_current_a > 0.0 && c.rpm_control_threshold_rpm > 0.0;
      if (!cmd.rpm_active && (!current_start ||
        (cmd.target * states_[i].velocity > 0.0 && measured_rpm >=
        std::min(target_rpm, c.rpm_control_threshold_rpm))))
      {
        cmd.rpm_active = true;
        cmd.ramped_velocity = current_start ? states_[i].velocity : 0.0;
      }
      if (!cmd.rpm_active) {
        if (now - cmd.startup_time > c.startup_timeout) {
          fault(c.name + ": startup timeout");
          stop_frames(frames);
          return;
        }
        frames.push_back(protocol::make_set_current_frame(c.controller_id,
            std::copysign(c.startup_current_a, cmd.target * c.direction)));
      } else {
        const double dt = std::max(0.0, std::chrono::duration<double>(now - cmd.ramp_time).count());
        const double step = c.velocity_slew_rate * dt;
        cmd.ramped_velocity += std::clamp(cmd.target - cmd.ramped_velocity, -step, step);
        frames.push_back(protocol::make_set_rpm_frame(c.controller_id,
            static_cast<int32_t>(std::llround(joint_velocity_to_erpm(c, cmd.ramped_velocity)))));
      }
    }
    cmd.ramp_time = now;
  }
}
}  // namespace vesc_can_ros2_control
