#include "vesc_can_driver/driver_core.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <utility>
namespace vesc_can_driver
{
void validate_configs(const std::vector<MotorConfig> &configs)
{
  if (configs.empty()) {
    throw std::invalid_argument("At least one motor is required");
  }
  std::set<std::string> names;
  std::set<std::uint8_t> logical_ids;
  std::set<std::uint8_t> ids;
  for (const auto &cfg : configs) {
    MotorController::validate_config(cfg);
    if (!names.insert(cfg.name).second || !logical_ids.insert(cfg.logical_id).second ||
        !ids.insert(cfg.motor_id).second) {
      throw std::invalid_argument("Motor names, logical IDs and CAN IDs must be unique");
    }
  }
}

// Motor group: routing, activation and stopping the registered motors together.
DriverCore::DriverCore(std::vector<MotorConfig> configs)
{
  validate_configs(configs);
  motors_.reserve(configs.size());
  for (auto &cfg : configs) {
    motor_idx_[cfg.motor_id] = motors_.size();
    motors_.emplace_back(std::move(cfg));
  }
}

bool DriverCore::receive(const VescFeedback &feedback, TimePoint now)
{
  if (const auto *fault = std::get_if<FaultCode>(&feedback)) {
    return receive_fault(*fault, now);
  }
  const auto motor_id = std::visit([](const auto &value) { return value.motor_id; }, feedback);
  const auto idx = motor_index(motor_id);
  return idx && motors_[*idx].receive(feedback, now);
}

bool DriverCore::receive_fault(const FaultCode &fault, TimePoint now)
{
  const auto idx = motor_index(fault.motor_id);
  if (!idx) {
    return false;
  }
  auto &motor = motors_[*idx];
  motor.receive_fault(fault, now);
  if (fault.code != 0 && !faulted_) {
    stop_on_error(motor.config().name + ": VESC fault_code=" + std::to_string(fault.code));
  }
  return true;
}

bool DriverCore::ready(TimePoint now) const
{
  return std::all_of(motors_.begin(), motors_.end(),
    [now](const auto &motor) { return state_ready(motor.config(), motor.state(), now); });
}

bool DriverCore::activate(TimePoint now)
{
  if (!ready(now)) {
    return false;
  }
  for (auto &motor : motors_) {
    motor.reset_command(now);
  }
  active_ = true;
  faulted_ = false;
  fault_reason_.clear();
  return true;
}

void DriverCore::deactivate()
{
  active_ = false;
  for (auto &motor : motors_) {
    motor.reset_command();
  }
}

bool DriverCore::reset(std::size_t motor_idx)
{
  if (motor_idx >= motors_.size()) {
    return false;
  }
  deactivate();
  motors_[motor_idx].reset();
  faulted_ = false;
  fault_reason_.clear();
  return true;
}

void DriverCore::stop_on_error(const std::string &reason)
{
  deactivate();
  faulted_ = true;
  fault_reason_ = reason;
}

bool DriverCore::set_command(std::size_t idx, double value, TimePoint now)
{
  if (idx >= motors_.size() || !active_) {
    return false;
  }
  if (!motors_[idx].set_command(value, now)) {
    stop_on_error(motors_[idx].config().name + ": invalid command");
    return false;
  }
  return true;
}

void DriverCore::stop_commands(std::vector<can_msgs::msg::Frame> &outputs) const
{
  outputs.clear();
  for (const auto &motor : motors_) {
    outputs.push_back(motor.stop_command());
  }
}

void DriverCore::commands(TimePoint now, std::vector<can_msgs::msg::Frame> &outputs)
{
  if (!active_) {
    stop_commands(outputs);
    return;
  }
  outputs.clear();
  for (auto &motor : motors_) {
    const auto result = motor.command(now);
    if (!result.error.empty()) {
      stop_on_error(motor.config().name + ": " + std::string(result.error));
      stop_commands(outputs);
      return;
    }
    outputs.push_back(result.command);
  }
}
}  // namespace vesc_can_driver
