#include "vesc_can_ros2_control/vesc_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include "pluginlib/class_list_macros.hpp"

namespace vesc_can_ros2_control
{
namespace
{
using Parameters = std::unordered_map<std::string, std::string>;
std::string value(const Parameters & params, const std::string & key, const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}
double number(const Parameters & params, const std::string & key, double fallback)
{
  const auto text = value(params, key, std::to_string(fallback));
  std::size_t parsed{};
  const double result = std::stod(text, &parsed);
  if (parsed != text.size() || !std::isfinite(result)) {
    throw std::invalid_argument(key + ": expected a finite number");
  }
  return result;
}
int integer(const Parameters & params, const std::string & key, int fallback)
{
  const double result = number(params, key, fallback);
  if (std::floor(result) != result || result < 0.0 || result > 1000000.0) {
    throw std::invalid_argument(key + ": expected a non-negative integer");
  }
  return static_cast<int>(result);
}
}  // namespace

hardware_interface::CallbackReturn VescSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    const auto & info = params.hardware_info;
    options_.tx_topic = value(info.hardware_parameters, "can_tx_topic", "/can/tx");
    options_.rx_topic = value(info.hardware_parameters, "can_rx_topic", "/can/rx");
    options_.period = std::chrono::milliseconds(integer(info.hardware_parameters, "send_period_ms", 10));
    activation_timeout_ = std::chrono::milliseconds(
      integer(info.hardware_parameters, "activation_timeout_ms", 2000));
    if (options_.period.count() <= 0 || activation_timeout_.count() <= 0) {
      throw std::invalid_argument("Transport and activation periods must be positive");
    }
    configs_.clear();
    for (const auto & joint : info.joints) {
      if (joint.command_interfaces.size() != 1) {
        throw std::invalid_argument(joint.name + ": exactly one command interface is required");
      }
      MotorConfig c;
      c.name = joint.name;
      c.logical_id = static_cast<uint16_t>(configs_.size());
      const auto command = joint.command_interfaces.front().name;
      if (command == "velocity") {c.mode = ControlMode::VELOCITY;}
      else if (command == "position") {c.mode = ControlMode::POSITION;}
      else {throw std::invalid_argument(joint.name + ": unsupported command interface");}
      if (joint.parameters.count("controller_id") == 0 || joint.parameters.count("pole_pairs") == 0) {
        throw std::invalid_argument(joint.name + ": controller_id and pole_pairs are required");
      }
      const int id = integer(joint.parameters, "controller_id", 0);
      if (id > 254) {throw std::invalid_argument("Controller ID must be in [0, 254]");}
      c.controller_id = static_cast<uint8_t>(id);
      c.pole_pairs = number(joint.parameters, "pole_pairs", 7.0);
      c.gear_ratio = number(joint.parameters, "gear_ratio", 1.0);
      c.direction = number(joint.parameters, "direction", 1.0);
      c.zero_offset_rad = number(joint.parameters, "zero_offset_rad", 0.0);
      c.max_velocity = number(joint.parameters, "max_velocity", 20.0);
      c.velocity_slew_rate = number(joint.parameters, "velocity_slew_rate", 100.0);
      c.min_position = number(joint.parameters, "min_position", -PI);
      c.max_position = number(joint.parameters, "max_position", PI);
      c.feedback_timeout = std::chrono::milliseconds(integer(joint.parameters, "feedback_timeout_ms", 500));
      c.command_timeout = std::chrono::milliseconds(integer(joint.parameters, "command_timeout_ms", 500));
      bool have_required_state = false;
      for (const auto & state : joint.state_interfaces) {
        have_required_state = have_required_state || state.name == command;
        c.require_tachometer = c.require_tachometer ||
          (c.mode == ControlMode::VELOCITY && state.name == "position");
        const std::vector<std::string> supported = {"position", "velocity", "current",
          "temperature_fet", "temperature_motor", "input_voltage", "duty_cycle"};
        if (std::find(supported.begin(), supported.end(), state.name) == supported.end()) {
          throw std::invalid_argument(joint.name + ": unsupported state interface " + state.name);
        }
      }
      if (!have_required_state) {throw std::invalid_argument(joint.name + ": missing matching state");}
      configs_.push_back(c);
    }
    validate_configs(configs_);
    states_.resize(configs_.size());
    commands_.resize(configs_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Invalid VESC hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn VescSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    command_handles_.clear();
    state_handles_.clear();
    std::fill(states_.begin(), states_.end(), MotorState{});
    const auto & info = get_hardware_info();
    for (std::size_t i = 0; i < configs_.size(); ++i) {
      const auto & joint = info.joints[i];
      command_handles_.push_back(get_command_interface_handle(
          joint.name + "/" + joint.command_interfaces.front().name));
      for (const auto & state : joint.state_interfaces) {
        StateField field = StateField::POSITION;
        if (state.name == "velocity") {field = StateField::VELOCITY;}
        else if (state.name == "current") {field = StateField::CURRENT;}
        else if (state.name == "temperature_fet") {field = StateField::TEMP_FET;}
        else if (state.name == "temperature_motor") {field = StateField::TEMP_MOTOR;}
        else if (state.name == "input_voltage") {field = StateField::VOLTAGE;}
        else if (state.name == "duty_cycle") {field = StateField::DUTY;}
        state_handles_.push_back({i, field, get_state_interface_handle(joint.name + "/" + state.name)});
      }
    }
    const auto parent = get_node();
    auto node_options = rclcpp::NodeOptions().use_global_arguments(false);
    if (parent) {node_options.context(parent->get_node_base_interface()->get_context());}
    io_node_ = std::make_shared<rclcpp::Node>(
      info.name + "_can_io", parent ? parent->get_namespace() : "/", node_options);
    transport_ = std::make_unique<CanTransport>(configs_, options_, io_node_);
    transport_->start();
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "VESC configure failed: %s", error.what());
    transport_.reset();
    io_node_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn VescSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!transport_ || !transport_->wait_ready(activation_timeout_)) {
    RCLCPP_ERROR(get_logger(), "VESC activation requires fresh feedback from every motor");
    return hardware_interface::CallbackReturn::ERROR;
  }
  states_ = transport_->snapshot();
  for (std::size_t i = 0; i < configs_.size(); ++i) {
    const double initial = configs_[i].mode == ControlMode::POSITION ?
      std::clamp(states_[i].position, configs_[i].min_position, configs_[i].max_position) : 0.0;
    set_command(command_handles_[i], initial, true);
  }
  active_ = transport_->enable();
  return active_ ? hardware_interface::CallbackReturn::SUCCESS : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn VescSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (transport_) {transport_->disable();}
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  active_ = false;
  transport_.reset();
  io_node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystem::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

hardware_interface::CallbackReturn VescSystem::on_error(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

hardware_interface::return_type VescSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!transport_) {return hardware_interface::return_type::ERROR;}
  transport_->try_snapshot(states_);
  const auto now = SteadyClock::now();
  if (active_) {
    for (std::size_t i = 0; i < configs_.size(); ++i) {
      if (!state_ready(configs_[i], states_[i], now) || transport_->faulted()) {
        transport_->request_stop();
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  for (const auto & item : state_handles_) {
    const auto & s = states_[item.motor];
    double value = NAN_VALUE;
    switch (item.field) {
      case StateField::POSITION: value = s.position; break;
      case StateField::VELOCITY: value = s.velocity; break;
      case StateField::CURRENT: value = s.current_a; break;
      case StateField::TEMP_FET: value = s.temperature_fet_c; break;
      case StateField::TEMP_MOTOR: value = s.temperature_motor_c; break;
      case StateField::VOLTAGE: value = s.input_voltage_v; break;
      case StateField::DUTY: value = s.duty_cycle; break;
    }
    const auto & c = configs_[item.motor];
    TimePoint stamp = s.status1_time;
    bool have = s.have_status1;
    if (item.field == StateField::TEMP_FET || item.field == StateField::TEMP_MOTOR ||
      (item.field == StateField::POSITION && c.mode == ControlMode::POSITION))
    {
      stamp = s.status4_time; have = s.have_status4;
    } else if (item.field == StateField::VOLTAGE || item.field == StateField::POSITION) {
      stamp = s.status5_time; have = s.have_status5;
    }
    if (!have || now < stamp || now - stamp > c.feedback_timeout) {value = NAN_VALUE;}
    set_state(item.handle, value, false);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {return hardware_interface::return_type::OK;}
  if (!transport_ || transport_->faulted()) {return hardware_interface::return_type::ERROR;}
  for (std::size_t i = 0; i < configs_.size(); ++i) {
    if (!get_command(command_handles_[i], commands_[i], false)) {
      return hardware_interface::return_type::OK;  // retry next cycle; old batch expires
    }
    const auto & c = configs_[i];
    if (!std::isfinite(commands_[i]) ||
      (c.mode == ControlMode::VELOCITY && std::abs(commands_[i]) > c.max_velocity) ||
      (c.mode == ControlMode::POSITION &&
      (commands_[i] < c.min_position || commands_[i] > c.max_position)))
    {
      transport_->request_stop();
      return hardware_interface::return_type::ERROR;
    }
  }
  transport_->try_set_commands(commands_);
  return hardware_interface::return_type::OK;
}
}  // namespace vesc_can_ros2_control

PLUGINLIB_EXPORT_CLASS(
  vesc_can_ros2_control::VescSystem, hardware_interface::SystemInterface)
