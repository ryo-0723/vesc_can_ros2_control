#include "vesc_can_ros2_control/vesc_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vesc_can_ros2_control
{
namespace
{
using Parameters = std::unordered_map<std::string, std::string>;

std::string get_string_parameter(
  const Parameters &parameters, const std::string &name, const std::string &default_value)
{
  const auto parameter = parameters.find(name);
  return parameter == parameters.end() ? default_value : parameter->second;
}

double get_numeric_parameter(
  const Parameters &parameters, const std::string &name, double default_value)
{
  const auto parameter = parameters.find(name);
  if (parameter == parameters.end()) {
    return default_value;
  }
  const auto &parameter_text = parameter->second;
  std::size_t parsed_characters = 0;
  const double parameter_value = std::stod(parameter_text, &parsed_characters);
  if (parsed_characters != parameter_text.size() || !std::isfinite(parameter_value)) {
    throw std::invalid_argument(name + ": expected a finite number");
  }
  return parameter_value;
}

int get_integer_parameter(const Parameters &parameters, const std::string &name, int default_value)
{
  constexpr int kMaxIntegerParameter = 1'000'000;
  const double parameter_value = get_numeric_parameter(parameters, name, default_value);
  if (std::floor(parameter_value) != parameter_value || parameter_value < 0.0 ||
      parameter_value > kMaxIntegerParameter) {
    throw std::invalid_argument(name + ": expected a non-negative integer");
  }
  return static_cast<int>(parameter_value);
}
}  // namespace

VescSystem::StateField VescSystem::state_field_from_name(const std::string &interface_name)
{
  if (interface_name == hardware_interface::HW_IF_POSITION) {
    return StateField::Position;
  }
  if (interface_name == hardware_interface::HW_IF_VELOCITY) {
    return StateField::Velocity;
  }
  if (interface_name == "current") {
    return StateField::Current;
  }
  if (interface_name == "temperature_fet") {
    return StateField::TemperatureFet;
  }
  if (interface_name == "temperature_motor") {
    return StateField::TemperatureMotor;
  }
  if (interface_name == "input_voltage") {
    return StateField::InputVoltage;
  }
  if (interface_name == "duty_cycle") {
    return StateField::DutyCycle;
  }
  throw std::invalid_argument("Unsupported state interface: " + interface_name);
}

MotorConfig VescSystem::load_joint_config(const hardware_interface::ComponentInfo &joint)
{
  if (joint.command_interfaces.size() != 1) {
    throw std::invalid_argument(joint.name + ": exactly one command interface is required");
  }
  MotorConfig motor_config;
  motor_config.name = joint.name;
  const auto &command_interface = joint.command_interfaces.front().name;
  if (command_interface == hardware_interface::HW_IF_VELOCITY) {
    motor_config.mode = ControlMode::Velocity;
  } else if (command_interface == hardware_interface::HW_IF_POSITION) {
    motor_config.mode = ControlMode::Position;
  } else {
    throw std::invalid_argument(joint.name + ": unsupported command interface");
  }
  const auto &parameters = joint.parameters;
  if (parameters.count("controller_id") == 0 || parameters.count("pole_pairs") == 0) {
    throw std::invalid_argument(joint.name + ": controller_id and pole_pairs are required");
  }
  const int controller_id = get_integer_parameter(parameters, "controller_id", 0);
  if (controller_id > 254) {
    throw std::invalid_argument("Controller ID must be in [0, 254]");
  }
  motor_config.controller_id = static_cast<std::uint8_t>(controller_id);
  motor_config.pole_pairs = get_numeric_parameter(parameters, "pole_pairs", 7.0);
  motor_config.gear_ratio = get_numeric_parameter(parameters, "gear_ratio", 1.0);
  motor_config.direction = get_numeric_parameter(parameters, "direction", 1.0);
  motor_config.zero_offset_rad = get_numeric_parameter(parameters, "zero_offset_rad", 0.0);
  motor_config.max_velocity = get_numeric_parameter(parameters, "max_velocity", 20.0);
  motor_config.velocity_slew_rate = get_numeric_parameter(parameters, "velocity_slew_rate", 100.0);
  motor_config.min_position = get_numeric_parameter(parameters, "min_position", -kPi);
  motor_config.max_position = get_numeric_parameter(parameters, "max_position", kPi);
  motor_config.feedback_timeout =
    std::chrono::milliseconds(get_integer_parameter(parameters, "feedback_timeout_ms", 500));
  motor_config.command_timeout =
    std::chrono::milliseconds(get_integer_parameter(parameters, "command_timeout_ms", 500));

  bool has_matching_state_interface = false;
  for (const auto &state_interface : joint.state_interfaces) {
    const auto state_field = state_field_from_name(state_interface.name);
    has_matching_state_interface =
      has_matching_state_interface || state_interface.name == command_interface;
    if (motor_config.mode == ControlMode::Velocity && state_field == StateField::Position) {
      motor_config.require_tachometer = true;
    }
  }
  if (!has_matching_state_interface) {
    throw std::invalid_argument(joint.name + ": missing matching state interface");
  }
  return motor_config;
}

hardware_interface::CallbackReturn VescSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams &parameters)
{
  if (SystemInterface::on_init(parameters) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    const auto &hardware_info = parameters.hardware_info;
    const auto &hardware_parameters = hardware_info.hardware_parameters;
    transport_options_.tx_topic =
      get_string_parameter(hardware_parameters, "can_tx_topic", "/can/tx");
    transport_options_.rx_topic =
      get_string_parameter(hardware_parameters, "can_rx_topic", "/can/rx");
    transport_options_.period =
      std::chrono::milliseconds(get_integer_parameter(hardware_parameters, "send_period_ms", 10));
    activation_timeout_ = std::chrono::milliseconds(
      get_integer_parameter(hardware_parameters, "activation_timeout_ms", 2000));
    if (transport_options_.period.count() <= 0 || activation_timeout_.count() <= 0) {
      throw std::invalid_argument("Transport and activation periods must be positive");
    }
    motor_configs_.clear();
    motor_configs_.reserve(hardware_info.joints.size());
    for (const auto &joint : hardware_info.joints) {
      motor_configs_.push_back(load_joint_config(joint));
    }
    validate_configs(motor_configs_);
    motor_states_.resize(motor_configs_.size());
    motor_commands_.resize(motor_configs_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Invalid VESC hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

void VescSystem::cache_interface_handles()
{
  command_handles_.clear();
  state_handles_.clear();
  const auto &hardware_info = get_hardware_info();
  for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
    const auto &joint = hardware_info.joints[motor_index];
    command_handles_.push_back(
      get_command_interface_handle(joint.name + "/" + joint.command_interfaces.front().name));
    for (const auto &state_interface : joint.state_interfaces) {
      state_handles_.push_back({motor_index, state_field_from_name(state_interface.name),
        get_state_interface_handle(joint.name + "/" + state_interface.name)});
    }
  }
}

hardware_interface::CallbackReturn VescSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    std::fill(motor_states_.begin(), motor_states_.end(), MotorState{});
    cache_interface_handles();
    const auto hardware_node = get_node();
    auto node_options = rclcpp::NodeOptions().use_global_arguments(false);
    if (hardware_node) {
      node_options.context(hardware_node->get_node_base_interface()->get_context());
    }
    const auto node_namespace = hardware_node ? hardware_node->get_namespace() : "/";
    auto can_node = std::make_shared<rclcpp::Node>(
      get_hardware_info().name + "_can_io", node_namespace, node_options);
    can_transport_ = std::make_unique<CanTransport>(motor_configs_, transport_options_, can_node);
    can_transport_->start();
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "VESC configure failed: %s", error.what());
    release_can_transport();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn VescSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!can_transport_ || !can_transport_->wait_ready(activation_timeout_)) {
    RCLCPP_ERROR(get_logger(), "VESC activation requires fresh feedback from every motor");
    return hardware_interface::CallbackReturn::ERROR;
  }
  motor_states_ = can_transport_->snapshot();
  for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
    const auto &motor_config = motor_configs_[motor_index];
    const double initial_command = motor_config.mode == ControlMode::Position
                                     ? std::clamp(motor_states_[motor_index].position,
                                         motor_config.min_position, motor_config.max_position)
                                     : 0.0;
    set_command(command_handles_[motor_index], initial_command, true);
  }
  active_ = can_transport_->enable();
  return active_ ? hardware_interface::CallbackReturn::SUCCESS
                 : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn VescSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (can_transport_) {
    can_transport_->disable();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void VescSystem::release_can_transport()
{
  active_ = false;
  can_transport_.reset();
}

hardware_interface::CallbackReturn VescSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  release_can_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  release_can_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystem::on_error(const rclcpp_lifecycle::State &)
{
  release_can_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}

double VescSystem::read_state_value(const StateHandle &state_handle, TimePoint now) const
{
  const auto &motor_state = motor_states_[state_handle.motor_index];
  const auto &motor_config = motor_configs_[state_handle.motor_index];
  const auto timeout = motor_config.feedback_timeout;
  const bool status_1_fresh =
    feedback_is_fresh(motor_state.have_status1, motor_state.status1_time, now, timeout);
  const bool status_4_fresh =
    feedback_is_fresh(motor_state.have_status4, motor_state.status4_time, now, timeout);
  const bool status_5_fresh =
    feedback_is_fresh(motor_state.have_status5, motor_state.status5_time, now, timeout);
  switch (state_handle.field) {
    case StateField::Position: {
      const bool position_fresh =
        motor_config.mode == ControlMode::Position ? status_4_fresh : status_5_fresh;
      return position_fresh ? motor_state.position : kUnknownValue;
    }
    case StateField::Velocity:
      return status_1_fresh ? motor_state.velocity : kUnknownValue;
    case StateField::Current:
      return status_1_fresh ? motor_state.current_a : kUnknownValue;
    case StateField::TemperatureFet:
      return status_4_fresh ? motor_state.temperature_fet_c : kUnknownValue;
    case StateField::TemperatureMotor:
      return status_4_fresh ? motor_state.temperature_motor_c : kUnknownValue;
    case StateField::InputVoltage:
      return status_5_fresh ? motor_state.input_voltage_v : kUnknownValue;
    case StateField::DutyCycle:
      return status_1_fresh ? motor_state.duty_cycle : kUnknownValue;
  }
  return kUnknownValue;
}

hardware_interface::return_type VescSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!can_transport_) {
    return hardware_interface::return_type::ERROR;
  }
  // A busy mailbox retains the last snapshot; the same freshness checks still apply.
  can_transport_->try_snapshot(motor_states_);
  const auto current_time = SteadyClock::now();
  if (active_) {
    if (can_transport_->faulted()) {
      can_transport_->request_stop();
      return hardware_interface::return_type::ERROR;
    }
    for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
      if (!state_ready(motor_configs_[motor_index], motor_states_[motor_index], current_time)) {
        can_transport_->request_stop();
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  for (const auto &state_handle : state_handles_) {
    set_state(state_handle.handle, read_state_value(state_handle, current_time), false);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (!can_transport_ || can_transport_->faulted()) {
    return hardware_interface::return_type::ERROR;
  }
  for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
    if (!get_command(command_handles_[motor_index], motor_commands_[motor_index], false)) {
      return hardware_interface::return_type::OK;  // Retry next cycle; the previous batch expires.
    }
    if (!command_is_valid(motor_configs_[motor_index], motor_commands_[motor_index])) {
      can_transport_->request_stop();
      return hardware_interface::return_type::ERROR;
    }
  }
  can_transport_->try_set_commands(motor_commands_);
  return hardware_interface::return_type::OK;
}
}  // namespace vesc_can_ros2_control

PLUGINLIB_EXPORT_CLASS(vesc_can_ros2_control::VescSystem, hardware_interface::SystemInterface)
