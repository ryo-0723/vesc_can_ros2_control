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

/// @brief 文字列パラメータを取得し、未指定なら既定値を返す。
std::string get_string_parameter(
  const Parameters &params, const std::string &name, const std::string &default_value)
{
  const auto param_it = params.find(name);
  return param_it == params.end() ? default_value : param_it->second;
}

/// @brief 数値パラメータを取得し、有限性と文字列全体の変換を検査する。
double get_numeric_parameter(
  const Parameters &params, const std::string &name, double default_value)
{
  const auto param_it = params.find(name);
  if (param_it == params.end()) {
    return default_value;
  }
  const auto &text = param_it->second;
  std::size_t parsed = 0;
  const double value = std::stod(text, &parsed);
  if (parsed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(name + ": expected a finite number");
  }
  return value;
}

/// @brief 非負整数パラメータを取得し、整数性と許容範囲を検査する。
int get_integer_parameter(const Parameters &params, const std::string &name, int default_value)
{
  constexpr int kMaxIntegerParameter = 1'000'000;
  const double value = get_numeric_parameter(params, name, default_value);
  if (std::floor(value) != value || value < 0.0 || value > kMaxIntegerParameter) {
    throw std::invalid_argument(name + ": expected a non-negative integer");
  }
  return static_cast<int>(value);
}
}  // namespace

VescSystem::~VescSystem()
{
  release_can_transport();
}

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

vesc_can_driver::MotorConfig VescSystem::load_joint_config(
  const hardware_interface::ComponentInfo &joint)
{
  if (joint.command_interfaces.size() != 1) {
    throw std::invalid_argument(joint.name + ": exactly one command interface is required");
  }
  vesc_can_driver::MotorConfig cfg;
  cfg.name = joint.name;
  const auto &command_interface = joint.command_interfaces.front().name;
  const auto &params = joint.parameters;
  cfg.mode = vesc_can_driver::resolve_control_mode(
    get_string_parameter(params, "control_mode", ""), command_interface);
  if (params.count("controller_id") == 0 || params.count("pole_pairs") == 0) {
    throw std::invalid_argument(joint.name + ": controller_id and pole_pairs are required");
  }
  const int controller_id = get_integer_parameter(params, "controller_id", 0);
  if (controller_id > 254) {
    throw std::invalid_argument("Controller ID must be in [0, 254]");
  }
  cfg.controller_id = static_cast<std::uint8_t>(controller_id);
  cfg.pole_pairs = get_numeric_parameter(params, "pole_pairs", 7.0);
  cfg.gear_ratio = get_numeric_parameter(params, "gear_ratio", 1.0);
  cfg.direction = get_numeric_parameter(params, "direction", 1.0);
  cfg.zero_offset_rad = get_numeric_parameter(params, "zero_offset_rad", 0.0);
  cfg.max_vel_rad_s = get_numeric_parameter(params, "max_velocity", 20.0);
  cfg.max_current_a = get_numeric_parameter(params, "max_current_a", 10.0);
  cfg.vel_slew_rad_s2 = get_numeric_parameter(params, "velocity_slew_rate", 100.0);
  cfg.startup_current_a = get_numeric_parameter(params, "startup_current_a", 0.0);
  cfg.rpm_threshold = get_numeric_parameter(params, "rpm_control_threshold_rpm", 1000.0);
  cfg.startup_timeout =
    std::chrono::milliseconds(get_integer_parameter(params, "startup_timeout_ms", 1000));
  cfg.min_pos_rad = get_numeric_parameter(params, "min_position", -vesc_can_driver::kPi);
  cfg.max_pos_rad = get_numeric_parameter(params, "max_position", vesc_can_driver::kPi);
  cfg.feedback_timeout =
    std::chrono::milliseconds(get_integer_parameter(params, "feedback_timeout_ms", 500));
  cfg.command_timeout =
    std::chrono::milliseconds(get_integer_parameter(params, "command_timeout_ms", 500));

  bool has_cmd_state = false;
  for (const auto &state_if : joint.state_interfaces) {
    const auto state_field = state_field_from_name(state_if.name);
    has_cmd_state = has_cmd_state || state_if.name == command_interface;
    if (cfg.mode != vesc_can_driver::ControlMode::Position && state_field == StateField::Position) {
      cfg.require_tachometer = true;
    }
  }
  if (!has_cmd_state) {
    throw std::invalid_argument(joint.name + ": missing matching state interface");
  }
  return cfg;
}

hardware_interface::CallbackReturn VescSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams &params)
{
  if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    const auto &hw_info = params.hardware_info;
    const auto &hw_params = hw_info.hardware_parameters;
    opts_.tx_topic = get_string_parameter(hw_params, "can_tx_topic", "/can/tx");
    opts_.rx_topic = get_string_parameter(hw_params, "can_rx_topic", "/can/rx");
    opts_.send_period =
      std::chrono::milliseconds(get_integer_parameter(hw_params, "send_period_ms", 10));
    activation_timeout_ =
      std::chrono::milliseconds(get_integer_parameter(hw_params, "activation_timeout_ms", 2000));
    state_period_ =
      std::chrono::milliseconds(get_integer_parameter(hw_params, "state_publish_period_ms", 20));
    if (opts_.send_period.count() <= 0 || activation_timeout_.count() <= 0 ||
        state_period_.count() <= 0) {
      throw std::invalid_argument("Transport, activation and state periods must be positive");
    }
    configs_.clear();
    configs_.reserve(hw_info.joints.size());
    for (const auto &joint : hw_info.joints) {
      configs_.push_back(load_joint_config(joint));
    }
    vesc_can_driver::validate_configs(configs_);
    states_.resize(configs_.size());
    cmds_.resize(configs_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Invalid VESC hardware configuration: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

void VescSystem::cache_interface_handles()
{
  cmd_handles_.clear();
  state_handles_.clear();
  const auto &hw_info = get_hardware_info();
  for (std::size_t motor_idx = 0; motor_idx < configs_.size(); ++motor_idx) {
    const auto &joint = hw_info.joints[motor_idx];
    cmd_handles_.push_back(
      get_command_interface_handle(joint.name + "/" + joint.command_interfaces.front().name));
    for (const auto &state_if : joint.state_interfaces) {
      state_handles_.push_back({motor_idx, state_field_from_name(state_if.name),
        get_state_interface_handle(joint.name + "/" + state_if.name)});
    }
  }
}

hardware_interface::CallbackReturn VescSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    std::fill(states_.begin(), states_.end(), vesc_can_driver::MotorState{});
    cache_interface_handles();
    const auto hw_node = get_node();
    auto node_opts = rclcpp::NodeOptions().use_global_arguments(false);
    if (hw_node) {
      node_opts.context(hw_node->get_node_base_interface()->get_context());
    }
    const auto node_ns = hw_node ? hw_node->get_namespace() : "/";
    auto can_node =
      std::make_shared<rclcpp::Node>(get_hardware_info().name + "_can_io", node_ns, node_opts);
    transport_ = std::make_unique<vesc_can_driver::CanTransport>(configs_, opts_, can_node);
    state_pub_ = std::make_unique<vesc_can_driver::MotorStatePublisher>(*can_node, configs_);
    state_timer_ =
      can_node->create_wall_timer(state_period_, [this, clock = can_node->get_clock()]() {
        const auto states = transport_->snapshot();
        state_pub_->publish(states, vesc_can_driver::SteadyClock::now(), clock->now());
      });
    transport_->start();
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "VESC configure failed: %s", error.what());
    release_can_transport();
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
  for (std::size_t motor_idx = 0; motor_idx < configs_.size(); ++motor_idx) {
    const auto &cfg = configs_[motor_idx];
    const double initial_cmd =
      cfg.mode == vesc_can_driver::ControlMode::Position
        ? std::clamp(states_[motor_idx].pos_rad, cfg.min_pos_rad, cfg.max_pos_rad)
        : 0.0;
    set_command(cmd_handles_[motor_idx], initial_cmd, true);
  }
  active_ = transport_->enable();
  return active_ ? hardware_interface::CallbackReturn::SUCCESS
                 : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn VescSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (transport_) {
    transport_->disable();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void VescSystem::release_can_transport()
{
  active_ = false;
  if (transport_) {
    transport_->shutdown();
  }
  state_timer_.reset();
  state_pub_.reset();
  transport_.reset();
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

double VescSystem::read_state_value(
  const StateHandle &binding, vesc_can_driver::TimePoint steady_now) const
{
  const auto &state = states_[binding.motor_idx];
  const auto &cfg = configs_[binding.motor_idx];
  const auto timeout = cfg.feedback_timeout;
  const bool status_1_fresh =
    vesc_can_driver::feedback_fresh(state.status1_at, steady_now, timeout);
  const bool status_4_fresh =
    vesc_can_driver::feedback_fresh(state.status4_at, steady_now, timeout);
  const bool status_5_fresh =
    vesc_can_driver::feedback_fresh(state.status5_at, steady_now, timeout);
  switch (binding.field) {
    case StateField::Position: {
      const bool pos_fresh =
        cfg.mode == vesc_can_driver::ControlMode::Position ? status_4_fresh : status_5_fresh;
      return pos_fresh ? state.pos_rad : vesc_can_driver::kUnknownValue;
    }
    case StateField::Velocity:
      return status_1_fresh ? state.vel_rad_s : vesc_can_driver::kUnknownValue;
    case StateField::Current:
      return status_1_fresh ? vesc_can_driver::apply_direction(cfg, state.motor_current_a)
                            : vesc_can_driver::kUnknownValue;
    case StateField::TemperatureFet:
      return status_4_fresh ? state.temperature_fet_c : vesc_can_driver::kUnknownValue;
    case StateField::TemperatureMotor:
      return status_4_fresh ? state.temperature_motor_c : vesc_can_driver::kUnknownValue;
    case StateField::InputVoltage:
      return status_5_fresh ? state.input_voltage_v : vesc_can_driver::kUnknownValue;
    case StateField::DutyCycle:
      return status_1_fresh ? state.duty_cycle : vesc_can_driver::kUnknownValue;
  }
  return vesc_can_driver::kUnknownValue;
}

hardware_interface::return_type VescSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!transport_) {
    return hardware_interface::return_type::ERROR;
  }
  // A busy mailbox retains the last snapshot; the same freshness checks still apply.
  transport_->try_snapshot(states_);
  const auto steady_now = vesc_can_driver::SteadyClock::now();
  if (active_) {
    if (transport_->faulted()) {
      transport_->request_stop();
      return hardware_interface::return_type::ERROR;
    }
    for (std::size_t motor_idx = 0; motor_idx < configs_.size(); ++motor_idx) {
      if (!vesc_can_driver::state_ready(configs_[motor_idx], states_[motor_idx], steady_now)) {
        transport_->request_stop();
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  for (const auto &binding : state_handles_) {
    set_state(binding.handle, read_state_value(binding, steady_now), false);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (!transport_ || transport_->faulted()) {
    return hardware_interface::return_type::ERROR;
  }
  for (std::size_t motor_idx = 0; motor_idx < configs_.size(); ++motor_idx) {
    if (!get_command(cmd_handles_[motor_idx], cmds_[motor_idx], false)) {
      return hardware_interface::return_type::OK;  // Retry next cycle; the previous batch expires.
    }
    if (!vesc_can_driver::command_is_valid(configs_[motor_idx], cmds_[motor_idx])) {
      transport_->request_stop();
      return hardware_interface::return_type::ERROR;
    }
  }
  transport_->try_set_commands(cmds_);
  return hardware_interface::return_type::OK;
}
}  // namespace vesc_can_ros2_control

PLUGINLIB_EXPORT_CLASS(vesc_can_ros2_control::VescSystem, hardware_interface::SystemInterface)
