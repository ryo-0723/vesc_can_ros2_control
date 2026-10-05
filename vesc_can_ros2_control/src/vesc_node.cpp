#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "vesc_can_ros2_control/can_transport.hpp"
#include "vesc_can_ros2_control/vesc_telemetry.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

namespace vesc_can_ros2_control
{
using VescStateStamped = vesc_msgs::msg::VescStateStamped;
using Float64 = std_msgs::msg::Float64;
using JointState = sensor_msgs::msg::JointState;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using SetBool = std_srvs::srv::SetBool;

class VescNode : public rclcpp::Node
{
public:
  VescNode() : rclcpp::Node("vesc_can_ros2_control")
  {
    TransportOptions transport_options;
    transport_options.tx_topic =
      declare_parameter<std::string>("can_tx_topic", "/socketcan_bridge/tx");
    transport_options.rx_topic =
      declare_parameter<std::string>("can_rx_topic", "/socketcan_bridge/rx");
    transport_options.period =
      std::chrono::milliseconds(declare_parameter<std::int64_t>("update_period_ms", 20));
    const auto state_publish_period =
      std::chrono::milliseconds(declare_parameter<std::int64_t>("state_publish_period_ms", 100));
    if (state_publish_period.count() <= 0) {
      throw std::invalid_argument("State period must be positive");
    }
    auto_enable_once_ = declare_parameter<bool>("auto_enable_once", false);

    const auto motor_names =
      declare_parameter<std::vector<std::string>>("motors", std::vector<std::string>{});
    motor_configs_.reserve(motor_names.size());
    for (const auto &motor_name : motor_names) {
      motor_configs_.push_back(load_motor_config(motor_name));
    }
    validate_configs(motor_configs_);
    initialize_can_transport(transport_options);
    initialize_command_interfaces();
    initialize_state_publishers();
    state_publish_timer_ = create_wall_timer(state_publish_period, [this]() { publish_states(); });
  }

  void stop()
  {
    can_transport_->disable();
    can_transport_->shutdown();
  }

private:
  MotorConfig load_motor_config(const std::string &motor_name)
  {
    const auto parameter_prefix = motor_name + ".";
    MotorConfig motor_config;
    motor_config.name = motor_name;
    const auto controller_id =
      declare_parameter<std::int64_t>(parameter_prefix + "controller_id", -1);
    if (controller_id < 0 || controller_id > 254) {
      throw std::invalid_argument(motor_name + ": controller ID must be in [0, 254]");
    }
    motor_config.controller_id = static_cast<std::uint8_t>(controller_id);

    const auto command_interface =
      declare_parameter<std::string>(parameter_prefix + "command_interface", "velocity");
    if (command_interface == "position") {
      motor_config.mode = ControlMode::Position;
    } else if (command_interface != "velocity") {
      throw std::invalid_argument(motor_name + ": unsupported mode");
    }
    motor_config.pole_pairs = declare_parameter<double>(parameter_prefix + "pole_pairs", 7.0);
    motor_config.gear_ratio = declare_parameter<double>(parameter_prefix + "gear_ratio", 1.0);
    motor_config.direction = declare_parameter<double>(parameter_prefix + "direction", 1.0);
    motor_config.zero_offset_rad =
      declare_parameter<double>(parameter_prefix + "zero_offset_rad", 0.0);
    motor_config.min_position = declare_parameter<double>(parameter_prefix + "min_position", -kPi);
    motor_config.max_position = declare_parameter<double>(parameter_prefix + "max_position", kPi);

    const auto max_motor_rpm = declare_parameter<double>(parameter_prefix + "max_rpm", 5600.0);
    const auto motor_rpm_slew_rate =
      declare_parameter<double>(parameter_prefix + "rpm_slew_rate", 4000.0);
    const double motor_rpm_to_joint_rad_s = 2.0 * kPi / (60.0 * motor_config.gear_ratio);
    motor_config.max_velocity = max_motor_rpm * motor_rpm_to_joint_rad_s;
    motor_config.velocity_slew_rate = motor_rpm_slew_rate * motor_rpm_to_joint_rad_s;
    motor_config.startup_current_a =
      declare_parameter<double>(parameter_prefix + "startup_current_a", 0.0);
    motor_config.rpm_control_threshold_rpm =
      declare_parameter<double>(parameter_prefix + "rpm_control_threshold_rpm", 1000.0);
    motor_config.startup_timeout = std::chrono::milliseconds(
      declare_parameter<std::int64_t>(parameter_prefix + "startup_timeout_ms", 1000));
    motor_config.feedback_timeout = std::chrono::milliseconds(
      declare_parameter<std::int64_t>(parameter_prefix + "feedback_timeout_ms", 500));
    motor_config.command_timeout = std::chrono::milliseconds(
      declare_parameter<std::int64_t>(parameter_prefix + "command_timeout_ms", 500));
    motor_config.require_tachometer =
      declare_parameter<bool>(parameter_prefix + "require_tachometer", false);
    return motor_config;
  }

  void initialize_can_transport(const TransportOptions &transport_options)
  {
    auto node_options = rclcpp::NodeOptions().use_global_arguments(false);
    node_options.context(get_node_base_interface()->get_context());
    auto can_node = std::make_shared<rclcpp::Node>("vesc_can_io", get_namespace(), node_options);
    can_transport_ = std::make_unique<CanTransport>(motor_configs_, transport_options, can_node);
    can_transport_->start();
  }

  void initialize_command_interfaces()
  {
    motor_command_subscriptions_.reserve(motor_configs_.size());
    for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
      const auto command_topic = "~/motors/" + motor_configs_[motor_index].name + "/command";
      motor_command_subscriptions_.push_back(create_subscription<Float64>(
        command_topic, 1, [this, motor_index](Float64::ConstSharedPtr message) {
          can_transport_->set_command(motor_index, message->data);
        }));
    }
    joint_command_subscription_ = create_subscription<JointState>("~/joint_commands", 1,
      [this](JointState::ConstSharedPtr message) { handle_joint_commands(*message); });
    enable_service_ = create_service<SetBool>("~/enable",
      [this](SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
        handle_enable_request(*request, *response);
      });
  }

  void initialize_state_publishers()
  {
    motor_state_publishers_.reserve(motor_configs_.size());
    for (const auto &motor_config : motor_configs_) {
      motor_state_publishers_.push_back(
        create_publisher<VescStateStamped>("~/motors/" + motor_config.name + "/state", 10));
    }
    joint_state_publisher_ = create_publisher<JointState>("~/joint_states", 10);
    diagnostic_publisher_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
  }

  void handle_joint_commands(const JointState &message)
  {
    const auto joint_count = message.name.size();
    if ((!message.position.empty() && message.position.size() != joint_count) ||
        (!message.velocity.empty() && message.velocity.size() != joint_count)) {
      can_transport_->request_stop();
      RCLCPP_ERROR(get_logger(), "Joint command arrays must match name length");
      return;
    }
    for (std::size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      const auto motor_config = std::find_if(motor_configs_.begin(), motor_configs_.end(),
        [&](const MotorConfig &config) { return config.name == message.name[joint_index]; });
      if (motor_config == motor_configs_.end()) {
        continue;
      }
      const auto &command_values =
        motor_config->mode == ControlMode::Position ? message.position : message.velocity;
      if (!command_values.empty()) {
        const auto motor_index = static_cast<std::size_t>(motor_config - motor_configs_.begin());
        can_transport_->set_command(motor_index, command_values[joint_index]);
      }
    }
  }

  void handle_enable_request(const SetBool::Request &request, SetBool::Response &response)
  {
    auto_enable_once_ = false;
    if (!request.data) {
      can_transport_->disable();
      response.success = true;
      response.message = "Disabled";
      return;
    }
    response.success = can_transport_->enable();
    if (response.success) {
      response.message = "Enabled; send fresh commands for every motor";
    } else {
      response.message = "Fresh feedback from every motor is required";
    }
  }

  static void add_diagnostic_value(
    DiagnosticStatus &message, const std::string &name, const std::string &value)
  {
    diagnostic_msgs::msg::KeyValue field;
    field.key = name;
    field.value = value;
    message.values.push_back(field);
  }

  DiagnosticStatus make_diagnostic_status(const MotorConfig &motor_config,
    const MotorState &motor_state, TimePoint now, bool feedback_ready, bool enabled, bool faulted,
    const std::string &fault_reason) const
  {
    DiagnosticStatus message;
    message.name = std::string(get_namespace()) + "/vesc/" + motor_config.name;
    message.hardware_id = "vesc_can_id_" + std::to_string(motor_config.controller_id);
    if (!feedback_ready) {
      message.level = DiagnosticStatus::ERROR;
      message.message = "Required feedback missing/stale or position out of range";
    } else if (faulted) {
      message.level = DiagnosticStatus::ERROR;
      message.message = fault_reason;
    } else if (enabled) {
      message.level = DiagnosticStatus::OK;
      message.message = "Enabled";
    } else {
      message.level = DiagnosticStatus::WARN;
      message.message = "Disabled";
    }
    const auto timeout = motor_config.feedback_timeout;
    const auto add_freshness = [&](const std::string &name, bool received, TimePoint received_at) {
      const bool fresh = feedback_is_fresh(received, received_at, now, timeout);
      add_diagnostic_value(message, name, fresh ? "true" : "false");
    };
    add_freshness("status_1_fresh", motor_state.have_status1, motor_state.status1_time);
    add_freshness("status_2_fresh", motor_state.have_status2, motor_state.status2_time);
    add_freshness("status_3_fresh", motor_state.have_status3, motor_state.status3_time);
    add_freshness("status_4_fresh", motor_state.have_status4, motor_state.status4_time);
    add_freshness("status_5_fresh", motor_state.have_status5, motor_state.status5_time);
    add_freshness("status_6_fresh", motor_state.have_status6, motor_state.status6_time);
    add_diagnostic_value(message, "enabled", enabled ? "true" : "false");
    add_diagnostic_value(message, "driver_faulted", faulted ? "true" : "false");
    add_diagnostic_value(message, "unavailable_vesc_fields",
      "avg_id,avg_iq,ntc_temp_mos1,ntc_temp_mos2,ntc_temp_mos3,avg_vd,avg_vq,"
      "distance_traveled,fault_code");
    const bool status_6_fresh =
      feedback_is_fresh(motor_state.have_status6, motor_state.status6_time, now, timeout);
    add_diagnostic_value(
      message, "adc_1_v", std::to_string(status_6_fresh ? motor_state.adc_1_v : kUnknownValue));
    add_diagnostic_value(
      message, "adc_2_v", std::to_string(status_6_fresh ? motor_state.adc_2_v : kUnknownValue));
    add_diagnostic_value(
      message, "adc_3_v", std::to_string(status_6_fresh ? motor_state.adc_3_v : kUnknownValue));
    add_diagnostic_value(
      message, "ppm", std::to_string(status_6_fresh ? motor_state.ppm : kUnknownValue));
    return message;
  }

  void publish_states()
  {
    if (auto_enable_once_ && can_transport_->enable()) {
      auto_enable_once_ = false;
    }
    const auto motor_states = can_transport_->snapshot();
    const auto current_time = SteadyClock::now();
    const auto message_stamp = now();
    const bool enabled = can_transport_->enabled();
    const bool faulted = can_transport_->faulted();
    const auto fault_reason = faulted ? can_transport_->fault_reason() : std::string{};
    JointState joint_states;
    DiagnosticArray diagnostics;
    joint_states.header.stamp = message_stamp;
    diagnostics.header.stamp = message_stamp;
    for (std::size_t motor_index = 0; motor_index < motor_configs_.size(); ++motor_index) {
      const auto &motor_config = motor_configs_[motor_index];
      const auto &motor_state = motor_states[motor_index];
      const bool feedback_ready = state_ready(motor_config, motor_state, current_time);
      VescStateStamped vesc_state;
      vesc_state.header.stamp = message_stamp;
      vesc_state.state = make_vesc_state(motor_config, motor_state, current_time);
      motor_state_publishers_[motor_index]->publish(vesc_state);
      const bool position_fresh =
        motor_config.mode == ControlMode::Position
          ? feedback_ready
          : feedback_is_fresh(motor_state.have_status5, motor_state.status5_time, current_time,
              motor_config.feedback_timeout);
      joint_states.name.push_back(motor_config.name);
      joint_states.position.push_back(
        feedback_ready && position_fresh ? motor_state.position : kUnknownValue);
      joint_states.velocity.push_back(feedback_ready ? motor_state.velocity : kUnknownValue);
      diagnostics.status.push_back(make_diagnostic_status(
        motor_config, motor_state, current_time, feedback_ready, enabled, faulted, fault_reason));
    }
    joint_state_publisher_->publish(joint_states);
    diagnostic_publisher_->publish(diagnostics);
  }

  std::vector<MotorConfig> motor_configs_;
  bool auto_enable_once_{false};
  std::unique_ptr<CanTransport> can_transport_;
  std::vector<rclcpp::Subscription<Float64>::SharedPtr> motor_command_subscriptions_;
  rclcpp::Subscription<JointState>::SharedPtr joint_command_subscription_;
  rclcpp::Service<SetBool>::SharedPtr enable_service_;
  std::vector<rclcpp::Publisher<VescStateStamped>::SharedPtr> motor_state_publishers_;
  rclcpp::Publisher<JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr state_publish_timer_;
};
}  // namespace vesc_can_ros2_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<vesc_can_ros2_control::VescNode>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
