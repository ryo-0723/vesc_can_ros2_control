#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "vesc_can_interfaces/msg/actuator_state_array.hpp"
#include "vesc_can_interfaces/msg/actuator_target.hpp"
#include "vesc_can_interfaces/msg/actuator_target_array.hpp"
#include "vesc_can_ros2_control/can_transport.hpp"

namespace vesc_can_ros2_control
{
class VescNode : public rclcpp::Node
{
public:
  VescNode() : rclcpp::Node("vesc_can_ros2_control")
  {
    TransportOptions options;
    options.tx_topic = declare_parameter<std::string>("can_tx_topic", "/socketcan_bridge/tx");
    options.rx_topic = declare_parameter<std::string>("can_rx_topic", "/socketcan_bridge/rx");
    options.period = std::chrono::milliseconds(declare_parameter<int64_t>("update_period_ms", 20));
    const auto state_period = declare_parameter<int64_t>("state_array_publish_period_ms", 100);
    if (state_period <= 0) {throw std::invalid_argument("State period must be positive");}
    auto_enable_once_ = declare_parameter<bool>("auto_enable_once", false);
    const auto names = declare_parameter<std::vector<std::string>>("motors", std::vector<std::string>{});
    for (const auto & name : names) {
      const auto prefix = name + ".";
      MotorConfig c;
      c.name = name;
      const auto logical_id = declare_parameter<int64_t>(prefix + "logical_id", -1);
      const auto controller_id = declare_parameter<int64_t>(prefix + "controller_id", -1);
      if (logical_id < 0 || logical_id > 65535 || controller_id < 0 || controller_id > 254) {
        throw std::invalid_argument(name + ": invalid logical/controller ID");
      }
      c.logical_id = static_cast<uint16_t>(logical_id);
      c.controller_id = static_cast<uint8_t>(controller_id);
      const auto mode = declare_parameter<std::string>(prefix + "command_interface", "velocity");
      if (mode == "position") {c.mode = ControlMode::POSITION;}
      else if (mode != "velocity") {throw std::invalid_argument(name + ": unsupported mode");}
      c.pole_pairs = declare_parameter<double>(prefix + "pole_pairs", 7.0);
      c.gear_ratio = declare_parameter<double>(prefix + "gear_ratio", 1.0);
      c.direction = declare_parameter<double>(prefix + "direction", 1.0);
      c.zero_offset_rad = declare_parameter<double>(prefix + "zero_offset_rad", 0.0);
      c.min_position = declare_parameter<double>(prefix + "min_position", -PI);
      c.max_position = declare_parameter<double>(prefix + "max_position", PI);
      const auto max_rpm = declare_parameter<double>(prefix + "max_rpm", 5600.0);
      const auto rpm_slew = declare_parameter<double>(prefix + "rpm_slew_rate", 4000.0);
      c.max_velocity = max_rpm * 2.0 * PI / (60.0 * c.gear_ratio);
      c.velocity_slew_rate = rpm_slew * 2.0 * PI / (60.0 * c.gear_ratio);
      c.startup_current_a = declare_parameter<double>(prefix + "startup_current_a", 0.0);
      c.rpm_control_threshold_rpm = declare_parameter<double>(prefix + "rpm_control_threshold_rpm", 1000.0);
      c.startup_timeout = std::chrono::milliseconds(declare_parameter<int64_t>(prefix + "startup_timeout_ms", 1000));
      c.feedback_timeout = std::chrono::milliseconds(declare_parameter<int64_t>(prefix + "feedback_timeout_ms", 500));
      c.command_timeout = std::chrono::milliseconds(declare_parameter<int64_t>(prefix + "command_timeout_ms", 500));
      c.require_tachometer = declare_parameter<bool>(prefix + "require_tachometer", false);
      configs_.push_back(c);
    }
    validate_configs(configs_);
    auto io_options = rclcpp::NodeOptions().use_global_arguments(false).context(get_node_base_interface()->get_context());
    io_node_ = std::make_shared<rclcpp::Node>("vesc_can_io", get_namespace(), io_options);
    transport_ = std::make_unique<CanTransport>(configs_, options, io_node_);
    transport_->start();
    target_ = create_subscription<vesc_can_interfaces::msg::ActuatorTarget>(
      declare_parameter<std::string>("target_topic", "/vesc/target"), 1,
      [this](vesc_can_interfaces::msg::ActuatorTarget::ConstSharedPtr msg) {legacy_target(*msg);});
    targets_ = create_subscription<vesc_can_interfaces::msg::ActuatorTargetArray>(
      declare_parameter<std::string>("target_array_topic", "/vesc/target_array"), 1,
      [this](vesc_can_interfaces::msg::ActuatorTargetArray::ConstSharedPtr msg) {
        for (const auto & target : msg->actuators) {legacy_target(target);}
      });
    joint_commands_ = create_subscription<sensor_msgs::msg::JointState>(
      "~/joint_commands", 1, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        if ((!msg->position.empty() && msg->position.size() != msg->name.size()) ||
          (!msg->velocity.empty() && msg->velocity.size() != msg->name.size()))
        {
          transport_->request_stop();
          RCLCPP_ERROR(get_logger(), "Joint command arrays must match name length");
          return;
        }
        for (std::size_t i = 0; i < msg->name.size(); ++i) {
          const auto it = std::find_if(configs_.begin(), configs_.end(), [&](const MotorConfig & c) {return c.name == msg->name[i];});
          if (it == configs_.end()) {continue;}
          const auto & values = it->mode == ControlMode::POSITION ? msg->position : msg->velocity;
          if (!values.empty()) {transport_->set_command(it->logical_id, values[i]);}
        }
      });
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "~/enable", [this](std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        auto_enable_once_ = false;
        if (request->data) {
          response->success = transport_->enable();
          response->message = response->success ? "Enabled; send fresh commands for every motor" : "Fresh feedback from every motor is required";
        } else {
          transport_->disable();
          response->success = true;
          response->message = "Disabled";
        }
      });
    states_ = create_publisher<vesc_can_interfaces::msg::ActuatorStateArray>(declare_parameter<std::string>("state_array_topic", "/vesc/state_array"), 10);
    state_ = create_publisher<vesc_can_interfaces::msg::ActuatorState>(declare_parameter<std::string>("state_topic", "/vesc/state"), 10);
    joint_states_ = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(state_period), [this]() {publish_states();});
  }
  void stop() {transport_->disable(); transport_->shutdown();}
private:
  void legacy_target(const vesc_can_interfaces::msg::ActuatorTarget & msg)
  {
    const auto it = std::find_if(configs_.begin(), configs_.end(), [&](const MotorConfig & c) {return c.logical_id == msg.logical_id;});
    if (it == configs_.end()) {return;}
    // Legacy velocity is motor mechanical RPM; position is joint radians.
    double command = it->mode == ControlMode::VELOCITY ? msg.target * 2.0 * PI / (60.0 * it->gear_ratio) : msg.target;
    if (it->mode == ControlMode::VELOCITY && std::isfinite(command)) {command = std::clamp(command, -it->max_velocity, it->max_velocity);}
    transport_->set_command(msg.logical_id, command);
  }
  void publish_states()
  {
    if (auto_enable_once_ && transport_->enable()) {auto_enable_once_ = false;}
    const auto values = transport_->snapshot();
    const auto time = SteadyClock::now();
    const auto stamp = now();
    vesc_can_interfaces::msg::ActuatorStateArray array;
    sensor_msgs::msg::JointState joints;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    array.header.stamp = stamp;
    joints.header.stamp = stamp;
    diagnostics.header.stamp = stamp;
    for (std::size_t i = 0; i < configs_.size(); ++i) {
      const auto & c = configs_[i];
      const auto & s = values[i];
      const bool ready = state_ready(c, s, time);
      const bool faulted = transport_->faulted();
      const bool position_fresh = c.mode == ControlMode::POSITION ? ready :
        s.have_status5 && time - s.status5_time <= c.feedback_timeout;
      vesc_can_interfaces::msg::ActuatorState state;
      state.logical_id = c.logical_id;
      state.state = !ready ? state.STATE_OFFLINE : faulted ? state.STATE_ERROR : transport_->enabled() ? state.STATE_READY : state.STATE_INITIALIZING;
      state.position_reference_set = c.mode == ControlMode::POSITION && ready;
      state.position = ready && position_fresh ? s.position : NAN_VALUE;
      state.velocity = ready ? s.velocity * 60.0 * c.gear_ratio / (2.0 * PI) : NAN_VALUE;
      state.current_a = ready ? s.current_a : NAN_VALUE;
      state.temperature = s.have_status4 && time - s.status4_time <= c.feedback_timeout ? s.temperature_fet_c : NAN_VALUE;
      state.torque_nm = NAN_VALUE;
      // Driver fault only: native STATUS frames do not include VESC fault codes.
      state.fault_code = faulted ? 1 : 0;
      state_->publish(state);
      array.actuators.push_back(state);
      joints.name.push_back(c.name);
      joints.position.push_back(ready && position_fresh ? s.position : NAN_VALUE);
      joints.velocity.push_back(ready ? s.velocity : NAN_VALUE);
      diagnostic_msgs::msg::DiagnosticStatus diagnostic;
      diagnostic.name = std::string(get_namespace()) + "/vesc/" + c.name;
      diagnostic.hardware_id = "vesc_can_id_" + std::to_string(c.controller_id);
      diagnostic.level = !ready || faulted ? diagnostic.ERROR : transport_->enabled() ? diagnostic.OK : diagnostic.WARN;
      diagnostic.message = !ready ? "Required feedback missing/stale or position out of range" : faulted ? transport_->fault_reason() : transport_->enabled() ? "Enabled" : "Disabled";
      diagnostics.status.push_back(diagnostic);
    }
    states_->publish(array);
    joint_states_->publish(joints);
    diagnostics_->publish(diagnostics);
  }
  std::vector<MotorConfig> configs_;
  bool auto_enable_once_{false};
  rclcpp::Node::SharedPtr io_node_;
  std::unique_ptr<CanTransport> transport_;
  rclcpp::Subscription<vesc_can_interfaces::msg::ActuatorTarget>::SharedPtr target_;
  rclcpp::Subscription<vesc_can_interfaces::msg::ActuatorTargetArray>::SharedPtr targets_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_commands_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Publisher<vesc_can_interfaces::msg::ActuatorStateArray>::SharedPtr states_;
  rclcpp::Publisher<vesc_can_interfaces::msg::ActuatorState>::SharedPtr state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace vesc_can_ros2_control
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<vesc_can_ros2_control::VescNode>();
    rclcpp::spin(node);
    node->stop();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
