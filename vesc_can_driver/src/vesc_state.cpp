#include "vesc_can_driver/vesc_state.hpp"

#include <utility>

namespace vesc_can_driver
{
namespace
{
/// @brief 確保済みの VescState を上書きし，名前の文字列領域を再利用する
void fill_state(const MotorConfig &cfg, const MotorState &state, TimePoint now,
  vesc_can_msgs::msg::VescState &msg)
{
  const auto timeout = cfg.feedback_timeout;
  const bool status_1_fresh = feedback_fresh(state.status1_at, now, timeout);
  const bool status_2_fresh = feedback_fresh(state.status2_at, now, timeout);
  const bool status_3_fresh = feedback_fresh(state.status3_at, now, timeout);
  const bool status_4_fresh = feedback_fresh(state.status4_at, now, timeout);
  const bool status_5_fresh = feedback_fresh(state.status5_at, now, timeout);

  msg.motor_name = cfg.name;
  msg.controller_id = cfg.controller_id;
  msg.erpm = status_1_fresh ? state.electrical_rpm : kUnknownValue;
  msg.motor_current_a = status_1_fresh ? state.motor_current_a : kUnknownValue;
  msg.duty_cycle = status_1_fresh ? state.duty_cycle : kUnknownValue;
  msg.charge_drawn_ah = status_2_fresh ? state.charge_drawn_ah : kUnknownValue;
  msg.charge_regen_ah = status_2_fresh ? state.charge_regen_ah : kUnknownValue;
  msg.energy_drawn_wh = status_3_fresh ? state.energy_drawn_wh : kUnknownValue;
  msg.energy_regen_wh = status_3_fresh ? state.energy_regen_wh : kUnknownValue;
  msg.temp_fet_c = status_4_fresh ? state.temperature_fet_c : kUnknownValue;
  msg.temp_motor_c = status_4_fresh ? state.temperature_motor_c : kUnknownValue;
  msg.input_current_a = status_4_fresh ? state.input_current_a : kUnknownValue;
  msg.pid_pos_deg = status_4_fresh ? state.pid_pos_deg : kUnknownValue;
  msg.input_voltage_v = status_5_fresh ? state.input_voltage_v : kUnknownValue;
  msg.tachometer_counts = state.tachometer_counts;
  msg.tachometer_fresh = status_5_fresh;
  msg.position_rad = status_5_fresh ? tacho_to_pos(cfg, state.tachometer_counts) : kUnknownValue;
  msg.fault_code =
    feedback_fresh(state.fault_at, now, timeout) ? state.fault_code : kUnknownFaultCode;
}
}  // namespace

vesc_can_msgs::msg::VescState make_vesc_state(
  const MotorConfig &cfg, const MotorState &state, TimePoint now)
{
  vesc_can_msgs::msg::VescState msg;
  fill_state(cfg, state, now, msg);
  return msg;
}

MotorStatePublisher::MotorStatePublisher(rclcpp::Node &node,
  const std::vector<MotorConfig> &configs, const std::string &error_topic,
  const std::string &state_topic)
  : configs_(configs), clock_(node.get_clock()), node_name_(node.get_fully_qualified_name())
{
  error_pub_ = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    error_topic, rclcpp::QoS(1).reliable().transient_local());
  state_msg_.states.resize(configs_.size());
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    state_msg_.states[idx].motor_name = configs_[idx].name;
  }
  if (!state_topic.empty()) {
    array_pub_ = node.create_publisher<vesc_can_msgs::msg::VescStateArray>(state_topic, 1);
  } else {
    pubs_.reserve(configs_.size());
    for (const auto &cfg : configs_) {
      pubs_.push_back(node.create_publisher<vesc_can_msgs::msg::VescState>(
        "~/motors/" + cfg.name + "/state", 10));
    }
  }
}

void MotorStatePublisher::publish_error(const DriverFault &fault)
{
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = clock_->now();
  for (const auto &cfg : configs_) {
    if (fault.device_fault && fault.device_fault->controller_id != cfg.controller_id) {
      continue;
    }
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.name = node_name_ + "/" + cfg.name;
    status.hardware_id = "vesc_can_id_" + std::to_string(cfg.controller_id);
    status.message = fault.reason;
    diagnostic_msgs::msg::KeyValue field;
    field.key = "source";
    field.value = fault.device_fault ? "vesc" : "driver";
    status.values.push_back(field);
    field.key = "controller_id";
    field.value = std::to_string(cfg.controller_id);
    status.values.push_back(field);
    if (fault.device_fault) {
      field.key = "fault_code";
      field.value = std::to_string(fault.device_fault->code);
      status.values.push_back(field);
    }
    msg.status.push_back(std::move(status));
  }
  error_pub_->publish(msg);
}

void MotorStatePublisher::publish(
  const std::vector<MotorState> &states, TimePoint now, const rclcpp::Time &stamp)
{
  state_msg_.stamp = stamp;
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    auto &msg = state_msg_.states[idx];
    fill_state(configs_[idx], states[idx], now, msg);
    msg.stamp = stamp;
    if (!array_pub_) {
      pubs_[idx]->publish(msg);
    }
  }
  if (array_pub_) {
    array_pub_->publish(state_msg_);
  }
}
}  // namespace vesc_can_driver
