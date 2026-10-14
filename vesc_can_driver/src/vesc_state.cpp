#include "vesc_can_driver/vesc_state.hpp"

#include <algorithm>
#include <array>
#include <string_view>

namespace vesc_can_driver
{
namespace
{
/// @brief 事前確保した状態へ変換済みの速度・角度と監視値を上書きする
void fill_state(const MotorConfig &cfg, const MotorState &state, TimePoint now,
  vesc_can_msgs::msg::VescState &msg)
{
  const auto timeout = cfg.feedback_timeout;
  const bool status_1_fresh = feedback_fresh(state.status1_at, now, timeout);
  const bool status_2_fresh = feedback_fresh(state.status2_at, now, timeout);
  const bool status_3_fresh = feedback_fresh(state.status3_at, now, timeout);
  const bool status_4_fresh = feedback_fresh(state.status4_at, now, timeout);
  const bool status_5_fresh = feedback_fresh(state.status5_at, now, timeout);

  msg.motor_id = cfg.motor_id;
  msg.velocity_rad_s = status_1_fresh ? state.vel_rad_s : kUnknownValue;
  msg.motor_current_a = static_cast<float>(status_1_fresh ? state.motor_current_a : kUnknownValue);
  msg.duty_cycle = static_cast<float>(status_1_fresh ? state.duty_cycle : kUnknownValue);
  msg.charge_drawn_ah = static_cast<float>(status_2_fresh ? state.charge_drawn_ah : kUnknownValue);
  msg.charge_regen_ah = static_cast<float>(status_2_fresh ? state.charge_regen_ah : kUnknownValue);
  msg.energy_drawn_wh = static_cast<float>(status_3_fresh ? state.energy_drawn_wh : kUnknownValue);
  msg.energy_regen_wh = static_cast<float>(status_3_fresh ? state.energy_regen_wh : kUnknownValue);
  msg.temp_fet_c = static_cast<float>(status_4_fresh ? state.temperature_fet_c : kUnknownValue);
  msg.temp_motor_c = static_cast<float>(status_4_fresh ? state.temperature_motor_c : kUnknownValue);
  msg.input_current_a = static_cast<float>(status_4_fresh ? state.input_current_a : kUnknownValue);
  msg.pid_pos_rad = status_4_fresh ? deg_to_pos(cfg, state.pid_pos_deg) : kUnknownValue;
  msg.input_voltage_v = static_cast<float>(status_5_fresh ? state.input_voltage_v : kUnknownValue);
  msg.position_rad = status_5_fresh ? tacho_to_pos(cfg, state.tachometer_counts) : kUnknownValue;
  msg.fault_code =
    feedback_fresh(state.fault_at, now, timeout) ? state.fault_code : kUnknownFaultCode;
}
/// @brief VESC 6.06 の故障コードを名称付きのエラー文字列へ変換する
std::string event_message(const DriverFault &fault)
{
  if (!fault.device_fault) {
    return fault.reason;
  }
  // Official mc_fault_code order, release_6_06 (94b305ec), datatypes.h:132-163.
  constexpr std::array<std::string_view, 30> names{"NONE", "OVER_VOLTAGE", "UNDER_VOLTAGE", "DRV",
    "ABS_OVER_CURRENT", "OVER_TEMP_FET", "OVER_TEMP_MOTOR", "GATE_DRIVER_OVER_VOLTAGE",
    "GATE_DRIVER_UNDER_VOLTAGE", "MCU_UNDER_VOLTAGE", "BOOTING_FROM_WATCHDOG_RESET", "ENCODER_SPI",
    "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE", "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE", "FLASH_CORRUPTION",
    "HIGH_OFFSET_CURRENT_SENSOR_1", "HIGH_OFFSET_CURRENT_SENSOR_2", "HIGH_OFFSET_CURRENT_SENSOR_3",
    "UNBALANCED_CURRENTS", "BRK", "RESOLVER_LOT", "RESOLVER_DOS", "RESOLVER_LOS",
    "FLASH_CORRUPTION_APP_CFG", "FLASH_CORRUPTION_MC_CFG", "ENCODER_NO_MAGNET",
    "ENCODER_MAGNET_TOO_STRONG", "PHASE_FILTER", "ENCODER_FAULT", "LV_OUTPUT_FAULT"};
  const auto code = fault.device_fault->code;
  const auto name = code < names.size() ? names[code] : std::string_view("UNKNOWN");
  return "VESC fault_code=" + std::to_string(code) + " (" + std::string(name) + ")";
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
  : configs_(configs)
{
  error_pub_ = node.create_publisher<vesc_can_msgs::msg::VescEvent>(error_topic,
    rclcpp::QoS(std::max<std::size_t>(1, configs_.size())).reliable().transient_local());
  state_msg_.states.resize(configs_.size());
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    state_msg_.states[idx].motor_id = configs_[idx].motor_id;
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
  if (fault.device_fault && fault.device_fault->code == 0) {
    return;
  }
  vesc_can_msgs::msg::VescEvent msg;
  msg.message = event_message(fault);
  for (const auto &cfg : configs_) {
    if (fault.device_fault && fault.device_fault->controller_id != cfg.controller_id) {
      continue;
    }
    msg.motor_id = cfg.motor_id;
    error_pub_->publish(msg);
  }
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
