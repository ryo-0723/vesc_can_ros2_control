#include "vesc_can_driver/vesc_state.hpp"

namespace vesc_can_driver
{
vesc_msgs::msg::VescState make_vesc_state(
  const MotorConfig &cfg, const MotorState &state, TimePoint now)
{
  const auto timeout = cfg.feedback_timeout;
  const bool status_1_fresh = feedback_fresh(state.status1_at, now, timeout);
  const bool status_2_fresh = feedback_fresh(state.status2_at, now, timeout);
  const bool status_3_fresh = feedback_fresh(state.status3_at, now, timeout);
  const bool status_4_fresh = feedback_fresh(state.status4_at, now, timeout);
  const bool status_5_fresh = feedback_fresh(state.status5_at, now, timeout);

  vesc_msgs::msg::VescState msg;
  msg.controller_id = cfg.controller_id;
  msg.speed = status_1_fresh ? state.electrical_rpm : kUnknownValue;
  msg.current_motor = status_1_fresh ? state.motor_current_a : kUnknownValue;
  msg.duty_cycle = status_1_fresh ? state.duty_cycle : kUnknownValue;
  msg.charge_drawn = status_2_fresh ? state.charge_drawn_ah : kUnknownValue;
  msg.charge_regen = status_2_fresh ? state.charge_regen_ah : kUnknownValue;
  msg.energy_drawn = status_3_fresh ? state.energy_drawn_wh : kUnknownValue;
  msg.energy_regen = status_3_fresh ? state.energy_regen_wh : kUnknownValue;
  msg.temp_fet = status_4_fresh ? state.temperature_fet_c : kUnknownValue;
  msg.temp_motor = status_4_fresh ? state.temperature_motor_c : kUnknownValue;
  msg.current_input = status_4_fresh ? state.input_current_a : kUnknownValue;
  msg.pid_pos_now = status_4_fresh ? state.pid_pos_deg : kUnknownValue;
  msg.voltage_input = status_5_fresh ? state.input_voltage_v : kUnknownValue;
  msg.displacement = state.tachometer_counts;

  msg.avg_id = kUnknownValue;
  msg.avg_iq = kUnknownValue;
  msg.ntc_temp_mos1 = kUnknownValue;
  msg.ntc_temp_mos2 = kUnknownValue;
  msg.ntc_temp_mos3 = kUnknownValue;
  msg.avg_vd = kUnknownValue;
  msg.avg_vq = kUnknownValue;
  msg.distance_traveled = kUnavailableVescInteger;
  msg.fault_code = kUnavailableVescInteger;
  return msg;
}
MotorStatePublisher::MotorStatePublisher(
  rclcpp::Node &node, const std::vector<MotorConfig> &configs)
  : configs_(configs)
{
  pubs_.reserve(configs_.size());
  for (const auto &cfg : configs_) {
    pubs_.push_back(node.create_publisher<vesc_msgs::msg::VescStateStamped>(
      "~/motors/" + cfg.name + "/state", 10));
  }
}

void MotorStatePublisher::publish(
  const std::vector<MotorState> &states, TimePoint now, const rclcpp::Time &stamp)
{
  for (std::size_t idx = 0; idx < configs_.size(); ++idx) {
    vesc_msgs::msg::VescStateStamped msg;
    msg.header.stamp = stamp;
    msg.state = make_vesc_state(configs_[idx], states[idx], now);
    pubs_[idx]->publish(msg);
  }
}
}  // namespace vesc_can_driver
