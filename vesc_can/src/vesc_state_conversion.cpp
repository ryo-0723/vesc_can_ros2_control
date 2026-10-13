#include "vesc_can/vesc_state_conversion.hpp"

namespace vesc_can
{
vesc_msgs::msg::VescState make_vesc_state(
  const MotorConfig &motor_config, const MotorState &motor_state, TimePoint current_time)
{
  const auto timeout = motor_config.feedback_timeout;
  const bool status_1_fresh = feedback_is_fresh(
    motor_state.has_status_1, motor_state.status_1_received_at, current_time, timeout);
  const bool status_2_fresh = feedback_is_fresh(
    motor_state.has_status_2, motor_state.status_2_received_at, current_time, timeout);
  const bool status_3_fresh = feedback_is_fresh(
    motor_state.has_status_3, motor_state.status_3_received_at, current_time, timeout);
  const bool status_4_fresh = feedback_is_fresh(
    motor_state.has_status_4, motor_state.status_4_received_at, current_time, timeout);
  const bool status_5_fresh = feedback_is_fresh(
    motor_state.has_status_5, motor_state.status_5_received_at, current_time, timeout);

  vesc_msgs::msg::VescState message;
  message.controller_id = motor_config.controller_id;
  message.speed = status_1_fresh ? motor_state.electrical_rpm : kUnknownValue;
  message.current_motor = status_1_fresh ? motor_state.motor_current_a : kUnknownValue;
  message.duty_cycle = status_1_fresh ? motor_state.duty_cycle : kUnknownValue;
  message.charge_drawn = status_2_fresh ? motor_state.charge_drawn_ah : kUnknownValue;
  message.charge_regen = status_2_fresh ? motor_state.charge_regen_ah : kUnknownValue;
  message.energy_drawn = status_3_fresh ? motor_state.energy_drawn_wh : kUnknownValue;
  message.energy_regen = status_3_fresh ? motor_state.energy_regen_wh : kUnknownValue;
  message.temp_fet = status_4_fresh ? motor_state.temperature_fet_c : kUnknownValue;
  message.temp_motor = status_4_fresh ? motor_state.temperature_motor_c : kUnknownValue;
  message.current_input = status_4_fresh ? motor_state.input_current_a : kUnknownValue;
  message.pid_pos_now = status_4_fresh ? motor_state.motor_pid_position_deg : kUnknownValue;
  message.voltage_input = status_5_fresh ? motor_state.input_voltage_v : kUnknownValue;
  message.displacement = motor_state.tachometer_counts;

  message.avg_id = kUnknownValue;
  message.avg_iq = kUnknownValue;
  message.ntc_temp_mos1 = kUnknownValue;
  message.ntc_temp_mos2 = kUnknownValue;
  message.ntc_temp_mos3 = kUnknownValue;
  message.avg_vd = kUnknownValue;
  message.avg_vq = kUnknownValue;
  message.distance_traveled = kUnavailableVescInteger;
  message.fault_code = kUnavailableVescInteger;
  return message;
}
}  // namespace vesc_can
