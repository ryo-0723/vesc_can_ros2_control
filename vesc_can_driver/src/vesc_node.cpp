#include "vesc_can_driver/vesc_node.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
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

  msg.logical_id = cfg.logical_id;
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
    if (fault.device_fault && fault.device_fault->motor_id != cfg.motor_id) {
      continue;
    }
    msg.logical_id = cfg.logical_id;
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
VescNode::VescNode(const rclcpp::NodeOptions &options) : rclcpp::Node("vesc_can_driver", options)
{
  motor_idx_.fill(-1);
  TransportOptions opts;
  opts.tx_topic = declare_parameter<std::string>("can_tx_topic", "can/tx");
  opts.rx_topic = declare_parameter<std::string>("can_rx_topic", "can/rx");
  opts.send_period =
    std::chrono::milliseconds(declare_parameter<std::int64_t>("update_period_ms", 10));
  opts.fault_period =
    std::chrono::milliseconds(declare_parameter<std::int64_t>("fault_poll_period_ms", 100));
  const auto state_period =
    std::chrono::milliseconds(declare_parameter<std::int64_t>("state_publish_period_ms", 20));

  if (state_period.count() <= 0) {
    throw std::invalid_argument("State period must be positive");
  }
  opts.auto_activate = true;

  const auto names =
    declare_parameter<std::vector<std::string>>("motors", std::vector<std::string>{});
  configs_.reserve(names.size());

  for (const auto &motor_name : names) {
    configs_.push_back(load_motor_config(motor_name));
  }
  cmds_.reserve(configs_.size());
  states_.resize(configs_.size());
  initialize_command_interfaces();
  initialize_state_publishers(state_period);
  initialize_can_transport(opts);
}

/// @brief モータ名を接頭辞に持つ ROS パラメータから設定を読み込む
MotorConfig VescNode::load_motor_config(const std::string &motor_name)
{
  const auto prefix = motor_name + ".";
  MotorConfig cfg;
  cfg.name = motor_name;
  const auto logical_id = declare_parameter<std::int64_t>(prefix + "logical_id", -1);
  if (logical_id < 0 || logical_id > 255 || motor_idx_[logical_id] >= 0) {
    throw std::invalid_argument(motor_name + ": logical_id must be unique and in [0, 255]");
  }
  cfg.logical_id = static_cast<std::uint8_t>(logical_id);
  const auto motor_id = declare_parameter<std::int64_t>(prefix + "motor_id", -1);
  if (motor_id < 0 || motor_id > 253) {
    throw std::invalid_argument(motor_name + ": motor_id must be in [0, 253]");
  }
  cfg.motor_id = static_cast<std::uint8_t>(motor_id);

  rcl_interfaces::msg::ParameterDescriptor fixed_mode;
  fixed_mode.read_only = true;
  fixed_mode.description = "Control mode is fixed at node startup";
  const auto mode_name = declare_parameter<std::string>(prefix + "control_mode", "", fixed_mode);
  const auto default_interface = mode_name.empty()
                                   ? std::string("velocity")
                                   : std::string(command_interface(parse_control_mode(mode_name)));

  const auto cmd_interface =
    declare_parameter<std::string>(prefix + "command_interface", default_interface, fixed_mode);
  cfg.mode = resolve_control_mode(mode_name, cmd_interface);
  cfg.pole_pairs = declare_parameter<double>(prefix + "pole_pairs", 7.0);
  cfg.gear_ratio = declare_parameter<double>(prefix + "gear_ratio", 1.0);
  cfg.direction = declare_parameter<double>(prefix + "direction", 1.0);
  cfg.zero_offset_rad = declare_parameter<double>(prefix + "zero_offset_rad", 0.0);
  cfg.min_pos_rad = declare_parameter<double>(prefix + "min_position", -kPi);
  cfg.max_pos_rad = declare_parameter<double>(prefix + "max_position", kPi);

  const auto max_rpm = declare_parameter<double>(prefix + "max_rpm", 5600.0);
  const auto rpm_slew_rate = declare_parameter<double>(prefix + "rpm_slew_rate", 4000.0);
  const double rpm_to_rad_s = 2.0 * kPi / (60.0 * cfg.gear_ratio);
  cfg.max_vel_rad_s = max_rpm * rpm_to_rad_s;
  cfg.vel_slew_rad_s2 = rpm_slew_rate * rpm_to_rad_s;
  cfg.max_current_a = declare_parameter<double>(prefix + "max_current_a", 10.0);
  cfg.max_duty = declare_parameter<double>(prefix + "max_duty", 1.0);
  cfg.startup_current_a = declare_parameter<double>(prefix + "startup_current_a", 0.0);
  cfg.rpm_threshold = declare_parameter<double>(prefix + "rpm_control_threshold_rpm", 1000.0);
  cfg.startup_timeout =
    std::chrono::milliseconds(declare_parameter<std::int64_t>(prefix + "startup_timeout_ms", 1000));
  cfg.feedback_timeout =
    std::chrono::milliseconds(declare_parameter<std::int64_t>(prefix + "feedback_timeout_ms", 500));
  cfg.command_timeout =
    std::chrono::milliseconds(declare_parameter<std::int64_t>(prefix + "command_timeout_ms", 500));
  cfg.require_tachometer = declare_parameter<bool>(prefix + "require_tachometer", false);
  motor_idx_[logical_id] = static_cast<int>(configs_.size());
  return cfg;
}

/// @brief 同じ namespace / context で通信ノードと専用 executor を起動する
void VescNode::initialize_can_transport(const TransportOptions &opts)
{
  auto node_opts = rclcpp::NodeOptions().use_global_arguments(false);
  node_opts.context(get_node_base_interface()->get_context());
  auto can_node =
    std::make_shared<rclcpp::Node>(std::string(get_name()) + "_can_io", get_namespace(), node_opts);
  transport_ = std::make_unique<VescTransport>(configs_, opts, can_node,
    [pub = state_pub_.get()](const DriverFault &fault) { pub->publish_error(fault); });
  transport_->start();
}

/// @brief 全モータ共通の指令トピックと reboot サービスを登録する
void VescNode::initialize_command_interfaces()
{
  const auto cmd_topic = declare_parameter<std::string>("command_topic", "vesc/commands");
  cmd_sub_ = create_subscription<VescCommandArray>(
    cmd_topic, 10, [this](VescCommandArray::ConstSharedPtr msg) { command_callback(*msg); });

  reboot_srv_ = create_service<Reboot>(
    "~/reboot", [this](Reboot::Request::SharedPtr request, Reboot::Response::SharedPtr response) {
      const auto motor_idx = motor_idx_[request->logical_id];
      if (motor_idx < 0) {
        response->success = false;
        response->message = "Unknown logical_id: " + std::to_string(request->logical_id);
        return;
      }
      const auto idx = static_cast<std::size_t>(motor_idx);
      response->success = transport_->reboot(idx);
      response->message = response->success ? "Reboot queued for " + configs_[idx].name +
                                                "; wait for fresh feedback and send new commands"
                                            : "Reboot already pending or transport unavailable";
    });
}

/// @brief 状態配列と異常通知の publisher，状態の周期タイマーを作る
void VescNode::initialize_state_publishers(std::chrono::milliseconds period)
{
  const auto error_topic = declare_parameter<std::string>("error_topic", "vesc/event");
  const auto state_topic = declare_parameter<std::string>("state_topic", "vesc/state");
  state_pub_ = std::make_unique<MotorStatePublisher>(*this, configs_, error_topic, state_topic);
  state_timer_ = create_wall_timer(period, [this]() { publish_states(); });
}

/// @brief 論理 ID を検査し，指令配列をまとめて最新値の保存領域へ渡す
/// @note 一部のモータだけ指定でき、未指定モータの指令期限は更新しない
/// @note msg.stamp は送信側の生成時刻，期限判定には受信側の単調時刻を使う
void VescNode::command_callback(const VescCommandArray &msg)
{
  cmds_.clear();
  for (const auto &cmd : msg.commands) {
    const auto motor_idx = motor_idx_[cmd.logical_id];
    if (motor_idx < 0) {
      transport_->stop_on_error("Unknown logical_id: " + std::to_string(cmd.logical_id));
      return;
    }
    cmds_.push_back({static_cast<std::size_t>(motor_idx), cmd.value});
  }
  transport_->set_commands(cmds_);
}

/// @brief 確保済みの領域へ最新状態を取得し，状態だけを周期配信する
void VescNode::publish_states()
{
  // On lock contention, retain the snapshot and continue applying packet expiry checks.
  transport_->try_snapshot(states_);
  state_pub_->publish(states_, SteadyClock::now(), now());
}
}  // namespace vesc_can_driver
