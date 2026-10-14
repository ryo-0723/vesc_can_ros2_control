#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "vesc_can_driver/vesc_driver.hpp"
#include "vesc_can_driver/vesc_state.hpp"
#include "vesc_can_msgs/msg/vesc_command_array.hpp"
#include "vesc_can_msgs/srv/reboot.hpp"

namespace vesc_can_driver
{
using VescCommandArray = vesc_can_msgs::msg::VescCommandArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using Reboot = vesc_can_msgs::srv::Reboot;

class VescNode : public rclcpp::Node
{
public:
  /// @brief パラメータを読み込み、通信・指令受付・状態配信を起動する
  VescNode() : rclcpp::Node("vesc_can_ros2_control")
  {
    TransportOptions opts;
    opts.tx_topic = declare_parameter<std::string>("can_tx_topic", "/socketcan_bridge/tx");
    opts.rx_topic = declare_parameter<std::string>("can_rx_topic", "/socketcan_bridge/rx");
    opts.send_period =
      std::chrono::milliseconds(declare_parameter<std::int64_t>("update_period_ms", 10));
    const auto host_id = declare_parameter<std::int64_t>("host_can_id", 254);
    if (host_id < 0 || host_id > 254) {
      throw std::invalid_argument("Host CAN ID must be in [0, 254]");
    }
    opts.host_id = static_cast<std::uint8_t>(host_id);
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
    initialize_command_interfaces();
    initialize_state_publishers(state_period);
    initialize_can_transport(opts);
  }

private:
  /// @brief モータ名を接頭辞に持つ ROS パラメータから設定を読み込む
  MotorConfig load_motor_config(const std::string &motor_name)
  {
    const auto prefix = motor_name + ".";
    MotorConfig cfg;
    cfg.name = motor_name;
    const auto motor_id = declare_parameter<std::int64_t>(prefix + "logical_id", -1);
    if (motor_id < 0 || motor_id > 255 ||
        std::find(motor_ids_.begin(), motor_ids_.end(), motor_id) != motor_ids_.end()) {
      throw std::invalid_argument(motor_name + ": logical ID must be unique and in [0, 255]");
    }
    const auto controller_id = declare_parameter<std::int64_t>(prefix + "controller_id", -1);
    if (controller_id < 0 || controller_id > 254) {
      throw std::invalid_argument(motor_name + ": controller ID must be in [0, 254]");
    }
    cfg.controller_id = static_cast<std::uint8_t>(controller_id);

    rcl_interfaces::msg::ParameterDescriptor fixed_mode;
    fixed_mode.read_only = true;
    fixed_mode.description = "Control mode is fixed at node startup";
    const auto mode_name = declare_parameter<std::string>(prefix + "control_mode", "", fixed_mode);
    const auto default_interface =
      mode_name.empty() ? std::string("velocity")
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
    cfg.startup_timeout = std::chrono::milliseconds(
      declare_parameter<std::int64_t>(prefix + "startup_timeout_ms", 1000));
    cfg.feedback_timeout = std::chrono::milliseconds(
      declare_parameter<std::int64_t>(prefix + "feedback_timeout_ms", 500));
    cfg.command_timeout = std::chrono::milliseconds(
      declare_parameter<std::int64_t>(prefix + "command_timeout_ms", 500));
    cfg.require_tachometer = declare_parameter<bool>(prefix + "require_tachometer", false);
    motor_ids_.push_back(static_cast<std::uint8_t>(motor_id));
    return cfg;
  }

  /// @brief 同じ namespace / context で通信ノードと専用 executor を起動する
  void initialize_can_transport(const TransportOptions &opts)
  {
    auto node_opts = rclcpp::NodeOptions().use_global_arguments(false);
    node_opts.context(get_node_base_interface()->get_context());
    auto can_node = std::make_shared<rclcpp::Node>("vesc_can_io", get_namespace(), node_opts);
    transport_ = std::make_unique<CanTransport>(configs_, opts, can_node,
      [pub = state_pub_.get()](const DriverFault &fault) { pub->publish_error(fault); });
    transport_->start();
  }

  /// @brief 全モータ共通の指令トピックと reboot サービスを登録する
  void initialize_command_interfaces()
  {
    const auto cmd_topic = declare_parameter<std::string>("command_topic", "/vesc/commands");
    cmd_sub_ = create_subscription<VescCommandArray>(
      cmd_topic, 10, [this](VescCommandArray::ConstSharedPtr msg) { handle_commands(*msg); });
    reboot_srv_ = create_service<Reboot>(
      "~/reboot", [this](Reboot::Request::SharedPtr request, Reboot::Response::SharedPtr response) {
        const auto id = std::find(motor_ids_.begin(), motor_ids_.end(), request->motor_id);
        if (id == motor_ids_.end()) {
          response->success = false;
          response->message = "Unknown logical motor ID: " + std::to_string(request->motor_id);
          return;
        }
        const auto idx = static_cast<std::size_t>(id - motor_ids_.begin());
        response->success = transport_->reboot(idx);
        response->message = response->success ? "Reboot queued for " + configs_[idx].name +
                                                  "; wait for fresh feedback and send new commands"
                                              : "Reboot already pending or transport unavailable";
      });
  }

  /// @brief VescState・診断の publisher と周期タイマーを作る
  void initialize_state_publishers(std::chrono::milliseconds period)
  {
    const auto error_topic = declare_parameter<std::string>("error_topic", "/vesc/event");
    state_pub_ = std::make_unique<MotorStatePublisher>(*this, configs_, error_topic);
    diag_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    state_timer_ = create_wall_timer(period, [this]() { publish_states(); });
  }

  /// @brief 論理 ID・指令値を検査し、起動時に設定したモードの最新指令へ渡す
  /// @note 一部のモータだけ指定でき、未指定モータの指令期限は更新しない
  /// @note msg.stamp は送信側の生成時刻，期限判定には受信側の単調時刻を使う
  void handle_commands(const VescCommandArray &msg)
  {
    std::vector<std::size_t> motor_indices;
    motor_indices.reserve(msg.commands.size());
    for (const auto &cmd : msg.commands) {
      const auto id_it = std::find(motor_ids_.begin(), motor_ids_.end(), cmd.motor_id);
      if (id_it == motor_ids_.end()) {
        transport_->stop_on_error("Unknown logical motor ID: " + std::to_string(cmd.motor_id));
        return;
      }
      const auto motor_idx = static_cast<std::size_t>(id_it - motor_ids_.begin());
      const auto &cfg = configs_[motor_idx];
      if (std::find(motor_indices.begin(), motor_indices.end(), motor_idx) != motor_indices.end()) {
        transport_->stop_on_error(
          cfg.name + ": duplicate logical ID " + std::to_string(cmd.motor_id));
        return;
      }
      if (!command_is_valid(cfg, cmd.value)) {
        transport_->stop_on_error(
          cfg.name + ": invalid command value " + std::to_string(cmd.value));
        return;
      }
      motor_indices.push_back(motor_idx);
    }
    for (std::size_t cmd_idx = 0; cmd_idx < motor_indices.size(); ++cmd_idx) {
      transport_->set_command(motor_indices[cmd_idx], msg.commands[cmd_idx].value);
    }
  }

  /// @brief 診断メッセージへキーと値を追加する
  static void add_diagnostic_value(
    DiagnosticStatus &msg, const std::string &key, const std::string &value)
  {
    diagnostic_msgs::msg::KeyValue field;
    field.key = key;
    field.value = value;
    msg.values.push_back(field);
  }

  /// @brief 準備・動作許可・異常と STATUS の鮮度を診断メッセージにまとめる
  DiagnosticStatus make_diagnostic_status(const MotorConfig &cfg, std::uint8_t motor_id,
    const MotorState &state, TimePoint steady_now, bool ready, bool active, bool faulted,
    const std::string &fault_reason) const
  {
    DiagnosticStatus msg;
    msg.name = std::string(get_namespace()) + "/vesc/" + cfg.name;
    msg.hardware_id = "vesc_can_id_" + std::to_string(cfg.controller_id);
    if (faulted) {
      msg.level = DiagnosticStatus::ERROR;
      msg.message = fault_reason;
    } else if (!ready) {
      msg.level = DiagnosticStatus::ERROR;
      msg.message = "Required feedback missing/stale or position out of range";
    } else if (active) {
      msg.level = DiagnosticStatus::OK;
      msg.message = "Commanding";
    } else {
      msg.level = DiagnosticStatus::WARN;
      msg.message = "Waiting for commands";
    }
    const auto timeout = cfg.feedback_timeout;
    const auto add_freshness = [&](const std::string &key,
                                 const std::optional<TimePoint> &received_at) {
      const bool fresh = feedback_fresh(received_at, steady_now, timeout);
      add_diagnostic_value(msg, key, fresh ? "true" : "false");
    };
    add_freshness("status_1_fresh", state.status1_at);
    add_freshness("status_2_fresh", state.status2_at);
    add_freshness("status_3_fresh", state.status3_at);
    add_freshness("status_4_fresh", state.status4_at);
    add_freshness("status_5_fresh", state.status5_at);
    add_freshness("status_6_fresh", state.status6_at);
    add_diagnostic_value(msg, "active", active ? "true" : "false");
    add_diagnostic_value(msg, "logical_id", std::to_string(motor_id));
    add_diagnostic_value(msg, "driver_faulted", faulted ? "true" : "false");
    add_freshness("fault_code_fresh", state.fault_at);
    add_diagnostic_value(msg, "fault_code",
      std::to_string(feedback_fresh(state.fault_at, steady_now, timeout) ? state.fault_code : -1));
    const bool status_6_fresh = feedback_fresh(state.status6_at, steady_now, timeout);
    add_diagnostic_value(
      msg, "adc_1_v", std::to_string(status_6_fresh ? state.adc_1_v : kUnknownValue));
    add_diagnostic_value(
      msg, "adc_2_v", std::to_string(status_6_fresh ? state.adc_2_v : kUnknownValue));
    add_diagnostic_value(
      msg, "adc_3_v", std::to_string(status_6_fresh ? state.adc_3_v : kUnknownValue));
    add_diagnostic_value(msg, "ppm", std::to_string(status_6_fresh ? state.ppm : kUnknownValue));
    return msg;
  }

  /// @brief 最新状態を取得し、VescState・診断を周期配信する
  void publish_states()
  {
    const auto states = transport_->snapshot();
    const auto steady_now = SteadyClock::now();
    const auto stamp = now();
    state_pub_->publish(states, steady_now, stamp);
    const bool active = transport_->active();
    const bool faulted = transport_->faulted();
    const auto fault_reason = faulted ? transport_->fault_reason() : std::string{};
    DiagnosticArray diag_msg;
    diag_msg.header.stamp = stamp;
    for (std::size_t motor_idx = 0; motor_idx < configs_.size(); ++motor_idx) {
      const auto &cfg = configs_[motor_idx];
      const auto &state = states[motor_idx];
      const bool ready = state_ready(cfg, state, steady_now);
      diag_msg.status.push_back(make_diagnostic_status(
        cfg, motor_ids_[motor_idx], state, steady_now, ready, active, faulted, fault_reason));
    }
    diag_pub_->publish(diag_msg);
  }

  std::vector<MotorConfig> configs_;
  std::vector<std::uint8_t> motor_ids_;
  // The transport joins its callbacks before the state/error publishers are destroyed.
  std::unique_ptr<MotorStatePublisher> state_pub_;
  std::unique_ptr<CanTransport> transport_;
  rclcpp::Subscription<VescCommandArray>::SharedPtr cmd_sub_;
  rclcpp::Service<Reboot>::SharedPtr reboot_srv_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};
}  // namespace vesc_can_driver

/// @brief 通常ノードを起動し、終了時に ROS を停止する
/// @return 正常終了で 0、ノード実行中の例外で 1
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<vesc_can_driver::VescNode>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("vesc_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
