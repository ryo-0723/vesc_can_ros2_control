#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "vesc_can_driver/msg/motor_commands.hpp"
#include "vesc_can_driver/vesc_driver.hpp"
#include "vesc_can_driver/vesc_state.hpp"

namespace vesc_can_driver
{
using MotorCommands = vesc_can_driver::msg::MotorCommands;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using SetBool = std_srvs::srv::SetBool;

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
    const auto state_period =
      std::chrono::milliseconds(declare_parameter<std::int64_t>("state_publish_period_ms", 20));

    if (state_period.count() <= 0) {
      throw std::invalid_argument("State period must be positive");
    }
    auto_enable_once_ = declare_parameter<bool>("auto_enable_once", false);

    const auto names =
      declare_parameter<std::vector<std::string>>("motors", std::vector<std::string>{});
    configs_.reserve(names.size());

    for (const auto &motor_name : names) {
      configs_.push_back(load_motor_config(motor_name));
    }
    initialize_can_transport(opts);
    initialize_command_interfaces();
    initialize_state_publishers(state_period);
  }

private:
  /// @brief モータ名を接頭辞に持つ ROS パラメータから設定を読み込む
  MotorConfig load_motor_config(const std::string &motor_name)
  {
    const auto prefix = motor_name + ".";
    MotorConfig cfg;
    cfg.name = motor_name;
    const auto controller_id = declare_parameter<std::int64_t>(prefix + "controller_id", -1);
    if (controller_id < 0 || controller_id > 254) {
      throw std::invalid_argument(motor_name + ": controller ID must be in [0, 254]");
    }
    cfg.controller_id = static_cast<std::uint8_t>(controller_id);

    const auto mode_name = declare_parameter<std::string>(prefix + "control_mode", "");
    const auto default_interface =
      mode_name.empty() ? std::string("velocity")
                        : std::string(command_interface(parse_control_mode(mode_name)));
    const auto cmd_interface =
      declare_parameter<std::string>(prefix + "command_interface", default_interface);
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
    cfg.startup_current_a = declare_parameter<double>(prefix + "startup_current_a", 0.0);
    cfg.rpm_threshold = declare_parameter<double>(prefix + "rpm_control_threshold_rpm", 1000.0);
    cfg.startup_timeout = std::chrono::milliseconds(
      declare_parameter<std::int64_t>(prefix + "startup_timeout_ms", 1000));
    cfg.feedback_timeout = std::chrono::milliseconds(
      declare_parameter<std::int64_t>(prefix + "feedback_timeout_ms", 500));
    cfg.command_timeout = std::chrono::milliseconds(
      declare_parameter<std::int64_t>(prefix + "command_timeout_ms", 500));
    cfg.require_tachometer = declare_parameter<bool>(prefix + "require_tachometer", false);
    return cfg;
  }

  /// @brief 同じ namespace / context で通信ノードと専用 executor を起動する
  void initialize_can_transport(const TransportOptions &opts)
  {
    auto node_opts = rclcpp::NodeOptions().use_global_arguments(false);
    node_opts.context(get_node_base_interface()->get_context());
    auto can_node = std::make_shared<rclcpp::Node>("vesc_can_io", get_namespace(), node_opts);
    transport_ = std::make_unique<CanTransport>(configs_, opts, can_node);
    transport_->start();
  }

  /// @brief 全モータ共通の指令トピックと enable サービスを登録する
  void initialize_command_interfaces()
  {
    cmd_sub_ = create_subscription<MotorCommands>(
      "~/commands", 10, [this](MotorCommands::ConstSharedPtr msg) { handle_commands(*msg); });
    enable_srv_ = create_service<SetBool>("~/enable",
      [this](SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
        handle_enable_request(*request, *response);
      });
  }

  /// @brief VescState・診断の publisher と周期タイマーを作る
  void initialize_state_publishers(std::chrono::milliseconds period)
  {
    state_pub_ = std::make_unique<MotorStatePublisher>(*this, configs_);
    diag_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    state_timer_ = create_wall_timer(period, [this]() { publish_states(); });
  }

  /// @brief モータ名と指令値を検査し、設定したモードの最新指令へ渡す
  /// @note 一部のモータだけ指定でき、未指定モータの指令期限は更新しない
  void handle_commands(const MotorCommands &msg)
  {
    if (msg.names.size() != msg.values.size()) {
      transport_->request_stop();
      RCLCPP_ERROR(get_logger(), "Motor command names and values must have equal lengths");
      return;
    }
    std::vector<std::size_t> motor_indices;
    motor_indices.reserve(msg.names.size());
    for (std::size_t cmd_idx = 0; cmd_idx < msg.names.size(); ++cmd_idx) {
      const auto cfg_it = std::find_if(configs_.begin(), configs_.end(),
        [&](const MotorConfig &cfg) { return cfg.name == msg.names[cmd_idx]; });
      if (cfg_it == configs_.end()) {
        transport_->request_stop();
        RCLCPP_ERROR(get_logger(), "Unknown motor name: %s", msg.names[cmd_idx].c_str());
        return;
      }
      const auto motor_idx = static_cast<std::size_t>(cfg_it - configs_.begin());
      if (std::find(motor_indices.begin(), motor_indices.end(), motor_idx) != motor_indices.end() ||
          !command_is_valid(*cfg_it, msg.values[cmd_idx])) {
        transport_->request_stop();
        RCLCPP_ERROR(
          get_logger(), "Duplicate motor name or invalid command: %s", cfg_it->name.c_str());
        return;
      }
      motor_indices.push_back(motor_idx);
    }
    for (std::size_t cmd_idx = 0; cmd_idx < motor_indices.size(); ++cmd_idx) {
      transport_->set_command(motor_indices[cmd_idx], msg.values[cmd_idx]);
    }
  }

  /// @brief 有効化・無効化要求を処理し、自動有効化を解除する
  void handle_enable_request(const SetBool::Request &request, SetBool::Response &response)
  {
    auto_enable_once_ = false;
    if (!request.data) {
      transport_->disable();
      response.success = true;
      response.message = "Disabled";
      return;
    }
    response.success = transport_->enable();
    if (response.success) {
      response.message = "Enabled; send fresh commands for every motor";
    } else {
      response.message = "Fresh feedback from every motor is required";
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
  DiagnosticStatus make_diagnostic_status(const MotorConfig &cfg, const MotorState &state,
    TimePoint steady_now, bool ready, bool enabled, bool faulted,
    const std::string &fault_reason) const
  {
    DiagnosticStatus msg;
    msg.name = std::string(get_namespace()) + "/vesc/" + cfg.name;
    msg.hardware_id = "vesc_can_id_" + std::to_string(cfg.controller_id);
    if (!ready) {
      msg.level = DiagnosticStatus::ERROR;
      msg.message = "Required feedback missing/stale or position out of range";
    } else if (faulted) {
      msg.level = DiagnosticStatus::ERROR;
      msg.message = fault_reason;
    } else if (enabled) {
      msg.level = DiagnosticStatus::OK;
      msg.message = "Enabled";
    } else {
      msg.level = DiagnosticStatus::WARN;
      msg.message = "Disabled";
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
    add_diagnostic_value(msg, "enabled", enabled ? "true" : "false");
    add_diagnostic_value(msg, "driver_faulted", faulted ? "true" : "false");
    add_diagnostic_value(msg, "unavailable_vesc_fields",
      "avg_id,avg_iq,ntc_temp_mos1,ntc_temp_mos2,ntc_temp_mos3,avg_vd,avg_vq,"
      "distance_traveled,fault_code");
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
  /// @note auto_enable_once が有効なら初回の準備完了時だけ有効化する
  void publish_states()
  {
    if (auto_enable_once_ && transport_->enable()) {
      auto_enable_once_ = false;
    }
    const auto states = transport_->snapshot();
    const auto steady_now = SteadyClock::now();
    const auto stamp = now();
    state_pub_->publish(states, steady_now, stamp);
    const bool enabled = transport_->enabled();
    const bool faulted = transport_->faulted();
    const auto fault_reason = faulted ? transport_->fault_reason() : std::string{};
    DiagnosticArray diag_msg;
    diag_msg.header.stamp = stamp;
    for (std::size_t motor_idx = 0; motor_idx < configs_.size(); ++motor_idx) {
      const auto &cfg = configs_[motor_idx];
      const auto &state = states[motor_idx];
      const bool ready = state_ready(cfg, state, steady_now);
      diag_msg.status.push_back(
        make_diagnostic_status(cfg, state, steady_now, ready, enabled, faulted, fault_reason));
    }
    diag_pub_->publish(diag_msg);
  }

  std::vector<MotorConfig> configs_;
  bool auto_enable_once_{false};
  std::unique_ptr<CanTransport> transport_;
  rclcpp::Subscription<MotorCommands>::SharedPtr cmd_sub_;
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;
  std::unique_ptr<MotorStatePublisher> state_pub_;
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
