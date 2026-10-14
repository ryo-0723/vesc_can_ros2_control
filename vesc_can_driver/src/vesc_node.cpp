#include <array>
#include <bitset>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "vesc_can_driver/vesc_driver.hpp"
#include "vesc_can_driver/vesc_state.hpp"
#include "vesc_can_msgs/msg/vesc_command_array.hpp"
#include "vesc_can_msgs/srv/reboot.hpp"

namespace vesc_can_driver
{
using VescCommandArray = vesc_can_msgs::msg::VescCommandArray;
using Reboot = vesc_can_msgs::srv::Reboot;

class VescNode : public rclcpp::Node
{
public:
  /// @brief パラメータを読み込み、通信・指令受付・状態配信を起動する
  VescNode() : rclcpp::Node("vesc_can_ros2_control")
  {
    motor_idx_.fill(-1);
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
    cmds_.reserve(configs_.size());
    states_.resize(configs_.size());
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
    if (motor_id < 0 || motor_id > 255 || motor_idx_[motor_id] >= 0) {
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
    motor_idx_[motor_id] = static_cast<int>(configs_.size());
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
        const auto motor_idx = motor_idx_[request->motor_id];
        if (motor_idx < 0) {
          response->success = false;
          response->message = "Unknown logical motor ID: " + std::to_string(request->motor_id);
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
  void initialize_state_publishers(std::chrono::milliseconds period)
  {
    const auto error_topic = declare_parameter<std::string>("error_topic", "/vesc/event");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/vesc/state");
    state_pub_ = std::make_unique<MotorStatePublisher>(*this, configs_, error_topic, state_topic);
    state_timer_ = create_wall_timer(period, [this]() { publish_states(); });
  }

  /// @brief 論理 ID を検査し，指令配列をまとめて最新値の保存領域へ渡す
  /// @note 一部のモータだけ指定でき、未指定モータの指令期限は更新しない
  /// @note msg.stamp は送信側の生成時刻，期限判定には受信側の単調時刻を使う
  void handle_commands(const VescCommandArray &msg)
  {
    cmds_.clear();
    std::bitset<256> seen;
    for (const auto &cmd : msg.commands) {
      const auto motor_idx = motor_idx_[cmd.motor_id];
      if (motor_idx < 0 || seen.test(cmd.motor_id)) {
        transport_->stop_on_error(
          "Unknown or duplicate logical motor ID: " + std::to_string(cmd.motor_id));
        return;
      }
      seen.set(cmd.motor_id);
      cmds_.push_back({static_cast<std::size_t>(motor_idx), cmd.value});
    }
    transport_->set_commands(cmds_);
  }

  /// @brief 確保済みの領域へ最新状態を取得し，状態だけを周期配信する
  void publish_states()
  {
    // On lock contention, retain the snapshot and continue applying packet expiry checks.
    transport_->try_snapshot(states_);
    state_pub_->publish(states_, SteadyClock::now(), now());
  }

  std::vector<MotorConfig> configs_;
  std::array<int, 256> motor_idx_;
  std::vector<MotorCommand> cmds_;
  std::vector<MotorState> states_;
  // The transport joins its callbacks before the state/error publishers are destroyed.
  std::unique_ptr<MotorStatePublisher> state_pub_;
  std::unique_ptr<CanTransport> transport_;
  rclcpp::Subscription<VescCommandArray>::SharedPtr cmd_sub_;
  rclcpp::Service<Reboot>::SharedPtr reboot_srv_;
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
