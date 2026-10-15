#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "vesc_can_driver/driver_core.hpp"
#include "vesc_can_driver/vesc_node.hpp"

namespace vesc_can_ros2_control
{
class VescSystem : public hardware_interface::SystemInterface
{
public:
  /// @brief 通信スレッドと状態配信リソースを終了する
  ~VescSystem() override;
  /// @brief URDF の設定・interface を検査し、状態と指令の領域を確保する
  /// @param params ros2_control が渡す hardware 設定と初期化情報
  /// @return 設定が有効なら SUCCESS、それ以外は ERROR
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) override;
  /// @brief CAN 通信と状態配信を起動し、interface のハンドルを保存する
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  /// @brief 新鮮な状態を待ち、指令の初期値を設定して有効化する
  /// @note 位置は現在角度、その他はゼロを初期値にする
  /// @return 有効化に成功すれば SUCCESS、それ以外は ERROR
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  /// @brief 指令を無効化し、CAN 送信をゼロ電流へ切り替える
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  /// @brief 通信と状態配信のリソースを解放する
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  /// @brief shutdown 時に通信と状態配信を終了する
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  /// @brief lifecycle の異常時に通信と状態配信を終了する
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  /// @brief ロックを待たずに状態を取得し、state interface を更新する
  /// @return 通信未構成・有効中の異常で ERROR、それ以外は OK
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  /// @brief command interface の値を検査し、CAN 送信用の最新指令を渡す
  /// @return 不正指令・有効中の通信異常で ERROR、正常・再試行待ちで OK
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  enum class StateField
  {
    Position,
    Velocity,
    Current,
    TemperatureFet,
    TemperatureMotor,
    InputVoltage,
    DutyCycle
  };
  struct StateHandle
  {
    std::size_t motor_idx{};
    StateField field{};
    hardware_interface::StateInterface::SharedPtr handle;
  };
  /// @brief state interface 名を内部の状態項目へ変換する
  static StateField state_field_from_name(const std::string &interface_name);
  /// @brief 1 関節のパラメータと interface からモータ設定を作成する
  static vesc_can_driver::MotorConfig load_joint_config(
    const hardware_interface::ComponentInfo &joint);
  /// @brief 制御周期で使う command / state interface のハンドルを保存する
  void cache_interface_handles();
  /// @brief 通信スレッドを join してから状態タイマーと publisher を解放する
  void release_can_transport();
  /// @brief 対応 STATUS の鮮度と方向を反映して interface の値を返す
  /// @return 取得できる値，欠測・期限切れでは NaN
  double read_state_value(const StateHandle &binding, vesc_can_driver::TimePoint steady_now) const;

  std::vector<vesc_can_driver::MotorConfig> configs_;
  std::vector<vesc_can_driver::MotorState> states_;
  std::vector<vesc_can_driver::MotorState> pub_states_;  // CAN executor only
  std::vector<double> cmds_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> cmd_handles_;
  std::vector<StateHandle> state_handles_;
  vesc_can_driver::TransportOptions opts_;
  vesc_can_driver::CanOptions can_opts_;
  std::chrono::milliseconds activation_timeout_{2000};
  std::chrono::milliseconds state_period_{20};
  // Transport must join its callbacks before publishers are destroyed.
  std::unique_ptr<vesc_can_driver::MotorStatePublisher> state_pub_;
  std::unique_ptr<vesc_can_driver::VescTransport> transport_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  bool active_{false};
};
}  // namespace vesc_can_ros2_control
