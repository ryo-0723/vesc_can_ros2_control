#pragma once

#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "vesc_can_driver/vesc_driver.hpp"
#include "vesc_msgs/msg/vesc_state.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

namespace vesc_can_driver
{
// Missing/expired VESC integer values must not be reported as healthy zeros.
constexpr std::int32_t kUnavailableVescInteger = -1;

/// @brief VESC 生値を上流 VescState メッセージへ変換
/// @param cfg モータ ID とフィードバック期限を持つ設定
/// @param state 最終受信状態
/// @param now 鮮度判定に使う単調時刻
/// @note 欠測・期限切れの浮動値は NaN，displacement は最終カウントを保持する
/// @note displacement の鮮度は診断の status_5_fresh で確認する
vesc_msgs::msg::VescState make_vesc_state(
  const MotorConfig &cfg, const MotorState &state, TimePoint now);

/// @brief 通常ノードと hardware plugin でモータ状態の配信を共用
/// @note ROS executor から呼び出し、hardware read/write では配信しない
class MotorStatePublisher
{
public:
  /// @brief 各モータの状態と共通の異常通知用 publisher を作成
  /// @param error_topic 異常検出時に通知するトピック
  MotorStatePublisher(rclcpp::Node &node, const std::vector<MotorConfig> &configs,
    const std::string &error_topic = "/vesc/event");
  /// @brief 全モータの VescStateStamped を同じ ROS 時刻で配信
  /// @param states 設定と同じ順序の状態配列
  /// @param now 鮮度判定に使う単調時刻
  /// @param stamp メッセージに付ける ROS 時刻
  void publish(const std::vector<MotorState> &states, TimePoint now, const rclcpp::Time &stamp);

  /// @brief VESC 故障または PC 側の異常を DiagnosticArray として即時配信
  /// @note 本体の故障では該当モータの ID・故障コード，PC 側では全台の停止理由を通知
  void publish_error(const DriverFault &fault);

private:
  std::vector<MotorConfig> configs_;
  std::vector<rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr> pubs_;
  rclcpp::Clock::SharedPtr clock_;
  std::string node_name_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr error_pub_;
};
}  // namespace vesc_can_driver
