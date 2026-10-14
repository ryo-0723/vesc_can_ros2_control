#pragma once

#include <vector>

#include "vesc_can_driver/vesc_driver.hpp"
#include "vesc_msgs/msg/vesc_state.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

namespace vesc_can_driver
{
// Native STATUS frames do not include VESC fault codes or absolute tachometer counts.
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
  /// @brief 各モータの状態トピック用 publisher を作成
  MotorStatePublisher(rclcpp::Node &node, const std::vector<MotorConfig> &configs);
  /// @brief 全モータの VescStateStamped を同じ ROS 時刻で配信
  /// @param states 設定と同じ順序の状態配列
  /// @param now 鮮度判定に使う単調時刻
  /// @param stamp メッセージに付ける ROS 時刻
  void publish(const std::vector<MotorState> &states, TimePoint now, const rclcpp::Time &stamp);

private:
  std::vector<MotorConfig> configs_;
  std::vector<rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr> pubs_;
};
}  // namespace vesc_can_driver
