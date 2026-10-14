#pragma once

#include <vector>

#include "vesc_can_driver/vesc_driver.hpp"
#include "vesc_can_msgs/msg/vesc_event.hpp"
#include "vesc_can_msgs/msg/vesc_state.hpp"
#include "vesc_can_msgs/msg/vesc_state_array.hpp"

namespace vesc_can_driver
{
constexpr std::int16_t kUnknownFaultCode = -1;

/// @brief 速度と角度を関節単位へ変換し，監視値を float32 の VescState へ整理する
/// @param cfg 論理 ID・単位変換・フィードバック期限を持つ設定
/// @param state 最終受信状態
/// @param now 鮮度判定に使う単調時刻
/// @note 欠測・期限切れの浮動値は NaN，故障コードは -1
vesc_can_msgs::msg::VescState make_vesc_state(
  const MotorConfig &cfg, const MotorState &state, TimePoint now);

/// @brief 通常ノードと hardware plugin でモータ状態の配信を共用
/// @note ROS executor から呼び出し、hardware read/write では配信しない
class MotorStatePublisher
{
public:
  /// @brief 状態と異常通知用 publisher を作成する
  /// @param error_topic 異常検出時に通知するトピック
  /// @param state_topic 全台の配列を配信するトピック，空ならモータ別に配信する
  MotorStatePublisher(rclcpp::Node &node, const std::vector<MotorConfig> &configs,
    const std::string &error_topic = "/vesc/event", const std::string &state_topic = "");
  /// @brief 全モータの VescState を同じ ROS 時刻で配信
  /// @param states 設定と同じ順序の状態配列
  /// @param now 鮮度判定に使う単調時刻
  /// @param stamp メッセージに付ける ROS 時刻
  void publish(const std::vector<MotorState> &states, TimePoint now, const rclcpp::Time &stamp);

  /// @brief 異常が発生したモータの論理 ID とエラー文字列を即時配信する
  /// @note 本体の故障は該当 ID，PC 側の停止では影響する全 ID に通知する
  void publish_error(const DriverFault &fault);

private:
  std::vector<MotorConfig> configs_;
  std::vector<rclcpp::Publisher<vesc_can_msgs::msg::VescState>::SharedPtr> pubs_;
  rclcpp::Publisher<vesc_can_msgs::msg::VescStateArray>::SharedPtr array_pub_;
  vesc_can_msgs::msg::VescStateArray state_msg_;
  rclcpp::Publisher<vesc_can_msgs::msg::VescEvent>::SharedPtr error_pub_;
};
}  // namespace vesc_can_driver
