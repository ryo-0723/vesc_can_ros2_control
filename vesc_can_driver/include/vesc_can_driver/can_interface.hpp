#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vesc_can_driver/can_protocol.hpp"

namespace vesc_can_driver
{
struct CanOptions
{
  std::string tx_topic{"can/tx"};
  std::string rx_topic{"can/rx"};
  std::chrono::milliseconds fault_reply_timeout{100};
};

/// @brief ROS の CAN トピックと VESC の CAN プロトコルを接続する
/// @note node は VescTransport の専用 executor に登録する
class CanInterface final : public VescInterface
{
public:
  /// @brief CAN トピックを作成し，受信通知は start() で設定する
  /// @param motor_count 送信キューの深さに使う登録台数
  CanInterface(rclcpp::Node::SharedPtr node, const CanOptions &options, std::size_t motor_count);
  ~CanInterface() override;
  void configure_targets(std::span<const std::uint8_t> ids) override;
  void start(FeedbackCallback on_feedback, ErrorCallback on_error) override;
  void stop() noexcept override;
  void send(const VescCommand &command) override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr tx_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr rx_sub_;
  std::mutex callback_mutex_;
  std::array<bool, 256> targets_{};
  std::array<std::optional<std::chrono::steady_clock::time_point>, 256> fault_requests_;
  std::chrono::milliseconds fault_reply_timeout_;
  FeedbackCallback on_feedback_;
  ErrorCallback on_error_;
};
}  // namespace vesc_can_driver
