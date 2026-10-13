#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vesc_can/driver_core.hpp"

namespace vesc_can
{
struct TransportOptions
{
  std::string can_tx_topic{"/can/tx"};
  std::string can_rx_topic{"/can/rx"};
  std::chrono::milliseconds send_period{10};
};

// Owns a private executor thread. ROS publication never runs in read()/write().
class CanTransport
{
public:
  CanTransport(std::vector<MotorConfig> motor_configs, TransportOptions transport_options,
    rclcpp::Node::SharedPtr can_node);
  ~CanTransport();
  CanTransport(const CanTransport &) = delete;
  CanTransport &operator=(const CanTransport &) = delete;
  void start();
  void shutdown() noexcept;
  bool wait_ready(std::chrono::milliseconds timeout);
  bool enable();
  void disable();
  void request_stop() noexcept
  {
    fault_requested_.store(true);
    stop_requested_.store(true);
  }
  bool enabled() const
  {
    return enabled_.load();
  }
  bool faulted() const
  {
    return faulted_.load();
  }
  bool try_set_commands(const std::vector<double> &command_values);
  bool set_command(std::size_t motor_index, double command_value);
  bool try_snapshot(std::vector<MotorState> &motor_states);
  std::vector<MotorState> snapshot();
  std::string fault_reason();

private:
  struct PendingCommand
  {
    double command_value{};
    TimePoint received_at{};
    bool has_pending_update{false};
  };
  void send_commands();
  DriverCore driver_core_;
  TransportOptions transport_options_;
  rclcpp::Node::SharedPtr can_node_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_subscription_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::executors::SingleThreadedExecutor can_executor_;
  std::thread executor_thread_;
  std::mutex state_mutex_;
  std::condition_variable feedback_changed_;
  std::vector<PendingCommand> pending_commands_;
  std::vector<can_msgs::msg::Frame> command_frames_;
  std::atomic<bool> shutdown_started_{false};
  std::atomic<bool> stop_requested_{true};
  std::atomic<bool> enabled_{false};
  std::atomic<bool> faulted_{false};
  std::atomic<bool> fault_requested_{false};
};
}  // namespace vesc_can
