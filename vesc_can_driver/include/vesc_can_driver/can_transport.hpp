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
#include "vesc_can_driver/driver_core.hpp"

namespace vesc_can_driver
{
struct TransportOptions
{
  std::string tx_topic{"/can/tx"};
  std::string rx_topic{"/can/rx"};
  std::chrono::milliseconds send_period{10};
};

// Owns a private executor thread. ROS publication never runs in read()/write().
class CanTransport
{
public:
  CanTransport(
    std::vector<MotorConfig> configs, TransportOptions opts, rclcpp::Node::SharedPtr can_node);
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
  bool try_set_commands(const std::vector<double> &values);
  bool set_command(std::size_t motor_idx, double value);
  bool try_snapshot(std::vector<MotorState> &states);
  std::vector<MotorState> snapshot();
  std::string fault_reason();

private:
  struct PendingCommand
  {
    double value{};
    TimePoint received_at{};
    bool updated{false};
  };
  void send_commands();
  DriverCore core_;
  TransportOptions opts_;
  rclcpp::Node::SharedPtr can_node_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr tx_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr rx_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::executors::SingleThreadedExecutor can_exec_;
  std::thread exec_thread_;
  std::mutex state_mutex_;
  std::condition_variable feedback_cv_;
  std::vector<PendingCommand> pending_cmds_;
  std::vector<can_msgs::msg::Frame> cmd_frames_;
  std::atomic<bool> shutdown_started_{false};
  std::atomic<bool> stop_requested_{true};
  std::atomic<bool> enabled_{false};
  std::atomic<bool> faulted_{false};
  std::atomic<bool> fault_requested_{false};
};
}  // namespace vesc_can_driver
