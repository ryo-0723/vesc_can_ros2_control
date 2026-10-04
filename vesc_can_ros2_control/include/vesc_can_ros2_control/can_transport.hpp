#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vesc_can_ros2_control/driver_core.hpp"

namespace vesc_can_ros2_control
{
struct TransportOptions
{
  std::string tx_topic{"/can/tx"};
  std::string rx_topic{"/can/rx"};
  std::chrono::milliseconds period{10};
};

// Owns a private executor thread. ROS publication never runs in read()/write().
class CanTransport
{
public:
  CanTransport(
    std::vector<MotorConfig> configs, TransportOptions options,
    rclcpp::Node::SharedPtr node);
  ~CanTransport();
  CanTransport(const CanTransport &) = delete;
  CanTransport & operator=(const CanTransport &) = delete;
  void start();
  void shutdown() noexcept;
  bool wait_ready(std::chrono::milliseconds timeout);
  bool enable();
  void disable();
  void request_stop() noexcept {fault_requested_.store(true); stop_requested_.store(true);}
  bool enabled() const {return enabled_.load();}
  bool faulted() const {return faulted_.load();}
  bool try_set_commands(const std::vector<double> & commands);
  bool set_command(uint16_t logical_id, double command);
  bool try_snapshot(std::vector<MotorState> & states);
  std::vector<MotorState> snapshot();
  std::string fault_reason();

private:
  struct PendingCommand
  {
    double value{};
    TimePoint received{};
    bool updated{false};
  };
  void cycle();
  DriverCore core_;
  TransportOptions options_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable feedback_changed_;
  std::vector<PendingCommand> pending_;
  std::vector<can_msgs::msg::Frame> frames_;
  std::atomic<bool> stop_requested_{true};
  std::atomic<bool> enabled_{false};
  std::atomic<bool> faulted_{false};
  std::atomic<bool> fault_requested_{false};
};
}  // namespace vesc_can_ros2_control
