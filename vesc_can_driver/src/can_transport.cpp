#include "vesc_can_driver/can_transport.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace vesc_can_driver
{
CanTransport::CanTransport(
  std::vector<MotorConfig> configs, TransportOptions opts, rclcpp::Node::SharedPtr can_node)
  : core_(std::move(configs)), opts_(std::move(opts)), can_node_(std::move(can_node)),
    pending_cmds_(core_.motor_configs().size())
{
  if (!can_node_ || opts_.send_period.count() <= 0 || opts_.tx_topic.empty() ||
      opts_.rx_topic.empty() || opts_.tx_topic == opts_.rx_topic) {
    throw std::invalid_argument("Invalid CAN transport node, topics or period");
  }

  cmd_frames_.reserve(core_.motor_configs().size());
  const auto tx_depth = std::max<std::size_t>(50, core_.motor_configs().size() * 4);
  const auto tx_qos = rclcpp::QoS(tx_depth).reliable().durability_volatile();
  tx_pub_ = can_node_->create_publisher<can_msgs::msg::Frame>(opts_.tx_topic, tx_qos);
  rx_sub_ = can_node_->create_subscription<can_msgs::msg::Frame>(opts_.rx_topic,
    rclcpp::SensorDataQoS().keep_last(512), [this](can_msgs::msg::Frame::ConstSharedPtr frame) {
      {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        core_.receive(*frame, SteadyClock::now());
      }
      feedback_cv_.notify_all();
    });
  send_timer_ = can_node_->create_wall_timer(opts_.send_period, [this]() { send_commands(); });
  can_exec_.add_node(can_node_);
}

CanTransport::~CanTransport()
{
  shutdown();
}

void CanTransport::start()
{
  if (shutdown_started_.load()) {
    throw std::logic_error("Transport has been shut down");
  }
  if (exec_thread_.joinable()) {
    throw std::logic_error("Transport already started");
  }
  exec_thread_ = std::thread([this]() {
    try {
      can_exec_.spin();
    } catch (const std::exception &error) {
      enabled_.store(false);
      faulted_.store(true);
      stop_requested_.store(true);
      RCLCPP_ERROR(can_node_->get_logger(), "CAN executor failed: %s", error.what());
    }
  });
}

void CanTransport::shutdown() noexcept
{
  if (shutdown_started_.exchange(true)) {
    return;
  }
  stop_requested_.store(true);
  enabled_.store(false);
  can_exec_.cancel();
  if (exec_thread_.joinable()) {
    exec_thread_.join();
  }
  try {
    if (rclcpp::ok(can_node_->get_node_base_interface()->get_context())) {
      core_.stop_frames(cmd_frames_);
      publish_frames();
    }
  } catch (const std::exception &) {
    // Destruction cannot guarantee network delivery; MCU/VESC watchdogs remain necessary.
  }
  feedback_cv_.notify_all();
}

bool CanTransport::wait_ready(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  const bool condition_met = feedback_cv_.wait_for(state_lock, timeout,
    [this]() { return shutdown_started_.load() || core_.ready(SteadyClock::now()); });
  return condition_met && !shutdown_started_.load();
}

bool CanTransport::enable()
{
  if (shutdown_started_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
  const bool enabled = core_.enable(SteadyClock::now());
  fault_requested_.store(false);
  enabled_.store(enabled);
  faulted_.store(core_.faulted());
  stop_requested_.store(!enabled);
  return enabled;
}

void CanTransport::disable()
{
  stop_requested_.store(true);
  enabled_.store(false);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  core_.disable();
  std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
}

bool CanTransport::try_set_commands(const std::vector<double> &values)
{
  if (values.size() != pending_cmds_.size() || !enabled_.load() || stop_requested_.load()) {
    return false;
  }
  for (std::size_t motor_idx = 0; motor_idx < values.size(); ++motor_idx) {
    if (!command_is_valid(core_.motor_configs()[motor_idx], values[motor_idx])) {
      request_stop();
      return false;
    }
  }
  std::unique_lock<std::mutex> state_lock(state_mutex_, std::try_to_lock);
  if (!state_lock.owns_lock()) {
    return false;
  }
  const auto received_at = SteadyClock::now();
  for (std::size_t motor_idx = 0; motor_idx < values.size(); ++motor_idx) {
    pending_cmds_[motor_idx] = {values[motor_idx], received_at};
  }
  return true;
}

bool CanTransport::set_command(std::size_t motor_idx, double value)
{
  if (motor_idx >= pending_cmds_.size() || !enabled_.load() || stop_requested_.load()) {
    return false;
  }
  if (!command_is_valid(core_.motor_configs()[motor_idx], value)) {
    request_stop();
    return false;
  }
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  pending_cmds_[motor_idx] = {value, SteadyClock::now()};
  return true;
}

bool CanTransport::try_snapshot(std::vector<MotorState> &states)
{
  if (states.size() != core_.motor_configs().size()) {
    return false;
  }
  std::unique_lock<std::mutex> state_lock(state_mutex_, std::try_to_lock);
  if (!state_lock.owns_lock()) {
    return false;
  }
  std::copy(core_.states().begin(), core_.states().end(), states.begin());
  return true;
}

std::vector<MotorState> CanTransport::snapshot()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return core_.states();
}

std::string CanTransport::fault_reason()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return core_.fault_reason();
}

void CanTransport::send_commands()
{
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (stop_requested_.load()) {
      if (fault_requested_.exchange(false)) {
        core_.stop_on_error("Stop requested by hardware/topic command validation");
        std::fill(pending_cmds_.begin(), pending_cmds_.end(), PendingCommand{});
      }
    } else {
      for (std::size_t motor_idx = 0; motor_idx < pending_cmds_.size(); ++motor_idx) {
        auto &cmd = pending_cmds_[motor_idx];
        if (cmd.received_at) {
          core_.set_command(motor_idx, cmd.value, *cmd.received_at);
          cmd.received_at.reset();
        }
      }
    }
    core_.command_frames(SteadyClock::now(), cmd_frames_);
    enabled_.store(core_.enabled());
    faulted_.store(core_.faulted());
  }
  publish_frames();
}

void CanTransport::publish_frames()
{
  for (auto &frame : cmd_frames_) {
    // A concurrent disable must not leave a previously prepared motion command in flight.
    if (stop_requested_.load()) {
      frame = protocol::make_set_current_frame(
        static_cast<std::uint8_t>(frame.id & protocol::kControllerIdMask), 0.0);
    }
    frame.header.stamp = can_node_->now();
    tx_pub_->publish(frame);
  }
}
}  // namespace vesc_can_driver
