#include "vesc_can_ros2_control/can_transport.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace vesc_can_ros2_control
{
CanTransport::CanTransport(std::vector<MotorConfig> motor_configs,
  TransportOptions transport_options, rclcpp::Node::SharedPtr can_node)
  : driver_core_(std::move(motor_configs)), transport_options_(std::move(transport_options)),
    can_node_(std::move(can_node)), pending_commands_(driver_core_.motor_configs().size())
{
  if (!can_node_ || transport_options_.period.count() <= 0 || transport_options_.tx_topic.empty() ||
      transport_options_.rx_topic.empty() ||
      transport_options_.tx_topic == transport_options_.rx_topic) {
    throw std::invalid_argument("Invalid CAN transport node, topics or period");
  }
  command_frames_.reserve(driver_core_.motor_configs().size());
  const auto tx_queue_depth = std::max<std::size_t>(50, driver_core_.motor_configs().size() * 4);
  const auto tx_qos = rclcpp::QoS(tx_queue_depth).reliable().durability_volatile();
  can_publisher_ =
    can_node_->create_publisher<can_msgs::msg::Frame>(transport_options_.tx_topic, tx_qos);
  can_subscription_ =
    can_node_->create_subscription<can_msgs::msg::Frame>(transport_options_.rx_topic,
      rclcpp::SensorDataQoS().keep_last(512), [this](can_msgs::msg::Frame::ConstSharedPtr frame) {
        {
          std::lock_guard<std::mutex> state_lock(state_mutex_);
          driver_core_.receive(*frame, SteadyClock::now());
        }
        feedback_changed_.notify_all();
      });
  send_timer_ =
    can_node_->create_wall_timer(transport_options_.period, [this]() { send_commands(); });
  can_executor_.add_node(can_node_);
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
  if (executor_thread_.joinable()) {
    throw std::logic_error("Transport already started");
  }
  executor_thread_ = std::thread([this]() {
    try {
      can_executor_.spin();
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
  can_executor_.cancel();
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  try {
    if (rclcpp::ok(can_node_->get_node_base_interface()->get_context())) {
      std::vector<can_msgs::msg::Frame> stop_frames;
      stop_frames.reserve(driver_core_.motor_configs().size());
      driver_core_.stop_frames(stop_frames);
      for (auto &frame : stop_frames) {
        frame.header.stamp = can_node_->now();
        can_publisher_->publish(frame);
      }
    }
  } catch (const std::exception &) {
    // Destruction cannot guarantee network delivery; MCU/VESC watchdogs remain necessary.
  }
  feedback_changed_.notify_all();
}

bool CanTransport::wait_ready(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  const bool condition_met = feedback_changed_.wait_for(state_lock, timeout,
    [this]() { return shutdown_started_.load() || driver_core_.ready(SteadyClock::now()); });
  return condition_met && !shutdown_started_.load();
}

bool CanTransport::enable()
{
  if (shutdown_started_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  std::fill(pending_commands_.begin(), pending_commands_.end(), PendingCommand{});
  const bool enabled_successfully = driver_core_.enable(SteadyClock::now());
  fault_requested_.store(false);
  enabled_.store(enabled_successfully);
  faulted_.store(driver_core_.faulted());
  stop_requested_.store(!enabled_successfully);
  return enabled_successfully;
}

void CanTransport::disable()
{
  stop_requested_.store(true);
  enabled_.store(false);
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  driver_core_.disable();
  std::fill(pending_commands_.begin(), pending_commands_.end(), PendingCommand{});
}

bool CanTransport::try_set_commands(const std::vector<double> &commands)
{
  if (commands.size() != pending_commands_.size() || !enabled_.load() || stop_requested_.load()) {
    return false;
  }
  std::unique_lock<std::mutex> state_lock(state_mutex_, std::try_to_lock);
  if (!state_lock.owns_lock()) {
    return false;
  }
  const auto received_at = SteadyClock::now();
  for (std::size_t motor_index = 0; motor_index < commands.size(); ++motor_index) {
    pending_commands_[motor_index] = {commands[motor_index], received_at, true};
  }
  return true;
}

bool CanTransport::set_command(std::size_t motor_index, double command)
{
  if (motor_index >= pending_commands_.size() || !enabled_.load() || stop_requested_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  pending_commands_[motor_index] = {command, SteadyClock::now(), true};
  return true;
}

bool CanTransport::try_snapshot(std::vector<MotorState> &states)
{
  if (states.size() != driver_core_.motor_configs().size()) {
    return false;
  }
  std::unique_lock<std::mutex> state_lock(state_mutex_, std::try_to_lock);
  if (!state_lock.owns_lock()) {
    return false;
  }
  std::copy(driver_core_.states().begin(), driver_core_.states().end(), states.begin());
  return true;
}

std::vector<MotorState> CanTransport::snapshot()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return driver_core_.states();
}

std::string CanTransport::fault_reason()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return driver_core_.fault_reason();
}

void CanTransport::send_commands()
{
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (stop_requested_.load()) {
      if (fault_requested_.exchange(false)) {
        driver_core_.stop_on_error("Stop requested by hardware/topic command validation");
      }
      driver_core_.disable();
      std::fill(pending_commands_.begin(), pending_commands_.end(), PendingCommand{});
    } else {
      for (std::size_t motor_index = 0; motor_index < pending_commands_.size(); ++motor_index) {
        if (pending_commands_[motor_index].updated) {
          driver_core_.set_command(motor_index, pending_commands_[motor_index].value,
            pending_commands_[motor_index].received_at);
          pending_commands_[motor_index].updated = false;
        }
      }
    }
    driver_core_.command_frames(SteadyClock::now(), command_frames_);
    enabled_.store(driver_core_.enabled());
    faulted_.store(driver_core_.faulted());
  }
  for (auto &frame : command_frames_) {
    // A concurrent disable must not leave a previously prepared motion command in flight.
    if (stop_requested_.load()) {
      frame = protocol::make_set_current_frame(
        static_cast<std::uint8_t>(frame.id & protocol::kControllerIdMask), 0.0);
    }
    frame.header.stamp = can_node_->now();
    can_publisher_->publish(frame);
  }
}
}  // namespace vesc_can_ros2_control
