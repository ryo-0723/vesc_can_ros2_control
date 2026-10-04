#include "vesc_can_ros2_control/can_transport.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace vesc_can_ros2_control
{
CanTransport::CanTransport(
  std::vector<MotorConfig> configs, TransportOptions options, rclcpp::Node::SharedPtr node)
: core_(std::move(configs)), options_(std::move(options)), node_(std::move(node)),
  pending_(core_.configs().size())
{
  if (!node_ || options_.period.count() <= 0 || options_.tx_topic.empty() ||
    options_.rx_topic.empty() || options_.tx_topic == options_.rx_topic)
  {
    throw std::invalid_argument("Invalid CAN transport node, topics or period");
  }
  frames_.reserve(core_.configs().size());
  publisher_ = node_->create_publisher<can_msgs::msg::Frame>(
    options_.tx_topic, rclcpp::QoS(std::max<std::size_t>(50, core_.configs().size() * 4)));
  subscription_ = node_->create_subscription<can_msgs::msg::Frame>(
    options_.rx_topic, rclcpp::SensorDataQoS().keep_last(512),
    [this](can_msgs::msg::Frame::ConstSharedPtr frame) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        core_.receive(*frame, SteadyClock::now());
      }
      feedback_changed_.notify_all();
    });
  timer_ = node_->create_wall_timer(options_.period, [this]() {cycle();});
  executor_.add_node(node_);
}

CanTransport::~CanTransport() {shutdown();}

void CanTransport::start()
{
  if (thread_.joinable()) {throw std::logic_error("Transport already started");}
  thread_ = std::thread([this]() {
      try {
        executor_.spin();
      } catch (const std::exception & error) {
        enabled_.store(false);
        faulted_.store(true);
        stop_requested_.store(true);
        RCLCPP_ERROR(node_->get_logger(), "CAN executor failed: %s", error.what());
      }
    });
}

void CanTransport::shutdown() noexcept
{
  stop_requested_.store(true);
  enabled_.store(false);
  executor_.cancel();
  if (thread_.joinable()) {thread_.join();}
  try {
    if (rclcpp::ok(node_->get_node_base_interface()->get_context())) {
      std::vector<can_msgs::msg::Frame> stop;
      stop.reserve(core_.configs().size());
      core_.stop_frames(stop);
      for (auto & frame : stop) {
        frame.header.stamp = node_->now();
        publisher_->publish(frame);
      }
    }
  } catch (const std::exception &) {
    // Destruction cannot guarantee network delivery; MCU/VESC watchdogs remain necessary.
  }
  feedback_changed_.notify_all();
}

bool CanTransport::wait_ready(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return feedback_changed_.wait_for(lock, timeout, [this]() {
      return core_.ready(SteadyClock::now());
    });
}

bool CanTransport::enable()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::fill(pending_.begin(), pending_.end(), PendingCommand{});
  const bool result = core_.enable(SteadyClock::now());
  fault_requested_.store(false);
  enabled_.store(result);
  faulted_.store(core_.faulted());
  stop_requested_.store(!result);
  return result;
}

void CanTransport::disable()
{
  stop_requested_.store(true);
  enabled_.store(false);
  std::lock_guard<std::mutex> lock(mutex_);
  core_.disable();
  std::fill(pending_.begin(), pending_.end(), PendingCommand{});
}

bool CanTransport::try_set_commands(const std::vector<double> & commands)
{
  if (commands.size() != pending_.size() || !enabled_.load() || stop_requested_.load()) {
    return false;
  }
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {return false;}
  const auto now = SteadyClock::now();
  for (std::size_t i = 0; i < commands.size(); ++i) {pending_[i] = {commands[i], now, true};}
  return true;
}

bool CanTransport::set_command(uint16_t logical_id, double command)
{
  if (!enabled_.load() || stop_requested_.load()) {return false;}
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & configs = core_.configs();
  for (std::size_t i = 0; i < configs.size(); ++i) {
    if (configs[i].logical_id == logical_id) {
      pending_[i] = {command, SteadyClock::now(), true};
      return true;
    }
  }
  return false;
}

bool CanTransport::try_snapshot(std::vector<MotorState> & states)
{
  if (states.size() != core_.configs().size()) {return false;}
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {return false;}
  std::copy(core_.states().begin(), core_.states().end(), states.begin());
  return true;
}

std::vector<MotorState> CanTransport::snapshot()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return core_.states();
}

std::string CanTransport::fault_reason()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return core_.fault_reason();
}

void CanTransport::cycle()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_requested_.load()) {
      if (fault_requested_.exchange(false)) {
        core_.stop_on_error("Stop requested by hardware/topic command validation");
      }
      core_.disable();
      std::fill(pending_.begin(), pending_.end(), PendingCommand{});
    } else {
      for (std::size_t i = 0; i < pending_.size(); ++i) {
        if (pending_[i].updated) {
          core_.set_command(i, pending_[i].value, pending_[i].received);
          pending_[i].updated = false;
        }
      }
    }
    core_.command_frames(SteadyClock::now(), frames_);
    enabled_.store(core_.enabled());
    faulted_.store(core_.faulted());
  }
  for (auto & frame : frames_) {
    // A concurrent disable must not leave a previously prepared motion command in flight.
    if (stop_requested_.load()) {
      frame = protocol::make_set_current_frame(static_cast<uint8_t>(frame.id & 0xFF), 0.0);
    }
    frame.header.stamp = node_->now();
    publisher_->publish(frame);
  }
}
}  // namespace vesc_can_ros2_control
