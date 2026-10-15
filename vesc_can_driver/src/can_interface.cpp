#include "vesc_can_driver/can_interface.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace vesc_can_driver
{
CanInterface::CanInterface(
  rclcpp::Node::SharedPtr node, const CanOptions &options, std::size_t motor_count)
  : node_(std::move(node)), fault_reply_timeout_(options.fault_reply_timeout)
{
  if (!node_ || options.tx_topic.empty() || options.rx_topic.empty() ||
      fault_reply_timeout_.count() < 0) {
    throw std::invalid_argument("Invalid CAN node or topics");
  }
  const auto tx_topic = node_->get_node_topics_interface()->resolve_topic_name(options.tx_topic);
  const auto rx_topic = node_->get_node_topics_interface()->resolve_topic_name(options.rx_topic);
  if (tx_topic == rx_topic) {
    throw std::invalid_argument("CAN TX and RX topics must be different");
  }
  tx_pub_ = node_->create_publisher<can_msgs::msg::Frame>(options.tx_topic,
    rclcpp::QoS(std::max<std::size_t>(50, motor_count * 4)).reliable().durability_volatile());
  rx_sub_ = node_->create_subscription<can_msgs::msg::Frame>(options.rx_topic,
    rclcpp::SensorDataQoS().keep_last(512), [this](can_msgs::msg::Frame::ConstSharedPtr frame) {
      std::lock_guard<std::mutex> callback_lock(callback_mutex_);
      if (frame->is_error) {
        if (on_error_) {
          on_error_("CAN error frame received: id=" + std::to_string(frame->id));
        }
      } else if (on_feedback_) {
        if (const auto feedback = protocol::decode_feedback(*frame)) {
          if (const auto *fault = std::get_if<FaultCode>(&*feedback)) {
            auto &requested_at = fault_requests_[fault->controller_id];
            const auto now = std::chrono::steady_clock::now();
            if (!requested_at || now < *requested_at ||
                now - *requested_at > fault_reply_timeout_) {
              return;
            }
            requested_at.reset();
          }
          on_feedback_(*feedback);
        }
      }
    });
}

CanInterface::~CanInterface()
{
  stop();
}

void CanInterface::configure_targets(std::span<const std::uint8_t> ids)
{
  std::lock_guard<std::mutex> callback_lock(callback_mutex_);
  targets_.fill(false);
  for (const auto id : ids) {
    if (id >= protocol::kReplyCanId) {
      throw std::invalid_argument("CAN ID 254 is reserved for fault replies; 255 is broadcast");
    }
    targets_[id] = true;
  }
}

void CanInterface::start(FeedbackCallback on_feedback, ErrorCallback on_error)
{
  std::lock_guard<std::mutex> callback_lock(callback_mutex_);
  on_feedback_ = std::move(on_feedback);
  on_error_ = std::move(on_error);
}

void CanInterface::stop() noexcept
{
  std::lock_guard<std::mutex> callback_lock(callback_mutex_);
  on_feedback_ = {};
  on_error_ = {};
  fault_requests_.fill(std::nullopt);
}

void CanInterface::send(const VescCommand &command)
{
  auto frame = protocol::encode_command(command);
  {
    std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    if (!targets_[command.controller_id]) {
      throw std::invalid_argument("Unregistered CAN target");
    }
    if (command.type == VescCommandType::FaultRequest) {
      fault_requests_[command.controller_id] = std::chrono::steady_clock::now();
    } else if (command.type == VescCommandType::Reboot) {
      fault_requests_[command.controller_id].reset();
    }
  }
  frame.header.stamp = node_->now();
  tx_pub_->publish(frame);
}
}  // namespace vesc_can_driver
