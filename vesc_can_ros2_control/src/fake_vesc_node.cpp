#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>
#include "vesc_can_ros2_control/driver_core.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vesc_can_ros2_control
{
namespace
{
void put(std::array<uint8_t, 8> & data, std::size_t offset, int64_t value, int bytes)
{
  const auto bits = static_cast<uint32_t>(value);
  for (int i = 0; i < bytes; ++i) {data[offset + i] = (bits >> (8 * (bytes - i - 1))) & 0xFF;}
}
int32_t read32(const std::array<uint8_t, 8> & d)
{
  const uint32_t value = (uint32_t(d[0]) << 24) | (uint32_t(d[1]) << 16) | (uint32_t(d[2]) << 8) | d[3];
  return value <= INT32_MAX ? static_cast<int32_t>(value) : -1 - static_cast<int32_t>(UINT32_MAX - value);
}
}
// Protocol fixture only: not a physical motor, encoder, brake or VESC firmware simulation.
class FakeVescNode : public rclcpp::Node
{
public:
  FakeVescNode() : rclcpp::Node("fake_vesc")
  {
    const auto ids = declare_parameter<std::vector<int64_t>>("controller_ids", {11, 21});
    pole_pairs_ = declare_parameter<double>("pole_pairs", 7.0);
    declare_parameter<bool>("publish_feedback", true);
    const auto period = declare_parameter<int64_t>("period_ms", 20);
    watchdog_ = std::chrono::milliseconds(declare_parameter<int64_t>("watchdog_ms", 200));
    if (!std::isfinite(pole_pairs_) || pole_pairs_ <= 0.0 || period <= 0 || watchdog_.count() <= 0) {
      throw std::invalid_argument("Invalid fake VESC settings");
    }
    for (auto id : ids) {
      if (id < 0 || id > 254) {throw std::invalid_argument("Invalid fake controller ID");}
      for (const auto & m : motors_) {if (m.id == id) {throw std::invalid_argument("Duplicate ID");}}
      motors_.push_back({static_cast<uint8_t>(id), 0.0, 0.0, 0.0, SteadyClock::now()});
    }
    publisher_ = create_publisher<can_msgs::msg::Frame>(declare_parameter<std::string>("can_rx_topic", "/can/rx"), rclcpp::SensorDataQoS());
    subscription_ = create_subscription<can_msgs::msg::Frame>(declare_parameter<std::string>("can_tx_topic", "/can/tx"), 100,
      [this](can_msgs::msg::Frame::ConstSharedPtr frame) {
        if (!frame->is_extended || frame->is_rtr || frame->is_error || frame->dlc != 4 || frame->id > 0x1FFFFFFF) {return;}
        for (auto & motor : motors_) {
          if (motor.id != (frame->id & 0xFF)) {continue;}
          const auto command = frame->id >> 8;
          const auto value = read32(frame->data);
          if (command == protocol::SET_RPM_ID) {motor.erpm = value; motor.current = 0.0;}
          else if (command == protocol::SET_POSITION_ID) {
            motor.angle = value / 1e6 * PI / 180.0;
            motor.erpm = 0.0;
            motor.current = 0.0;
          } else if (command == protocol::SET_CURRENT_ID) {
            motor.current = value / 1000.0;
            motor.erpm = motor.current * 1000.0;
          } else {return;}
          motor.command_time = SteadyClock::now();
        }
      });
    last_update_ = SteadyClock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(period), [this]() {tick();});
    RCLCPP_WARN(get_logger(), "Fake VESC active; use isolated CAN topics. No physical dynamics are simulated.");
  }
private:
  struct Motor {uint8_t id; double erpm; double angle; double current; TimePoint command_time;};
  void tick()
  {
    const auto time = SteadyClock::now();
    const double dt = std::chrono::duration<double>(time - last_update_).count();
    last_update_ = time;
    for (auto & m : motors_) {
      if (time - m.command_time > watchdog_) {m.erpm = 0.0; m.current = 0.0;}
      m.angle += m.erpm * 2.0 * PI / (60.0 * pole_pairs_) * dt;
      if (!get_parameter("publish_feedback").as_bool()) {continue;}
      can_msgs::msg::Frame f;
      f.header.stamp = now(); f.is_extended = true; f.dlc = 8;
      f.id = (protocol::STATUS_1_ID << 8) | m.id;
      put(f.data, 0, std::llround(m.erpm), 4); put(f.data, 4, std::llround(m.current * 10.0), 2);
      put(f.data, 6, 0, 2); publisher_->publish(f);
      f.id = (protocol::STATUS_4_ID << 8) | m.id;
      double angle = std::fmod(m.angle * 180.0 / PI, 360.0); if (angle < 0.0) {angle += 360.0;}
      put(f.data, 0, 300, 2); put(f.data, 2, 310, 2); put(f.data, 4, 0, 2);
      put(f.data, 6, std::llround(angle * 50.0), 2); publisher_->publish(f);
      f.id = (protocol::STATUS_5_ID << 8) | m.id; f.dlc = 6;
      put(f.data, 0, std::llround(m.angle * 6.0 * pole_pairs_ / (2.0 * PI)), 4);
      put(f.data, 4, 240, 2); f.data[6] = 0; f.data[7] = 0; publisher_->publish(f);
    }
  }
  std::vector<Motor> motors_;
  double pole_pairs_{};
  TimePoint last_update_{};
  std::chrono::milliseconds watchdog_{};
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<vesc_can_ros2_control::FakeVescNode>());}
  catch (const std::exception & error) {RCLCPP_ERROR(rclcpp::get_logger("fake_vesc"), "%s", error.what()); result = 1;}
  rclcpp::shutdown();
  return result;
}
