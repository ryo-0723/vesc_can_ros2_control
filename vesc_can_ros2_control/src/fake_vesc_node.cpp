#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vesc_can_ros2_control/driver_core.hpp"

namespace vesc_can_ros2_control
{
namespace
{
void write_big_endian(std::array<std::uint8_t, 8> &payload, std::size_t offset, std::int64_t value,
  std::size_t byte_count)
{
  const auto payload_bits = static_cast<std::uint32_t>(value);
  for (std::size_t byte_index = 0; byte_index < byte_count; ++byte_index) {
    const auto bit_shift = 8 * (byte_count - byte_index - 1);
    payload[offset + byte_index] = static_cast<std::uint8_t>(payload_bits >> bit_shift);
  }
}

std::int32_t read_command_payload(const std::array<std::uint8_t, 8> &payload)
{
  std::uint32_t payload_bits = 0;
  for (std::size_t byte_index = 0; byte_index < 4; ++byte_index) {
    payload_bits = (payload_bits << 8) | payload[byte_index];
  }
  return std::bit_cast<std::int32_t>(payload_bits);
}
}  // namespace

// Protocol fixture only: it does not model physical motors, brakes or VESC firmware.
class FakeVescNode : public rclcpp::Node
{
public:
  FakeVescNode() : rclcpp::Node("fake_vesc")
  {
    const auto controller_ids =
      declare_parameter<std::vector<std::int64_t>>("controller_ids", {11, 21});
    pole_pairs_ = declare_parameter<double>("pole_pairs", 7.0);
    declare_parameter<bool>("publish_feedback", true);
    const auto feedback_period =
      std::chrono::milliseconds(declare_parameter<std::int64_t>("period_ms", 20));
    command_watchdog_timeout_ =
      std::chrono::milliseconds(declare_parameter<std::int64_t>("watchdog_ms", 200));
    if (!std::isfinite(pole_pairs_) || pole_pairs_ <= 0.0 || feedback_period.count() <= 0 ||
        command_watchdog_timeout_.count() <= 0) {
      throw std::invalid_argument("Invalid fake VESC settings");
    }
    for (const auto controller_id : controller_ids) {
      if (controller_id < 0 || controller_id >= protocol::kBroadcastControllerId) {
        throw std::invalid_argument("Invalid fake controller ID");
      }
      const auto duplicate_motor = std::find_if(motors_.begin(), motors_.end(),
        [controller_id](const FakeMotor &motor) { return motor.controller_id == controller_id; });
      if (duplicate_motor != motors_.end()) {
        throw std::invalid_argument("Duplicate ID");
      }
      FakeMotor motor;
      motor.controller_id = static_cast<std::uint8_t>(controller_id);
      motor.last_command_time = SteadyClock::now();
      motors_.push_back(motor);
    }
    feedback_publisher_ = create_publisher<can_msgs::msg::Frame>(
      declare_parameter<std::string>("can_rx_topic", "/can/rx"), rclcpp::SensorDataQoS());
    command_subscription_ = create_subscription<can_msgs::msg::Frame>(
      declare_parameter<std::string>("can_tx_topic", "/can/tx"), 100,
      [this](can_msgs::msg::Frame::ConstSharedPtr frame) { handle_can_command(*frame); });
    last_update_time_ = SteadyClock::now();
    feedback_timer_ = create_wall_timer(feedback_period, [this]() { update_motors(); });
    RCLCPP_WARN(get_logger(),
      "Fake VESC active; use isolated CAN topics. No physical dynamics are simulated.");
  }

private:
  struct FakeMotor
  {
    std::uint8_t controller_id{};
    double erpm{};
    double position_rad{};
    double current_a{};
    TimePoint last_command_time{};
  };

  void handle_can_command(const can_msgs::msg::Frame &frame)
  {
    if (!frame.is_extended || frame.is_rtr || frame.is_error || frame.dlc != 4 ||
        frame.id > protocol::kExtendedCanIdMask) {
      return;
    }
    const auto controller_id = frame.id & protocol::kControllerIdMask;
    const auto motor =
      std::find_if(motors_.begin(), motors_.end(), [controller_id](const FakeMotor &candidate) {
        return candidate.controller_id == controller_id;
      });
    if (motor == motors_.end()) {
      return;
    }
    const auto packet_id = frame.id >> protocol::kPacketIdShift;
    const auto command_value = read_command_payload(frame.data);
    switch (packet_id) {
      case protocol::kSetRpmId:
        motor->erpm = command_value;
        motor->current_a = 0.0;
        break;
      case protocol::kSetPositionId:
        motor->position_rad = command_value / 1e6 * kPi / 180.0;
        motor->erpm = 0.0;
        motor->current_a = 0.0;
        break;
      case protocol::kSetCurrentId:
        motor->current_a = command_value / 1000.0;
        motor->erpm = motor->current_a * 1000.0;
        break;
      default:
        return;
    }
    motor->last_command_time = SteadyClock::now();
  }

  can_msgs::msg::Frame make_status_frame(
    const FakeMotor &motor, std::uint32_t packet_id, std::uint8_t dlc)
  {
    can_msgs::msg::Frame frame;
    frame.header.stamp = now();
    frame.id = (packet_id << protocol::kPacketIdShift) | motor.controller_id;
    frame.is_extended = true;
    frame.dlc = dlc;
    return frame;
  }

  void publish_motor_feedback(const FakeMotor &motor)
  {
    auto status_1 = make_status_frame(motor, protocol::kStatus1Id, 8);
    write_big_endian(status_1.data, 0, std::llround(motor.erpm), 4);
    write_big_endian(status_1.data, 4, std::llround(motor.current_a * 10.0), 2);
    feedback_publisher_->publish(status_1);

    auto status_2 = make_status_frame(motor, protocol::kStatus2Id, 8);
    write_big_endian(status_2.data, 0, 12500, 4);  // Fixture: 1.25 Ah drawn.
    write_big_endian(status_2.data, 4, 5000, 4);   // Fixture: 0.5 Ah regenerated.
    feedback_publisher_->publish(status_2);
    auto status_3 = make_status_frame(motor, protocol::kStatus3Id, 8);
    write_big_endian(status_3.data, 0, 120000, 4);  // Fixture: 12 Wh drawn.
    write_big_endian(status_3.data, 4, 10000, 4);   // Fixture: 1 Wh regenerated.
    feedback_publisher_->publish(status_3);

    double position_deg = std::fmod(motor.position_rad * 180.0 / kPi, 360.0);
    if (position_deg < 0.0) {
      position_deg += 360.0;
    }
    auto status_4 = make_status_frame(motor, protocol::kStatus4Id, 8);
    write_big_endian(status_4.data, 0, 300, 2);  // FET temperature: 30 C.
    write_big_endian(status_4.data, 2, 310, 2);  // Motor temperature: 31 C.
    write_big_endian(status_4.data, 6, std::llround(position_deg * 50.0), 2);
    feedback_publisher_->publish(status_4);

    auto status_5 = make_status_frame(motor, protocol::kStatus5Id, 8);
    const auto tachometer_counts =
      std::llround(motor.position_rad * 6.0 * pole_pairs_ / (2.0 * kPi));
    write_big_endian(status_5.data, 0, tachometer_counts, 4);
    write_big_endian(status_5.data, 4, 240, 2);  // Input voltage: 24 V.
    feedback_publisher_->publish(status_5);

    auto status_6 = make_status_frame(motor, protocol::kStatus6Id, 8);
    write_big_endian(status_6.data, 0, 1100, 2);  // Fixture: 1.1 V.
    write_big_endian(status_6.data, 2, 2200, 2);  // Fixture: 2.2 V.
    write_big_endian(status_6.data, 4, 3300, 2);  // Fixture: 3.3 V.
    write_big_endian(status_6.data, 6, 500, 2);   // Fixture: normalized PPM 0.5.
    feedback_publisher_->publish(status_6);
  }

  void update_motors()
  {
    const auto current_time = SteadyClock::now();
    const double elapsed_seconds =
      std::chrono::duration<double>(current_time - last_update_time_).count();
    last_update_time_ = current_time;
    const bool publish_feedback = get_parameter("publish_feedback").as_bool();
    for (auto &motor : motors_) {
      if (current_time - motor.last_command_time > command_watchdog_timeout_) {
        motor.erpm = 0.0;
        motor.current_a = 0.0;
      }
      motor.position_rad += motor.erpm * 2.0 * kPi / (60.0 * pole_pairs_) * elapsed_seconds;
      if (publish_feedback) {
        publish_motor_feedback(motor);
      }
    }
  }

  std::vector<FakeMotor> motors_;
  double pole_pairs_{};
  TimePoint last_update_time_{};
  std::chrono::milliseconds command_watchdog_timeout_{};
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr feedback_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};
}  // namespace vesc_can_ros2_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<vesc_can_ros2_control::FakeVescNode>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("fake_vesc"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
