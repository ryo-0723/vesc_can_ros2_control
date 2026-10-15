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
#include "vesc_can_driver/can_protocol.hpp"
#include "vesc_can_driver/driver_core.hpp"

namespace vesc_can_driver
{
namespace
{
/// @brief 模擬 STATUS の指定位置へ値を big endian で書き込む
void write_big_endian(std::array<std::uint8_t, 8> &payload, std::size_t offset, std::int64_t value,
  std::size_t byte_count)
{
  const auto payload_bits = static_cast<std::uint32_t>(value);
  for (std::size_t byte_idx = 0; byte_idx < byte_count; ++byte_idx) {
    const auto bit_shift = 8 * (byte_count - byte_idx - 1);
    payload[offset + byte_idx] = static_cast<std::uint8_t>(payload_bits >> bit_shift);
  }
}

/// @brief 指令の先頭 4 バイトを符号付き int32 として読む
std::int32_t read_command_payload(const std::array<std::uint8_t, 8> &payload)
{
  std::uint32_t payload_bits = 0;
  for (std::size_t byte_idx = 0; byte_idx < 4; ++byte_idx) {
    payload_bits = (payload_bits << 8) | payload[byte_idx];
  }
  return std::bit_cast<std::int32_t>(payload_bits);
}
}  // namespace

// Protocol fixture only: it does not model physical motors, brakes or VESC firmware.
class FakeVescNode : public rclcpp::Node
{
public:
  /// @brief 模擬モータ・CAN 指令受付・STATUS 配信タイマーを作成する
  FakeVescNode() : rclcpp::Node("fake_vesc")
  {
    const auto controller_ids =
      declare_parameter<std::vector<std::int64_t>>("controller_ids", {11, 21});
    pole_pairs_ = declare_parameter<double>("pole_pairs", 7.0);
    declare_parameter<bool>("publish_feedback", true);
    declare_parameter<std::int64_t>("fault_controller_id", -1);
    declare_parameter<std::int64_t>("fault_code", 0);
    const auto feedback_period =
      std::chrono::milliseconds(declare_parameter<std::int64_t>("period_ms", 20));
    watchdog_timeout_ =
      std::chrono::milliseconds(declare_parameter<std::int64_t>("watchdog_ms", 200));
    if (!std::isfinite(pole_pairs_) || pole_pairs_ <= 0.0 || feedback_period.count() <= 0 ||
        watchdog_timeout_.count() <= 0) {
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
      motor.cmd_at = SteadyClock::now();
      motors_.push_back(motor);
    }
    feedback_pub_ = create_publisher<can_msgs::msg::Frame>(
      declare_parameter<std::string>("can_rx_topic", "can/rx"), rclcpp::SensorDataQoS());
    cmd_sub_ = create_subscription<can_msgs::msg::Frame>(
      declare_parameter<std::string>("can_tx_topic", "can/tx"), 100,
      [this](can_msgs::msg::Frame::ConstSharedPtr frame) { handle_can_command(*frame); });
    last_update_at_ = SteadyClock::now();
    feedback_timer_ = create_wall_timer(feedback_period, [this]() { update_motors(); });
    RCLCPP_WARN(get_logger(),
      "Fake VESC active; use isolated CAN topics. No physical dynamics are simulated.");
  }

private:
  struct FakeMotor
  {
    std::uint8_t controller_id{};
    double electrical_rpm{};
    double pos_rad{};
    double motor_current_a{};
    double duty_cycle{};
    TimePoint cmd_at{};
    TimePoint boot_until{};
  };

  /// @brief 登録 ID の電流・速度・位置指令を模擬状態へ反映する
  void handle_can_command(const can_msgs::msg::Frame &frame)
  {
    if (!frame.is_extended || frame.is_rtr || frame.is_error ||
        frame.id > protocol::kExtendedCanIdMask) {
      return;
    }
    const auto controller_id = frame.id & protocol::kControllerIdMask;
    const auto motor_it =
      std::find_if(motors_.begin(), motors_.end(), [controller_id](const FakeMotor &candidate) {
        return candidate.controller_id == controller_id;
      });
    if (motor_it == motors_.end()) {
      return;
    }
    const auto packet_id = frame.id >> protocol::kPacketIdShift;
    if (packet_id == protocol::kProcessShortBufferId && frame.dlc == 3 && frame.data[1] == 2 &&
        frame.data[2] == protocol::kCommRebootId) {
      const auto id = motor_it->controller_id;
      *motor_it = FakeMotor{};
      motor_it->controller_id = id;
      motor_it->cmd_at = SteadyClock::now();
      motor_it->boot_until = motor_it->cmd_at + std::chrono::milliseconds(200);
      if (get_parameter("fault_controller_id").as_int() == id) {
        set_parameter(rclcpp::Parameter("fault_code", std::int64_t{0}));
      }
      return;
    }
    if (SteadyClock::now() < motor_it->boot_until) {
      return;
    }
    if (packet_id == protocol::kProcessShortBufferId && frame.dlc == 7 && frame.data[1] == 0 &&
        frame.data[2] == protocol::kGetValuesSelectiveId && frame.data[3] == 0 &&
        frame.data[4] == 0 && frame.data[5] == 0x80 && frame.data[6] == 0) {
      if (!get_parameter("publish_feedback").as_bool()) {
        return;
      }
      const auto fault_id = get_parameter("fault_controller_id").as_int();
      const auto code = get_parameter("fault_code").as_int();
      if (code < 0 || code > 255) {
        return;
      }
      can_msgs::msg::Frame reply;
      reply.header.stamp = now();
      reply.id = (protocol::kProcessShortBufferId << 8) | frame.data[0];
      reply.is_extended = true;
      reply.dlc = 8;
      reply.data = {motor_it->controller_id, 1, protocol::kGetValuesSelectiveId, 0, 0, 0x80, 0,
        static_cast<std::uint8_t>(fault_id == motor_it->controller_id ? code : 0)};
      feedback_pub_->publish(reply);
      return;  // Reading fault_code does not refresh the command watchdog.
    }
    if (frame.dlc != 4) {
      return;
    }
    const auto value = read_command_payload(frame.data);
    switch (packet_id) {
      case protocol::kSetDutyId:
        motor_it->duty_cycle = value / 100000.0;
        motor_it->electrical_rpm = motor_it->duty_cycle * 1000.0;
        motor_it->motor_current_a = 0.0;
        break;
      case protocol::kSetRpmId:
        motor_it->duty_cycle = 0.0;
        motor_it->electrical_rpm = value;
        motor_it->motor_current_a = 0.0;
        break;
      case protocol::kSetPositionId:
        motor_it->duty_cycle = 0.0;
        motor_it->pos_rad = value / 1e6 * kPi / 180.0;
        motor_it->electrical_rpm = 0.0;
        motor_it->motor_current_a = 0.0;
        break;
      case protocol::kSetCurrentId:
        motor_it->duty_cycle = 0.0;
        motor_it->motor_current_a = value / 1000.0;
        motor_it->electrical_rpm = motor_it->motor_current_a * 1000.0;
        break;
      default:
        return;
    }
    motor_it->cmd_at = SteadyClock::now();
  }

  /// @brief ID・ROS 時刻・DLC=8 を設定した模擬 STATUS フレームを作る
  can_msgs::msg::Frame make_status_frame(const FakeMotor &motor, std::uint32_t packet_id)
  {
    can_msgs::msg::Frame frame;
    frame.header.stamp = now();
    frame.id = (packet_id << protocol::kPacketIdShift) | motor.controller_id;
    frame.is_extended = true;
    frame.dlc = 8;
    return frame;
  }

  /// @brief 1 台の模擬状態と固定の診断値を STATUS1〜6 で配信する
  void publish_motor_feedback(const FakeMotor &motor)
  {
    auto status_1 = make_status_frame(motor, protocol::kStatus1Id);
    write_big_endian(status_1.data, 0, std::llround(motor.electrical_rpm), 4);
    write_big_endian(status_1.data, 4, std::llround(motor.motor_current_a * 10.0), 2);
    write_big_endian(status_1.data, 6, std::llround(motor.duty_cycle * 1000.0), 2);
    feedback_pub_->publish(status_1);

    auto status_2 = make_status_frame(motor, protocol::kStatus2Id);
    write_big_endian(status_2.data, 0, 12500, 4);  // Fixture: 1.25 Ah drawn.
    write_big_endian(status_2.data, 4, 5000, 4);   // Fixture: 0.5 Ah regenerated.
    feedback_pub_->publish(status_2);
    auto status_3 = make_status_frame(motor, protocol::kStatus3Id);
    write_big_endian(status_3.data, 0, 120000, 4);  // Fixture: 12 Wh drawn.
    write_big_endian(status_3.data, 4, 10000, 4);   // Fixture: 1 Wh regenerated.
    feedback_pub_->publish(status_3);

    double pos_deg = std::fmod(motor.pos_rad * 180.0 / kPi, 360.0);
    if (pos_deg < 0.0) {
      pos_deg += 360.0;
    }
    auto status_4 = make_status_frame(motor, protocol::kStatus4Id);
    write_big_endian(status_4.data, 0, 300, 2);  // FET temperature: 30 C.
    write_big_endian(status_4.data, 2, 310, 2);  // Motor temperature: 31 C.
    write_big_endian(status_4.data, 6, std::llround(pos_deg * 50.0), 2);
    feedback_pub_->publish(status_4);

    auto status_5 = make_status_frame(motor, protocol::kStatus5Id);
    const auto tachometer_counts = std::llround(motor.pos_rad * 6.0 * pole_pairs_ / (2.0 * kPi));
    write_big_endian(status_5.data, 0, tachometer_counts, 4);
    write_big_endian(status_5.data, 4, 240, 2);  // Input voltage: 24 V.
    feedback_pub_->publish(status_5);

    auto status_6 = make_status_frame(motor, protocol::kStatus6Id);
    write_big_endian(status_6.data, 0, 1100, 2);  // Fixture: 1.1 V.
    write_big_endian(status_6.data, 2, 2200, 2);  // Fixture: 2.2 V.
    write_big_endian(status_6.data, 4, 3300, 2);  // Fixture: 3.3 V.
    write_big_endian(status_6.data, 6, 500, 2);   // Fixture: normalized PPM 0.5.
    feedback_pub_->publish(status_6);
  }

  /// @brief 模擬位置を更新し、指令切れの停止とフィードバック配信を行う
  void update_motors()
  {
    const auto steady_now = SteadyClock::now();
    const double dt_s = std::chrono::duration<double>(steady_now - last_update_at_).count();
    last_update_at_ = steady_now;
    const bool publish_feedback = get_parameter("publish_feedback").as_bool();
    for (auto &motor : motors_) {
      if (steady_now < motor.boot_until) {
        continue;
      }
      if (steady_now - motor.cmd_at > watchdog_timeout_) {
        motor.electrical_rpm = 0.0;
        motor.motor_current_a = 0.0;
        motor.duty_cycle = 0.0;
      }
      motor.pos_rad += motor.electrical_rpm * 2.0 * kPi / (60.0 * pole_pairs_) * dt_s;
      if (publish_feedback) {
        publish_motor_feedback(motor);
      }
    }
  }

  std::vector<FakeMotor> motors_;
  double pole_pairs_{};
  TimePoint last_update_at_{};
  std::chrono::milliseconds watchdog_timeout_{};
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr feedback_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
};
}  // namespace vesc_can_driver

/// @brief 模擬 VESC ノードを起動し、終了時に ROS を停止する
/// @return 正常終了で 0、ノード実行中の例外で 1
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<vesc_can_driver::FakeVescNode>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("fake_vesc"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
