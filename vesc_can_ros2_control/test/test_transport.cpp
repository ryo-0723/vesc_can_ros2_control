#include <chrono>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "vesc_can_ros2_control/can_transport.hpp"
namespace vesc_can_ros2_control
{
namespace
{
using namespace std::chrono_literals;
class TransportTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};
TEST_F(TransportTest, LatestCommandsTimeoutRecoveryAndIndependentExecutor)
{
  MotorConfig motor_config;
  motor_config.name = "drive";
  motor_config.controller_id = 11;
  motor_config.command_timeout = 100ms;
  motor_config.feedback_timeout = 100ms;
  TransportOptions transport_options;
  transport_options.tx_topic = "/transport_test/tx";
  transport_options.rx_topic = "/transport_test/rx";
  transport_options.period = 5ms;
  auto peer_node = std::make_shared<rclcpp::Node>("transport_test_peer");
  auto can_node = std::make_shared<rclcpp::Node>("transport_test_io");
  auto feedback_publisher = peer_node->create_publisher<can_msgs::msg::Frame>(
    transport_options.rx_topic, rclcpp::SensorDataQoS());
  std::vector<can_msgs::msg::Frame> received_frames;
  auto command_subscription =
    peer_node->create_subscription<can_msgs::msg::Frame>(transport_options.tx_topic, 100,
      [&](can_msgs::msg::Frame::ConstSharedPtr frame) { received_frames.push_back(*frame); });
  bool publish_feedback = true;
  auto feedback_timer = peer_node->create_wall_timer(10ms, [&]() {
    if (!publish_feedback) {
      return;
    }
    can_msgs::msg::Frame frame;
    frame.id = 0x90B;
    frame.is_extended = true;
    frame.dlc = 8;
    feedback_publisher->publish(frame);
  });
  rclcpp::executors::SingleThreadedExecutor peer_executor;
  peer_executor.add_node(peer_node);
  CanTransport can_transport({motor_config}, transport_options, can_node);
  can_transport.start();
  auto spin_peer_until = [&](auto predicate, std::chrono::milliseconds timeout) {
    const auto deadline = SteadyClock::now() + timeout;
    while (SteadyClock::now() < deadline) {
      peer_executor.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  };
  ASSERT_TRUE(spin_peer_until([&]() { return can_transport.snapshot()[0].have_status1; }, 2s));
  ASSERT_TRUE(can_transport.enable());
  // Multiple pending updates collapse to the latest value.
  ASSERT_TRUE(can_transport.set_command(0, 1.0));
  ASSERT_TRUE(can_transport.set_command(0, 2.0));
  ASSERT_TRUE(spin_peer_until(
    [&]() {
      for (const auto &frame : received_frames) {
        if ((frame.id >> 8) == 3 && (frame.data[2] || frame.data[3])) {
          return true;
        }
      }
      return false;
    },
    80ms));
  ASSERT_TRUE(spin_peer_until([&]() { return can_transport.faulted(); }, 250ms));
  EXPECT_FALSE(can_transport.enabled());
  EXPECT_FALSE(can_transport.set_command(0, 1.0));
  ASSERT_FALSE(can_transport.fault_reason().empty());
  ASSERT_TRUE(can_transport.enable());
  received_frames.clear();
  ASSERT_TRUE(can_transport.try_set_commands({2.0}));
  publish_feedback = false;
  ASSERT_TRUE(spin_peer_until(
    [&]() {
      can_transport.try_set_commands({2.0});
      return can_transport.faulted();
    },
    250ms));
  ASSERT_TRUE(spin_peer_until(
    [&]() {
      return !received_frames.empty() && (received_frames.back().id >> 8) == 1 &&
             received_frames.back().data[3] == 0;
    },
    100ms));
  EXPECT_FALSE(can_transport.enable());
  publish_feedback = true;
  ASSERT_TRUE(spin_peer_until(
    [&]() { return can_transport.snapshot()[0].status1_time > SteadyClock::now() - 30ms; }, 300ms));
  ASSERT_TRUE(can_transport.enable());
  can_transport.request_stop();
  ASSERT_TRUE(spin_peer_until([&]() { return can_transport.faulted(); }, 100ms));
  can_transport.shutdown();
  can_transport.shutdown();
  EXPECT_FALSE(can_transport.enable());
  EXPECT_FALSE(can_transport.wait_ready(10ms));
  auto heartbeat_count = 0;
  auto heartbeat_timer = peer_node->create_wall_timer(5ms, [&]() { ++heartbeat_count; });
  EXPECT_TRUE(spin_peer_until([&]() { return heartbeat_count >= 2; }, 100ms));
}

}  // namespace
}  // namespace vesc_can_ros2_control
