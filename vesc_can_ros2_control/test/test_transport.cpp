#include <chrono>
#include <thread>
#include "gtest/gtest.h"
#include "vesc_can_ros2_control/can_transport.hpp"
using namespace vesc_can_ros2_control;
using namespace std::chrono_literals;
class TransportTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};
TEST_F(TransportTest, LatestCommandsTimeoutRecoveryAndIndependentExecutor)
{
  MotorConfig c; c.name = "drive"; c.logical_id = 1; c.controller_id = 11;
  c.command_timeout = 100ms; c.feedback_timeout = 100ms;
  TransportOptions options; options.tx_topic = "/transport_test/tx"; options.rx_topic = "/transport_test/rx"; options.period = 5ms;
  auto mock = std::make_shared<rclcpp::Node>("transport_test_peer");
  auto io = std::make_shared<rclcpp::Node>("transport_test_io");
  auto feedback = mock->create_publisher<can_msgs::msg::Frame>(options.rx_topic, rclcpp::SensorDataQoS());
  std::vector<can_msgs::msg::Frame> received;
  auto subscription = mock->create_subscription<can_msgs::msg::Frame>(options.tx_topic, 100,
    [&](can_msgs::msg::Frame::ConstSharedPtr f) {received.push_back(*f);});
  bool publish_feedback = true;
  auto timer = mock->create_wall_timer(10ms, [&]() {
    if (!publish_feedback) {return;}
    can_msgs::msg::Frame f; f.id = 0x90B; f.is_extended = true; f.dlc = 8; feedback->publish(f);
  });
  rclcpp::executors::SingleThreadedExecutor peer_executor; peer_executor.add_node(mock);
  CanTransport transport({c}, options, io); transport.start();
  auto pump_until = [&](auto condition, std::chrono::milliseconds timeout) {
    const auto deadline = SteadyClock::now() + timeout;
    while (SteadyClock::now() < deadline) {
      peer_executor.spin_some(); if (condition()) {return true;} std::this_thread::sleep_for(2ms);
    }
    return condition();
  };
  ASSERT_TRUE(pump_until([&]() {return transport.snapshot()[0].have_status1;}, 2s));
  ASSERT_TRUE(transport.enable());
  // Multiple pending updates collapse to the latest value.
  ASSERT_TRUE(transport.set_command(1, 1.0)); ASSERT_TRUE(transport.set_command(1, 2.0));
  ASSERT_TRUE(pump_until([&]() {
    for (const auto & f : received) {if ((f.id >> 8) == 3 && (f.data[2] || f.data[3])) {return true;}}
    return false;
  }, 80ms));
  ASSERT_TRUE(pump_until([&]() {return transport.faulted();}, 250ms));
  EXPECT_FALSE(transport.enabled()); EXPECT_FALSE(transport.set_command(1, 1.0));
  ASSERT_FALSE(transport.fault_reason().empty());
  ASSERT_TRUE(transport.enable()); received.clear();
  ASSERT_TRUE(transport.try_set_commands({2.0}));
  publish_feedback = false;
  ASSERT_TRUE(pump_until([&]() {transport.try_set_commands({2.0}); return transport.faulted();}, 250ms));
  ASSERT_TRUE(pump_until([&]() {
    return !received.empty() && (received.back().id >> 8) == 1 && received.back().data[3] == 0;
  }, 100ms));
  EXPECT_FALSE(transport.enable());
  publish_feedback = true;
  ASSERT_TRUE(pump_until([&]() {return transport.snapshot()[0].status1_time > SteadyClock::now() - 30ms;}, 300ms));
  ASSERT_TRUE(transport.enable());
  transport.request_stop(); ASSERT_TRUE(pump_until([&]() {return transport.faulted();}, 100ms));
  transport.shutdown();
  auto heartbeat = 0;
  auto heartbeat_timer = mock->create_wall_timer(5ms, [&]() {++heartbeat;});
  EXPECT_TRUE(pump_until([&]() {return heartbeat >= 2;}, 100ms));
}
