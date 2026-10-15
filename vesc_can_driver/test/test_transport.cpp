#include <algorithm>
#include <array>
#include <chrono>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "vesc_can_driver/vesc_transport.hpp"
namespace vesc_can_driver
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

TEST_F(TransportTest, ImmediateShutdownNeverWaitsForTheFirstTimer)
{
  MotorConfig cfg;
  cfg.name = "drive";
  cfg.motor_id = 11;
  TransportOptions opts;
  opts.send_period = 5s;
  opts.fault_period = 0ms;
  for (int i = 0; i < 32; ++i) {
    auto node = std::make_shared<rclcpp::Node>("immediate_shutdown_" + std::to_string(i));
    VescTransport transport({cfg}, opts, node);
    transport.start();
    transport.shutdown();
    EXPECT_FALSE(transport.active());
    EXPECT_FALSE(transport.wait_ready(1ms));
  }
}

TEST_F(TransportTest, PendingStopRejectsFreshCommandsBeforeTheExecutorStarts)
{
  MotorConfig cfg;
  cfg.name = "drive";
  cfg.motor_id = 11;
  TransportOptions opts;
  opts.auto_activate = true;
  opts.fault_period = 0ms;
  auto node = std::make_shared<rclcpp::Node>("pending_stop_io");
  VescTransport transport({cfg}, opts, node);
  ASSERT_TRUE(transport.set_commands({{0, 1.0}}));
  transport.request_stop();
  EXPECT_FALSE(transport.active());
  EXPECT_FALSE(transport.set_commands({{0, 2.0}}));
  transport.start();
  const auto deadline = SteadyClock::now() + 1s;
  while (!transport.faulted() && SteadyClock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(transport.faulted());
  EXPECT_FALSE(transport.active());
}

TEST_F(TransportTest, RejectsInvalidCanConfigurationBeforeStarting)
{
  MotorConfig cfg;
  cfg.name = "drive";
  cfg.motor_id = 11;
  auto node = std::make_shared<rclcpp::Node>("config_test_io", "/config_test");
  TransportOptions opts;
  opts.tx_topic = "";
  EXPECT_THROW(VescTransport({cfg}, opts, node), std::invalid_argument);
  opts.tx_topic = "can/rx";
  opts.rx_topic = "/config_test/can/rx";
  EXPECT_THROW(VescTransport({cfg}, opts, node), std::invalid_argument);
  opts.tx_topic = "can/tx";
  opts.send_period = 0ms;
  EXPECT_THROW(VescTransport({cfg}, opts, node), std::invalid_argument);
  opts.send_period = 10ms;
  opts.fault_period = -1ms;
  EXPECT_THROW(VescTransport({cfg}, opts, node), std::invalid_argument);
  for (const auto id : {254, 255}) {
    cfg.motor_id = id;
    EXPECT_THROW(MotorController{cfg}, std::invalid_argument);
  }
}

TEST_F(TransportTest, PendingBatchesUseLatestValuesAndConcurrentUpdatesStayTogether)
{
  MotorConfig cfg;
  cfg.name = "first";
  cfg.logical_id = 1;
  cfg.motor_id = 11;
  cfg.mode = ControlMode::Current;
  cfg.command_timeout = 2s;
  cfg.feedback_timeout = 2s;
  auto second = cfg;
  second.name = "second";
  second.logical_id = 2;
  second.motor_id = 12;
  TransportOptions opts;
  opts.tx_topic = "/batch_test/tx";
  opts.rx_topic = "/batch_test/rx";
  opts.fault_period = 0ms;
  opts.auto_activate = true;
  auto peer = std::make_shared<rclcpp::Node>("batch_test_peer");
  auto can_node = std::make_shared<rclcpp::Node>("batch_test_io");
  auto feedback_pub =
    peer->create_publisher<can_msgs::msg::Frame>(opts.rx_topic, rclcpp::SensorDataQoS());
  std::vector<std::pair<std::int32_t, std::int32_t>> pairs;
  std::int32_t first_value = 0;
  auto sub = peer->create_subscription<can_msgs::msg::Frame>(
    opts.tx_topic, 100, [&](can_msgs::msg::Frame::ConstSharedPtr frame) {
      if ((frame->id >> 8) != 1) {
        return;
      }
      const auto value = static_cast<std::int32_t>(
        (std::uint32_t(frame->data[0]) << 24) | (std::uint32_t(frame->data[1]) << 16) |
        (std::uint32_t(frame->data[2]) << 8) | frame->data[3]);
      if ((frame->id & 255) == 11) {
        first_value = value;
      } else if ((frame->id & 255) == 12) {
        pairs.emplace_back(first_value, value);
      }
    });
  auto timer = peer->create_wall_timer(10ms, [&]() {
    for (std::uint8_t id : {11, 12}) {
      can_msgs::msg::Frame frame;
      frame.id = 0x900 | id;
      frame.is_extended = true;
      frame.dlc = 8;
      feedback_pub->publish(frame);
    }
  });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(peer);
  VescTransport transport({cfg, second}, opts, can_node);
  const auto wait_for = [&](auto predicate, std::chrono::milliseconds timeout) {
    const auto deadline = SteadyClock::now() + timeout;
    while (SteadyClock::now() < deadline) {
      exec.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return predicate();
  };
  ASSERT_TRUE(wait_for([&]() { return feedback_pub->get_subscription_count() > 0; }, 2s));
  // No CAN executor is running yet, so none of these earlier values can be sent.
  ASSERT_TRUE(transport.set_commands({{0, 1.0}, {1, 2.0}}));
  ASSERT_TRUE(transport.set_commands({{0, 3.0}, {1, 4.0}}));
  ASSERT_TRUE(transport.set_commands({{1, 5.0}}));
  transport.start();
  ASSERT_TRUE(
    wait_for([&]() { return !pairs.empty() && pairs.back() == std::pair{3000, 5000}; }, 1s));
  for (const auto &[first, second_value] : pairs) {
    EXPECT_TRUE((first == 0 && second_value == 0) || (first == 3000 && second_value == 5000));
  }
  pairs.clear();
  std::jthread writer([&](std::stop_token stop) {
    const std::vector<MotorCommand> a{{0, 1.0}, {1, 2.0}};
    const std::vector<MotorCommand> b{{0, 3.0}, {1, 4.0}};
    while (!stop.stop_requested()) {
      transport.set_commands(a);
      transport.set_commands(b);
      std::this_thread::yield();
    }
  });
  wait_for([]() { return false; }, 250ms);
  writer.request_stop();
  writer.join();
  ASSERT_GE(pairs.size(), 10u);
  for (const auto &[first, second_value] : pairs) {
    EXPECT_TRUE((first == 3000 && second_value == 5000) ||
                (first == 1000 && second_value == 2000) || (first == 3000 && second_value == 4000));
  }
  EXPECT_FALSE(transport.faulted());
}

TEST_F(TransportTest, InvalidBatchStopsBeforeAnyElementIsCommitted)
{
  const std::vector<std::vector<MotorCommand>> invalid_batches{{{0, 1.0}, {1, kUnknownValue}},
    {{0, 1.0}, {1, 11.0}}, {{0, 1.0}, {2, 2.0}}, {{0, 1.0}, {0, 2.0}}};
  for (const auto &batch : invalid_batches) {
    MotorConfig cfg;
    cfg.name = "first";
    cfg.logical_id = 1;
    cfg.motor_id = 11;
    cfg.mode = ControlMode::Current;
    auto second = cfg;
    second.name = "second";
    second.logical_id = 2;
    second.motor_id = 12;
    TransportOptions opts;
    opts.auto_activate = true;
    auto node = std::make_shared<rclcpp::Node>("invalid_batch_io");
    std::vector<DriverFault> faults;
    VescTransport transport(
      {cfg, second}, opts, node, [&](const DriverFault &fault) { faults.push_back(fault); });
    ASSERT_TRUE(transport.set_commands({{0, 3.0}, {1, 4.0}}));
    EXPECT_FALSE(transport.set_commands(batch));
    EXPECT_TRUE(transport.faulted());
    EXPECT_FALSE(transport.active());
    EXPECT_FALSE(transport.set_commands({{0, 1.0}, {1, 2.0}}));
    ASSERT_EQ(faults.size(), 1u);
    EXPECT_FALSE(faults.front().reason.empty());
  }
}

TEST_F(TransportTest, LatestCommandsTimeoutRecoveryAndIndependentExecutor)
{
  MotorConfig cfg;
  cfg.name = "drive";
  cfg.logical_id = 1;
  cfg.motor_id = 11;
  cfg.command_timeout = 100ms;
  cfg.feedback_timeout = 100ms;
  TransportOptions opts;
  opts.tx_topic = "/transport_test/tx";
  opts.rx_topic = "/transport_test/rx";
  opts.send_period = 5ms;
  auto peer_node = std::make_shared<rclcpp::Node>("transport_test_peer");
  auto can_node = std::make_shared<rclcpp::Node>("transport_test_io");
  auto feedback_publisher =
    peer_node->create_publisher<can_msgs::msg::Frame>(opts.rx_topic, rclcpp::SensorDataQoS());
  std::vector<can_msgs::msg::Frame> received_frames;
  auto command_subscription = peer_node->create_subscription<can_msgs::msg::Frame>(opts.tx_topic,
    100, [&](can_msgs::msg::Frame::ConstSharedPtr frame) { received_frames.push_back(*frame); });
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
  VescTransport can_transport({cfg}, opts, can_node);
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
  ASSERT_TRUE(
    spin_peer_until([&]() { return can_transport.snapshot()[0].status1_at.has_value(); }, 2s));
  ASSERT_TRUE(can_transport.activate());
  // Multiple pending updates collapse to the latest value.
  ASSERT_TRUE(can_transport.set_commands({{0, 1.0}}));
  ASSERT_TRUE(can_transport.set_commands({{0, 2.0}}));
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
  EXPECT_FALSE(can_transport.active());
  EXPECT_FALSE(can_transport.set_commands({{0, 1.0}}));
  ASSERT_FALSE(can_transport.fault_reason().empty());
  ASSERT_TRUE(can_transport.activate());
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
  EXPECT_FALSE(can_transport.activate());
  publish_feedback = true;
  ASSERT_TRUE(spin_peer_until(
    [&]() {
      const auto stamp = can_transport.snapshot()[0].status1_at;
      return stamp && *stamp > SteadyClock::now() - 30ms;
    },
    300ms));
  ASSERT_TRUE(can_transport.activate());
  for (double invalid : {kUnknownValue, 1000.0}) {
    EXPECT_FALSE(can_transport.set_commands({{0, invalid}}));
    EXPECT_FALSE(
      can_transport.set_commands({{0, 1.0}}));  // A valid update cannot hide invalid input.
    ASSERT_TRUE(spin_peer_until([&]() { return can_transport.faulted(); }, 100ms));
    ASSERT_TRUE(can_transport.activate());
  }
  EXPECT_FALSE(can_transport.try_set_commands({kUnknownValue}));
  EXPECT_FALSE(can_transport.try_set_commands({1.0}));
  ASSERT_TRUE(spin_peer_until([&]() { return can_transport.faulted(); }, 100ms));
  ASSERT_TRUE(can_transport.activate());
  can_transport.request_stop();
  ASSERT_TRUE(spin_peer_until([&]() { return can_transport.faulted(); }, 100ms));
  can_transport.shutdown();
  can_transport.shutdown();
  EXPECT_FALSE(can_transport.activate());
  EXPECT_FALSE(can_transport.wait_ready(10ms));
  auto heartbeat_count = 0;
  auto heartbeat_timer = peer_node->create_wall_timer(5ms, [&]() { ++heartbeat_count; });
  EXPECT_TRUE(spin_peer_until([&]() { return heartbeat_count >= 2; }, 100ms));
}

TEST_F(TransportTest, DefaultTimerRepeatsOneCommandAt100Hz)
{
  MotorConfig cfg;
  cfg.name = "current";
  cfg.logical_id = 1;
  cfg.motor_id = 11;
  cfg.mode = ControlMode::Current;
  cfg.command_timeout = 2s;
  cfg.feedback_timeout = 2s;
  TransportOptions opts;
  opts.tx_topic = "/period_test/tx";
  opts.rx_topic = "/period_test/rx";
  auto peer = std::make_shared<rclcpp::Node>("period_test_peer");
  auto can_node = std::make_shared<rclcpp::Node>("period_test_io");
  auto feedback_pub =
    peer->create_publisher<can_msgs::msg::Frame>(opts.rx_topic, rclcpp::SensorDataQoS());
  std::vector<TimePoint> samples;
  auto cmd_sub = peer->create_subscription<can_msgs::msg::Frame>(
    opts.tx_topic, 100, [&](can_msgs::msg::Frame::ConstSharedPtr frame) {
      if (frame->data == protocol::comm_can_set_current(11, 2.0).data) {
        samples.push_back(SteadyClock::now());
      }
    });
  auto feedback_timer = peer->create_wall_timer(10ms, [&]() {
    can_msgs::msg::Frame frame;
    frame.id = 0x90B;
    frame.is_extended = true;
    frame.dlc = 8;
    feedback_pub->publish(frame);
  });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(peer);
  VescTransport transport({cfg}, opts, can_node);
  transport.start();
  auto deadline = SteadyClock::now() + 2s;
  while (!transport.snapshot()[0].status1_at && SteadyClock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(transport.activate());
  ASSERT_TRUE(
    transport.set_commands({{0, 2.0}}));  // One input; the timer must keep transmitting it.
  deadline = SteadyClock::now() + 200ms;
  while (samples.empty() && SteadyClock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_FALSE(samples.empty());
  samples.clear();
  deadline = SteadyClock::now() + 600ms;
  while (SteadyClock::now() < deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_GE(samples.size(), 48u);
  ASSERT_LE(samples.size(), 72u);
  const double hz =
    (samples.size() - 1) / std::chrono::duration<double>(samples.back() - samples.front()).count();
  EXPECT_GT(hz, 80.0);
  EXPECT_LT(hz, 120.0);
  EXPECT_FALSE(transport.faulted());
  transport.shutdown();
}

TEST_F(TransportTest, FaultCallbackRunsOutsideStateLockWithoutWaitingForSendTimer)
{
  MotorConfig cfg;
  cfg.name = "drive";
  cfg.logical_id = 1;
  cfg.motor_id = 11;
  cfg.feedback_timeout = 2s;
  TransportOptions opts;
  opts.send_period = 2s;
  opts.tx_topic = "/fault_test/tx";
  opts.rx_topic = "/fault_test/rx";
  auto peer = std::make_shared<rclcpp::Node>("fault_test_peer");
  auto can_node = std::make_shared<rclcpp::Node>("fault_test_io");
  auto rx_pub =
    peer->create_publisher<can_msgs::msg::Frame>(opts.rx_topic, rclcpp::SensorDataQoS());
  std::mutex notification_mutex;
  std::vector<std::string> faults;
  VescTransport *transport_ptr = nullptr;
  VescTransport transport({cfg}, opts, can_node, [&](const DriverFault &fault) {
    const auto states = transport_ptr->snapshot();  // Would deadlock if the callback held the lock.
    std::lock_guard<std::mutex> lock(notification_mutex);
    faults.push_back(fault.reason + ": states=" + std::to_string(states.size()));
  });
  transport_ptr = &transport;
  transport.start();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(peer);
  const auto wait_for = [&](auto condition, std::chrono::milliseconds timeout) {
    const auto deadline = SteadyClock::now() + timeout;
    while (SteadyClock::now() < deadline) {
      exec.spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return condition();
  };
  const auto fault_count = [&]() {
    std::lock_guard<std::mutex> lock(notification_mutex);
    return faults.size();
  };
  ASSERT_TRUE(wait_for([&]() { return rx_pub->get_subscription_count() > 0; }, 2s));
  can_msgs::msg::Frame frame;
  frame.is_error = true;
  frame.id = 0x40;
  rx_pub->publish(frame);
  ASSERT_TRUE(wait_for([&]() { return fault_count() == 1; }, 500ms));
  EXPECT_TRUE(transport.faulted());
  {
    std::lock_guard<std::mutex> lock(notification_mutex);
    EXPECT_NE(faults.front().find("CAN error frame"), std::string::npos);
  }
  rx_pub->publish(frame);
  wait_for([]() { return false; }, 50ms);
  EXPECT_EQ(fault_count(), 1u);

  frame.is_error = false;
  frame.is_extended = true;
  frame.id = 0x90B;
  frame.dlc = 8;
  rx_pub->publish(frame);
  ASSERT_TRUE(
    wait_for([&]() { return transport.snapshot().front().status1_at.has_value(); }, 500ms));
  ASSERT_TRUE(transport.activate());
  EXPECT_FALSE(transport.set_commands({{0, kUnknownValue}}));
  EXPECT_EQ(fault_count(), 2u);  // Invalid non-RT input notifies directly in this call.
  EXPECT_TRUE(transport.faulted());
  EXPECT_NE(transport.fault_reason().find("invalid command"), std::string::npos);
  ASSERT_TRUE(transport.activate());
  transport.request_stop();
  transport.shutdown();
  EXPECT_EQ(fault_count(), 3u);  // Shutdown must preserve a pending hardware error notification.
}

TEST_F(TransportTest, SelectivePollingRoutesDeviceFaultsAndNotifiesOnlyCodeChanges)
{
  MotorConfig cfg;
  cfg.name = "drive";
  cfg.logical_id = 1;
  cfg.motor_id = 11;
  cfg.feedback_timeout = 2s;
  TransportOptions opts;
  opts.tx_topic = "/device_fault_test/tx";
  opts.rx_topic = "/device_fault_test/rx";
  opts.send_period = 2s;  // Device errors must not wait for the motor command timer.
  auto peer = std::make_shared<rclcpp::Node>("device_fault_peer");
  auto can_node = std::make_shared<rclcpp::Node>("device_fault_io");
  auto rx_pub =
    peer->create_publisher<can_msgs::msg::Frame>(opts.rx_topic, rclcpp::SensorDataQoS());
  std::uint8_t reply_code = 0;
  std::vector<TimePoint> polls;
  auto tx_sub = peer->create_subscription<can_msgs::msg::Frame>(
    opts.tx_topic, 100, [&](can_msgs::msg::Frame::ConstSharedPtr request) {
      if (request->id != 0x80B || request->dlc != 7) {
        return;
      }
      EXPECT_EQ(request->data, (std::array<std::uint8_t, 8>{254, 0, 50, 0, 0, 128, 0, 0}));
      polls.push_back(SteadyClock::now());
      can_msgs::msg::Frame reply;
      reply.id = 0x8FE;
      reply.is_extended = true;
      reply.dlc = 8;
      reply.data = {11, 1, 50, 0, 0, 128, 0, reply_code};
      rx_pub->publish(reply);
    });
  auto feedback_timer = peer->create_wall_timer(10ms, [&]() {
    can_msgs::msg::Frame frame;
    frame.id = 0x90B;
    frame.is_extended = true;
    frame.dlc = 8;
    rx_pub->publish(frame);
  });
  std::mutex mutex;
  std::vector<DriverFault> faults;
  VescTransport *transport_ptr = nullptr;
  VescTransport transport({cfg}, opts, can_node, [&](const DriverFault &fault) {
    ASSERT_TRUE(fault.device_fault.has_value());
    EXPECT_EQ(transport_ptr->snapshot()[0].fault_code, fault.device_fault->code);
    std::lock_guard<std::mutex> lock(mutex);
    faults.push_back(fault);
  });
  transport_ptr = &transport;
  transport.start();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(peer);
  const auto wait_for = [&](auto condition, std::chrono::milliseconds timeout) {
    const auto deadline = SteadyClock::now() + timeout;
    while (SteadyClock::now() < deadline) {
      exec.spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return condition();
  };
  const auto count = [&]() {
    std::lock_guard<std::mutex> lock(mutex);
    return faults.size();
  };
  ASSERT_TRUE(wait_for([&]() { return transport.snapshot()[0].fault_code == 0; }, 2s));
  ASSERT_TRUE(transport.activate());
  reply_code = 4;
  ASSERT_TRUE(wait_for([&]() { return count() == 1; }, 500ms));
  EXPECT_TRUE(transport.faulted());
  EXPECT_FALSE(transport.active());
  EXPECT_FALSE(transport.activate());
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_TRUE(faults[0].device_fault.has_value());
    EXPECT_EQ(faults[0].device_fault->motor_id, 11);
    EXPECT_EQ(faults[0].device_fault->code, 4);
  }
  wait_for([]() { return false; }, 250ms);
  EXPECT_EQ(count(), 1u);
  reply_code = 5;
  ASSERT_TRUE(wait_for([&]() { return count() == 2; }, 500ms));
  reply_code = 0;
  ASSERT_TRUE(wait_for([&]() { return transport.snapshot()[0].fault_code == 0; }, 500ms));
  EXPECT_TRUE(transport.faulted());
  EXPECT_FALSE(transport.active());
  EXPECT_EQ(count(), 2u);
  EXPECT_TRUE(transport.activate());
  ASSERT_GE(polls.size(), 4u);
  const auto hz =
    (polls.size() - 1) / std::chrono::duration<double>(polls.back() - polls.front()).count();
  EXPECT_GT(hz, 8.0);
  EXPECT_LT(hz, 12.0);
  transport.shutdown();
}

TEST_F(TransportTest, RejectsReservedReplyIdAndIgnoresUnrequestedRepliesWhenPollingDisabled)
{
  MotorConfig cfg;
  cfg.name = "drive";
  cfg.logical_id = 1;
  cfg.motor_id = 11;
  auto peer = std::make_shared<rclcpp::Node>("disabled_fault_peer");
  auto can_node = std::make_shared<rclcpp::Node>("disabled_fault_io");
  TransportOptions opts;
  opts.tx_topic = "/disabled_fault_test/tx";
  opts.rx_topic = "/disabled_fault_test/rx";
  cfg.motor_id = 254;
  EXPECT_THROW(VescTransport({cfg}, opts, can_node), std::invalid_argument);
  cfg.motor_id = 11;
  opts.fault_period = -1ms;
  EXPECT_THROW(VescTransport({cfg}, opts, can_node), std::invalid_argument);
  opts.fault_period = 0ms;
  VescTransport transport({cfg}, opts, can_node);
  auto pub = peer->create_publisher<can_msgs::msg::Frame>(opts.rx_topic, rclcpp::SensorDataQoS());
  transport.start();
  can_msgs::msg::Frame reply;
  reply.id = 0x8FE;
  reply.is_extended = true;
  reply.dlc = 8;
  reply.data = {11, 1, 50, 0, 0, 128, 0, 4};
  for (int i = 0; i < 50; ++i) {
    pub->publish(reply);
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_EQ(transport.snapshot()[0].fault_code, -1);
  EXPECT_FALSE(transport.faulted());
}

TEST_F(TransportTest, StandaloneStartsOnFreshCommandsAndRebootsOneMotorWithoutReplay)
{
  MotorConfig cfg;
  cfg.name = "drive";
  cfg.logical_id = 1;
  cfg.motor_id = 11;
  cfg.mode = ControlMode::Current;
  cfg.command_timeout = 2s;
  cfg.feedback_timeout = 100ms;
  auto second = cfg;
  second.name = "other";
  second.logical_id = 2;
  second.motor_id = 12;
  TransportOptions opts;
  opts.tx_topic = "/auto_reboot_test/tx";
  opts.rx_topic = "/auto_reboot_test/rx";
  opts.auto_activate = true;
  auto peer = std::make_shared<rclcpp::Node>("auto_reboot_peer");
  auto can_node = std::make_shared<rclcpp::Node>("auto_reboot_io");
  auto pub = peer->create_publisher<can_msgs::msg::Frame>(opts.rx_topic, rclcpp::SensorDataQoS());
  std::vector<can_msgs::msg::Frame> frames;
  auto sub = peer->create_subscription<can_msgs::msg::Frame>(opts.tx_topic, 100,
    [&](can_msgs::msg::Frame::ConstSharedPtr frame) { frames.push_back(*frame); });
  bool feedback = false;
  auto timer = peer->create_wall_timer(10ms, [&]() {
    if (!feedback) {
      return;
    }
    for (std::uint8_t id : {11, 12}) {
      can_msgs::msg::Frame frame;
      frame.id = 0x900 | id;
      frame.is_extended = true;
      frame.dlc = 8;
      pub->publish(frame);
    }
  });
  VescTransport transport({cfg, second}, opts, can_node);
  transport.start();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(peer);
  const auto wait_for = [&](auto predicate, std::chrono::milliseconds timeout) {
    const auto deadline = SteadyClock::now() + timeout;
    while (SteadyClock::now() < deadline) {
      exec.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return predicate();
  };
  ASSERT_TRUE(transport.set_commands({{0, 1.0}}));  // Input may precede initial feedback.
  ASSERT_TRUE(transport.set_commands({{1, 2.0}}));
  wait_for([]() { return false; }, 50ms);
  EXPECT_FALSE(transport.active());
  EXPECT_FALSE(transport.faulted());
  feedback = true;
  ASSERT_TRUE(wait_for([&]() { return transport.active(); }, 2s));
  ASSERT_TRUE(wait_for(
    [&]() {
      return std::any_of(frames.begin(), frames.end(), [](const auto &frame) {
        return frame.id == 0x10B && frame.data == protocol::comm_can_set_current(11, 1.0).data;
      });
    },
    100ms));
  EXPECT_FALSE(transport.reboot(2));
  frames.clear();
  ASSERT_TRUE(transport.reboot(1));
  EXPECT_FALSE(transport.activate());
  ASSERT_TRUE(wait_for(
    [&]() {
      return std::any_of(frames.begin(), frames.end(), [](const auto &frame) {
        return frame.id == 0x80C && frame.dlc == 3 && frame.data[2] == 29;
      });
    },
    100ms));
  EXPECT_FALSE(transport.reboot(1));
  EXPECT_FALSE(transport.set_commands({{1, 2.0}}));  // Boot guard discards commands and old STATUS.
  EXPECT_FALSE(transport.snapshot()[1].status1_at.has_value());
  wait_for([]() { return false; }, 600ms);
  EXPECT_TRUE(transport.snapshot()[1].status1_at.has_value());
  EXPECT_FALSE(transport.active());
  EXPECT_EQ(std::count_if(frames.begin(), frames.end(),
              [](const auto &frame) { return frame.dlc == 3 && frame.data[2] == 29; }),
    1);
  // DDS may still deliver commands published before reboot() was called.
  // The reboot packet marks the boundary on the same publisher's ordered stream.
  const auto reboot_frame = std::find_if(frames.begin(), frames.end(),
    [](const auto &frame) { return frame.id == 0x80C && frame.dlc == 3 && frame.data[2] == 29; });
  ASSERT_NE(reboot_frame, frames.end());
  bool saw_stop_after_reboot = false;
  for (auto frame = reboot_frame + 1; frame != frames.end(); ++frame) {
    if ((frame->id >> 8) == 1) {
      saw_stop_after_reboot = true;
      EXPECT_EQ(frame->data, (std::array<std::uint8_t, 8>{}));
    }
  }
  EXPECT_TRUE(saw_stop_after_reboot);
  ASSERT_TRUE(transport.set_commands({{0, 1.0}}));
  ASSERT_TRUE(transport.set_commands({{1, 2.0}}));
  ASSERT_TRUE(wait_for([&]() { return transport.active(); }, 100ms));
}

}  // namespace
}  // namespace vesc_can_driver
