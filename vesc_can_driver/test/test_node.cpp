#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "vesc_can_driver/vesc_node.hpp"

namespace vesc_can_driver
{
namespace
{
using namespace std::chrono_literals;
using Commands = vesc_can_msgs::msg::VescCommandArray;
using States = vesc_can_msgs::msg::VescStateArray;
using Event = vesc_can_msgs::msg::VescEvent;
using Frame = can_msgs::msg::Frame;

class NodeTest : public ::testing::Test
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

  static rclcpp::NodeOptions options(const std::string &ns, const std::string &name)
  {
    return rclcpp::NodeOptions()
      .use_global_arguments(false)
      .arguments({"--ros-args", "-r", "__ns:=" + ns, "-r", "__node:=" + name})
      .parameter_overrides({rclcpp::Parameter("motors", std::vector<std::string>{"motor"}),
        rclcpp::Parameter("motor.logical_id", 1), rclcpp::Parameter("motor.motor_id", 11),
        rclcpp::Parameter("motor.control_mode", "current"),
        rclcpp::Parameter("motor.feedback_timeout_ms", 2000),
        rclcpp::Parameter("motor.command_timeout_ms", 2000),
        rclcpp::Parameter("fault_poll_period_ms", 0)});
  }
};

TEST_F(NodeTest, NamespacesIsolateCommandsFeedbackErrorsAndShutdownInOneProcess)
{
  auto first = std::make_shared<VescNode>(options("/node_a", "driver"));
  auto second = std::make_shared<VescNode>(options("/node_b", "driver"));
  auto observer = std::make_shared<rclcpp::Node>("multi_node_test");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(first);
  exec.add_node(second);
  exec.add_node(observer);
  std::array<int, 2> frame_count{}, state_count{}, event_count{};
  std::array<bool, 2> command_seen{}, feedback_seen{};
  std::vector<rclcpp::Publisher<Commands>::SharedPtr> commands;
  std::vector<rclcpp::Publisher<Frame>::SharedPtr> feedback;
  std::vector<rclcpp::Subscription<Frame>::SharedPtr> frames;
  std::vector<rclcpp::Subscription<States>::SharedPtr> states;
  std::vector<rclcpp::Subscription<Event>::SharedPtr> events;
  for (std::size_t idx = 0; idx < 2; ++idx) {
    const std::string ns = idx == 0 ? "/node_a" : "/node_b";
    commands.push_back(observer->create_publisher<Commands>(ns + "/vesc/commands", 10));
    feedback.push_back(observer->create_publisher<Frame>(ns + "/can/rx", rclcpp::SensorDataQoS()));
    frames.push_back(observer->create_subscription<Frame>(
      ns + "/can/tx", 100, [&, idx](Frame::ConstSharedPtr msg) {
        ++frame_count[idx];
        const auto expected = protocol::comm_can_set_current(11, idx + 1.0);
        if (msg->id == expected.id && msg->data == expected.data) {
          command_seen[idx] = true;
        }
      }));
    states.push_back(observer->create_subscription<States>(
      ns + "/vesc/state", 10, [&, idx](States::ConstSharedPtr msg) {
        ++state_count[idx];
        if (msg->states.size() == 1 && msg->states[0].logical_id == 1 &&
            std::abs(msg->states[0].velocity_rad_s - erpm_to_vel(MotorConfig{}, 100 * (idx + 1))) <
              1e-9) {
          feedback_seen[idx] = true;
        }
      }));
    events.push_back(observer->create_subscription<Event>(ns + "/vesc/event",
      rclcpp::QoS(1).reliable().transient_local(), [&, idx](Event::ConstSharedPtr msg) {
        EXPECT_EQ(msg->logical_id, 1);
        EXPECT_NE(msg->message.find("Unknown logical_id"), std::string::npos);
        ++event_count[idx];
      }));
  }
  const auto tick = [&]() {
    for (std::size_t idx = 0; idx < 2; ++idx) {
      Frame frame;
      frame.is_extended = true;
      frame.id = (9 << 8) | 11;
      frame.dlc = 8;
      frame.data[3] = static_cast<std::uint8_t>(100 * (idx + 1));
      feedback[idx]->publish(frame);
    }
    exec.spin_some();
    std::this_thread::sleep_for(2ms);
  };
  const auto wait_for = [&](auto condition) {
    const auto deadline = SteadyClock::now() + 3s;
    while (SteadyClock::now() < deadline) {
      tick();
      if (condition()) {
        return true;
      }
    }
    return condition();
  };
  ASSERT_TRUE(wait_for([&]() {
    return commands[0]->get_subscription_count() && commands[1]->get_subscription_count() &&
           feedback_seen[0] && feedback_seen[1];
  }));
  for (std::size_t idx = 0; idx < 2; ++idx) {
    Commands msg;
    vesc_can_msgs::msg::VescCommand cmd;
    cmd.logical_id = 1;
    cmd.value = idx + 1.0;
    msg.commands.push_back(cmd);
    commands[idx]->publish(msg);
  }
  ASSERT_TRUE(wait_for([&]() { return command_seen[0] && command_seen[1]; }));
  Commands invalid;
  invalid.commands.emplace_back().logical_id = 2;
  commands[0]->publish(invalid);
  ASSERT_TRUE(wait_for([&]() { return event_count[0] == 1; }));
  EXPECT_EQ(event_count[1], 0);
  auto reboot_a = observer->create_client<vesc_can_msgs::srv::Reboot>("/node_a/driver/reboot");
  auto reboot_b = observer->create_client<vesc_can_msgs::srv::Reboot>("/node_b/driver/reboot");
  ASSERT_TRUE(
    wait_for([&]() { return reboot_a->service_is_ready() && reboot_b->service_is_ready(); }));
  exec.remove_node(first);
  first.reset();
  const auto frames_before = frame_count[1];
  const auto states_before = state_count[1];
  ASSERT_TRUE(wait_for(
    [&]() { return frame_count[1] >= frames_before + 5 && state_count[1] >= states_before + 3; }));
  EXPECT_EQ(event_count[1], 0);
  const auto nodes = observer->get_node_names();
  EXPECT_NE(std::find(nodes.begin(), nodes.end(), "/node_b/driver_can_io"), nodes.end());
}

TEST_F(NodeTest, RenamedNodesGiveTheirCanNodesDistinctNames)
{
  auto first = std::make_shared<VescNode>(options("/renamed", "first"));
  auto second = std::make_shared<VescNode>(options("/renamed", "second"));
  auto observer = std::make_shared<rclcpp::Node>("renamed_nodes_test");
  const auto deadline = SteadyClock::now() + 3s;
  bool discovered = false;
  while (SteadyClock::now() < deadline) {
    const auto names = observer->get_node_names();
    discovered = std::find(names.begin(), names.end(), "/renamed/first_can_io") != names.end() &&
                 std::find(names.begin(), names.end(), "/renamed/second_can_io") != names.end();
    if (discovered) {
      break;
    }
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_TRUE(discovered);
}
TEST_F(NodeTest, OversizedCommandArrayIsRejectedBeforeCopyingEntries)
{
  auto driver = std::make_shared<VescNode>(options("/oversized", "driver"));
  auto peer = std::make_shared<rclcpp::Node>("oversized_array_peer");
  auto commands = peer->create_publisher<Commands>("/oversized/vesc/commands", 10);
  bool rejected = false;
  auto events = peer->create_subscription<Event>("/oversized/vesc/event",
    rclcpp::QoS(1).reliable().transient_local(), [&](Event::ConstSharedPtr event) {
      rejected = event->logical_id == 1 && event->message == "Too many motor commands in one array";
    });
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(driver);
  exec.add_node(peer);
  const auto deadline = SteadyClock::now() + 2s;
  Commands batch;
  batch.commands.resize(1024);
  for (auto &command : batch.commands) {
    command.logical_id = 1;
    command.value = 1.0;
  }
  while (!rejected && SteadyClock::now() < deadline) {
    commands->publish(batch);
    exec.spin_some();
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_TRUE(rejected);
}

}  // namespace
}  // namespace vesc_can_driver
