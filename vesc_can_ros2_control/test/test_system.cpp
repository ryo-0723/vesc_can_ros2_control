#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "gtest/gtest.h"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vesc_can_ros2_control/vesc_system.hpp"

namespace vesc_can_ros2_control
{
namespace
{
class SystemConfigurationTest : public ::testing::Test
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

  static hardware_interface::HardwareComponentParams make_parameters()
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "drive_joint";
    joint.type = "joint";
    joint.parameters = {{"logical_id", "1"}, {"motor_id", "11"}, {"pole_pairs", "7"}};
    hardware_interface::InterfaceInfo interface;
    interface.name = "velocity";
    joint.command_interfaces.push_back(interface);
    joint.state_interfaces.push_back(interface);
    hardware_interface::HardwareComponentParams params;
    params.hardware_info.name = "VescConfigurationTest";
    params.hardware_info.type = "system";
    params.hardware_info.joints.push_back(joint);
    params.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    return params;
  }
};

TEST_F(SystemConfigurationTest, PositionDefaultsPreservePiPrecision)
{
  hardware_interface::ComponentInfo steer_joint;
  steer_joint.name = "steer_joint";
  steer_joint.type = "joint";
  steer_joint.parameters = {{"logical_id", "2"}, {"motor_id", "21"}, {"pole_pairs", "7"}};
  hardware_interface::InterfaceInfo pos_interface;
  pos_interface.name = "position";
  steer_joint.command_interfaces.push_back(pos_interface);
  steer_joint.state_interfaces.push_back(pos_interface);

  hardware_interface::HardwareComponentParams params;
  params.hardware_info.name = "VescDefaultPositionTest";
  params.hardware_info.type = "system";
  params.hardware_info.joints.push_back(steer_joint);
  params.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  VescSystem hardware;
  // Omitting the limits must use exactly +/-pi, within the native single-turn range.
  EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(SystemConfigurationTest, RejectsUnsupportedOrAmbiguousCommandInterfaces)
{
  for (const auto &mode : {"effort", "unknown"}) {
    SCOPED_TRACE(mode);
    auto params = make_parameters();
    params.hardware_info.joints[0].command_interfaces[0].name = mode;
    VescSystem hardware;
    EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::ERROR);
  }
  auto params = make_parameters();
  auto &joint = params.hardware_info.joints[0];
  joint.command_interfaces.push_back(joint.command_interfaces[0]);
  VescSystem hardware;
  EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(SystemConfigurationTest, RequiresMotorIdentityAndMatchingFeedbackInterface)
{
  for (const auto &name : {"logical_id", "motor_id", "pole_pairs"}) {
    SCOPED_TRACE(name);
    auto params = make_parameters();
    params.hardware_info.joints[0].parameters.erase(name);
    VescSystem hardware;
    EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::ERROR);
  }
  auto params = make_parameters();
  params.hardware_info.joints[0].state_interfaces[0].name = "position";
  VescSystem hardware;
  EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(SystemConfigurationTest, RejectsInvalidNumericMotorSettings)
{
  for (const auto &[name, value] : {std::pair{"logical_id", "256"}, {"logical_id", "-1"},
         {"logical_id", "1.5"}, {"motor_id", "255"}, {"motor_id", "1.5"}, {"pole_pairs", "0"},
         {"pole_pairs", "7.5"}, {"gear_ratio", "0"}, {"direction", "0"}, {"max_velocity", "nan"},
         {"max_velocity", "20rad"}, {"command_timeout_ms", "0"}}) {
    SCOPED_TRACE(std::string(name) + "=" + value);
    auto params = make_parameters();
    params.hardware_info.joints[0].parameters[name] = value;
    VescSystem hardware;
    EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::ERROR);
  }
}

TEST_F(SystemConfigurationTest, LogicalIdsAreUniqueAndIndependentOfCanIds)
{
  auto params = make_parameters();
  auto second = params.hardware_info.joints.front();
  second.name = "second_joint";
  second.parameters["motor_id"] = "12";
  params.hardware_info.joints.push_back(second);
  VescSystem duplicate_id;
  EXPECT_EQ(duplicate_id.init(params), hardware_interface::CallbackReturn::ERROR);
  params.hardware_info.joints[1].parameters["logical_id"] = "255";
  VescSystem distinct_ids;
  EXPECT_EQ(distinct_ids.init(params), hardware_interface::CallbackReturn::SUCCESS);
}
TEST_F(SystemConfigurationTest, SupportsAllFiveModeInterfaces)
{
  for (const auto &mode : {"current", "velocity", "position", "duty_cycle", "current_velocity"}) {
    SCOPED_TRACE(mode);
    auto params = make_parameters();
    auto &joint = params.hardware_info.joints[0];
    joint.command_interfaces[0].name = mode;
    joint.state_interfaces[0].name = mode == std::string("current_velocity") ? "velocity" : mode;
    joint.parameters["control_mode"] = mode;
    if (mode == std::string("current_velocity")) {
      joint.parameters["startup_current_a"] = "2.0";
      joint.parameters["rpm_control_threshold_rpm"] = "50.0";
    }
    VescSystem hardware;
    EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::SUCCESS);
  }
}

TEST_F(SystemConfigurationTest, HybridHasItsOwnCommandInterfaceAndVelocityFeedback)
{
  auto params = make_parameters();
  auto &joint = params.hardware_info.joints[0];
  joint.command_interfaces[0].name = "current_velocity";
  joint.parameters["startup_current_a"] = "2.0";
  // With no explicit mode, the dedicated command interface selects hybrid control.
  VescSystem inferred;
  ASSERT_EQ(inferred.init(params), hardware_interface::CallbackReturn::SUCCESS);
  joint.parameters["control_mode"] = "velocity";
  VescSystem wrong_mode;
  EXPECT_EQ(wrong_mode.init(params), hardware_interface::CallbackReturn::ERROR);
  joint.parameters["control_mode"] = "current_velocity";
  joint.command_interfaces[0].name = "velocity";
  VescSystem old_interface;
  EXPECT_EQ(old_interface.init(params), hardware_interface::CallbackReturn::ERROR);
  joint.command_interfaces[0].name = "current_velocity";
  joint.state_interfaces[0].name = "current";
  VescSystem missing_velocity;
  EXPECT_EQ(missing_velocity.init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(SystemConfigurationTest, RejectsMismatchedOrIncompleteHybridSettings)
{
  auto params = make_parameters();
  params.hardware_info.joints[0].parameters["control_mode"] = "current";
  VescSystem wrong_interface;
  EXPECT_EQ(wrong_interface.init(params), hardware_interface::CallbackReturn::ERROR);
  params.hardware_info.joints[0].parameters["control_mode"] = "current_velocity";
  params.hardware_info.joints[0].command_interfaces[0].name = "current_velocity";
  VescSystem no_startup;
  EXPECT_EQ(no_startup.init(params), hardware_interface::CallbackReturn::ERROR);
  params.hardware_info.joints[0].parameters["startup_current_a"] = "2.0";
  params.hardware_info.joints[0].parameters["max_current_a"] = "1.0";
  VescSystem excess_current;
  EXPECT_EQ(excess_current.init(params), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(SystemConfigurationTest, RejectsInvalidDutyLimits)
{
  for (const auto &value : {"0", "-0.5", "1.1", "nan"}) {
    auto params = make_parameters();
    auto &joint = params.hardware_info.joints[0];
    joint.command_interfaces[0].name = "duty_cycle";
    joint.state_interfaces[0].name = "duty_cycle";
    joint.parameters["max_duty"] = value;
    VescSystem hardware;
    EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::ERROR);
  }
}

TEST_F(SystemConfigurationTest, RejectsInvalidHardwarePeriods)
{
  for (const auto &name : {"send_period_ms", "state_publish_period_ms", "activation_timeout_ms"}) {
    for (const auto &value : {"0", "-1", "1.5"}) {
      auto params = make_parameters();
      params.hardware_info.hardware_parameters[name] = value;
      VescSystem hardware;
      EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::ERROR);
    }
  }
}

TEST_F(SystemConfigurationTest, IndependentComponentsSurviveCleanupAndReconfigure)
{
  using namespace std::chrono_literals;
  auto first_params = make_parameters();
  auto second_params = make_parameters();
  first_params.hardware_info.name = "VescFirst";
  second_params.hardware_info.name = "VescSecond";
  first_params.hardware_info.hardware_parameters = {{"can_tx_topic", "/hardware_a/can/tx"},
    {"can_rx_topic", "/hardware_a/can/rx"}, {"error_topic", "/hardware_a/vesc/event"},
    {"fault_poll_period_ms", "0"}};
  second_params.hardware_info.hardware_parameters = {{"can_tx_topic", "/hardware_b/can/tx"},
    {"can_rx_topic", "/hardware_b/can/rx"}, {"error_topic", "/hardware_b/vesc/event"},
    {"fault_poll_period_ms", "0"}};
  VescSystem first;
  VescSystem second;
  ASSERT_EQ(first.init(first_params), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(second.init(second_params), hardware_interface::CallbackReturn::SUCCESS);
  first.on_export_state_interfaces();
  first.on_export_command_interfaces();
  second.on_export_state_interfaces();
  second.on_export_command_interfaces();
  const rclcpp_lifecycle::State lifecycle_state;
  ASSERT_EQ(first.on_configure(lifecycle_state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_configure(lifecycle_state), hardware_interface::CallbackReturn::SUCCESS);
  auto observer = std::make_shared<rclcpp::Node>("hardware_lifetime_test");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(observer);
  int first_states = 0;
  int second_states = 0;
  int second_frames = 0;
  auto first_sub = observer->create_subscription<vesc_can_msgs::msg::VescState>(
    "/VescFirst_can_io/motors/drive_joint/state", 10,
    [&](vesc_can_msgs::msg::VescState::ConstSharedPtr) { ++first_states; });
  auto second_sub = observer->create_subscription<vesc_can_msgs::msg::VescState>(
    "/VescSecond_can_io/motors/drive_joint/state", 10,
    [&](vesc_can_msgs::msg::VescState::ConstSharedPtr) { ++second_states; });
  auto frame_sub = observer->create_subscription<can_msgs::msg::Frame>(
    "/hardware_b/can/tx", 100, [&](can_msgs::msg::Frame::ConstSharedPtr) { ++second_frames; });
  const auto wait_for = [&](auto condition) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      exec.spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    }
    return condition();
  };
  ASSERT_TRUE(
    wait_for([&]() { return first_states >= 2 && second_states >= 2 && second_frames >= 2; }));
  ASSERT_EQ(first.on_cleanup(lifecycle_state), hardware_interface::CallbackReturn::SUCCESS);
  const auto states_before = second_states;
  const auto frames_before = second_frames;
  ASSERT_TRUE(wait_for(
    [&]() { return second_states >= states_before + 3 && second_frames >= frames_before + 5; }));
  const auto first_before = first_states;
  ASSERT_EQ(first.on_configure(lifecycle_state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(wait_for([&]() { return first_states >= first_before + 3; }));
  EXPECT_EQ(first.on_shutdown(lifecycle_state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(second.on_cleanup(lifecycle_state), hardware_interface::CallbackReturn::SUCCESS);
}

}  // namespace
}  // namespace vesc_can_ros2_control
