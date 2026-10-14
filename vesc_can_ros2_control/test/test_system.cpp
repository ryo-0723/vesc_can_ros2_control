#include <memory>
#include <string>
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
    joint.parameters = {{"logical_id", "1"}, {"controller_id", "11"}, {"pole_pairs", "7"}};
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
  steer_joint.parameters = {{"logical_id", "2"}, {"controller_id", "21"}, {"pole_pairs", "7"}};
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
  for (const auto &name : {"logical_id", "controller_id", "pole_pairs"}) {
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
         {"logical_id", "1.5"}, {"controller_id", "255"}, {"controller_id", "1.5"},
         {"pole_pairs", "0"}, {"pole_pairs", "7.5"}, {"gear_ratio", "0"}, {"direction", "0"},
         {"max_velocity", "nan"}, {"max_velocity", "20rad"}, {"command_timeout_ms", "0"}}) {
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
  second.parameters["controller_id"] = "12";
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
    const auto interface = mode == std::string("current_velocity") ? "velocity" : mode;
    joint.command_interfaces[0].name = interface;
    joint.state_interfaces[0].name = interface;
    joint.parameters["control_mode"] = mode;
    if (mode == std::string("current_velocity")) {
      joint.parameters["startup_current_a"] = "2.0";
      joint.parameters["rpm_control_threshold_rpm"] = "50.0";
    }
    VescSystem hardware;
    EXPECT_EQ(hardware.init(params), hardware_interface::CallbackReturn::SUCCESS);
  }
}

TEST_F(SystemConfigurationTest, RejectsMismatchedOrIncompleteHybridSettings)
{
  auto params = make_parameters();
  params.hardware_info.joints[0].parameters["control_mode"] = "current";
  VescSystem wrong_interface;
  EXPECT_EQ(wrong_interface.init(params), hardware_interface::CallbackReturn::ERROR);
  params.hardware_info.joints[0].parameters["control_mode"] = "current_velocity";
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

}  // namespace
}  // namespace vesc_can_ros2_control
