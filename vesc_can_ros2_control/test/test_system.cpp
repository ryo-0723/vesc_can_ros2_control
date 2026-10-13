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
    joint.parameters = {{"controller_id", "11"}, {"pole_pairs", "7"}};
    hardware_interface::InterfaceInfo interface;
    interface.name = "velocity";
    joint.command_interfaces.push_back(interface);
    joint.state_interfaces.push_back(interface);
    hardware_interface::HardwareComponentParams parameters;
    parameters.hardware_info.name = "VescConfigurationTest";
    parameters.hardware_info.type = "system";
    parameters.hardware_info.joints.push_back(joint);
    parameters.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    return parameters;
  }
};

TEST_F(SystemConfigurationTest, PositionDefaultsPreservePiPrecision)
{
  hardware_interface::ComponentInfo steer_joint;
  steer_joint.name = "steer_joint";
  steer_joint.type = "joint";
  steer_joint.parameters = {{"controller_id", "21"}, {"pole_pairs", "7"}};
  hardware_interface::InterfaceInfo position_interface;
  position_interface.name = "position";
  steer_joint.command_interfaces.push_back(position_interface);
  steer_joint.state_interfaces.push_back(position_interface);

  hardware_interface::HardwareComponentParams parameters;
  parameters.hardware_info.name = "VescDefaultPositionTest";
  parameters.hardware_info.type = "system";
  parameters.hardware_info.joints.push_back(steer_joint);
  parameters.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  VescSystem hardware;
  // Omitting the limits must use exactly +/-pi, within the native single-turn range.
  EXPECT_EQ(hardware.init(parameters), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(SystemConfigurationTest, RejectsUnsupportedOrAmbiguousCommandInterfaces)
{
  for (const auto &control_mode : {"effort", "duty_cycle"}) {
    SCOPED_TRACE(control_mode);
    auto parameters = make_parameters();
    parameters.hardware_info.joints[0].command_interfaces[0].name = control_mode;
    VescSystem hardware;
    EXPECT_EQ(hardware.init(parameters), hardware_interface::CallbackReturn::ERROR);
  }
  auto parameters = make_parameters();
  auto &joint = parameters.hardware_info.joints[0];
  joint.command_interfaces.push_back(joint.command_interfaces[0]);
  VescSystem hardware;
  EXPECT_EQ(hardware.init(parameters), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(SystemConfigurationTest, RequiresMotorIdentityAndMatchingFeedbackInterface)
{
  for (const auto &name : {"controller_id", "pole_pairs"}) {
    SCOPED_TRACE(name);
    auto parameters = make_parameters();
    parameters.hardware_info.joints[0].parameters.erase(name);
    VescSystem hardware;
    EXPECT_EQ(hardware.init(parameters), hardware_interface::CallbackReturn::ERROR);
  }
  auto parameters = make_parameters();
  parameters.hardware_info.joints[0].state_interfaces[0].name = "position";
  VescSystem hardware;
  EXPECT_EQ(hardware.init(parameters), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(SystemConfigurationTest, RejectsInvalidNumericMotorSettings)
{
  for (const auto &[name, value] : {std::pair{"controller_id", "255"}, {"controller_id", "1.5"},
         {"pole_pairs", "0"}, {"pole_pairs", "7.5"}, {"gear_ratio", "0"}, {"direction", "0"},
         {"max_velocity", "nan"}, {"max_velocity", "20rad"}, {"command_timeout_ms", "0"}}) {
    SCOPED_TRACE(std::string(name) + "=" + value);
    auto parameters = make_parameters();
    parameters.hardware_info.joints[0].parameters[name] = value;
    VescSystem hardware;
    EXPECT_EQ(hardware.init(parameters), hardware_interface::CallbackReturn::ERROR);
  }
}
TEST_F(SystemConfigurationTest, SupportsCurrentAndCurrentVelocityInterfaces)
{
  for (const auto &control_mode : {"current", "velocity", "position", "current_velocity"}) {
    SCOPED_TRACE(control_mode);
    auto parameters = make_parameters();
    auto &joint = parameters.hardware_info.joints[0];
    const auto interface =
      control_mode == std::string("current_velocity") ? "velocity" : control_mode;
    joint.command_interfaces[0].name = interface;
    joint.state_interfaces[0].name = interface;
    joint.parameters["control_mode"] = control_mode;
    if (control_mode == std::string("current_velocity")) {
      joint.parameters["startup_current_a"] = "2.0";
      joint.parameters["rpm_control_threshold_rpm"] = "50.0";
    }
    VescSystem hardware;
    EXPECT_EQ(hardware.init(parameters), hardware_interface::CallbackReturn::SUCCESS);
  }
}

TEST_F(SystemConfigurationTest, RejectsMismatchedOrIncompleteHybridSettings)
{
  auto parameters = make_parameters();
  parameters.hardware_info.joints[0].parameters["control_mode"] = "current";
  VescSystem wrong_interface;
  EXPECT_EQ(wrong_interface.init(parameters), hardware_interface::CallbackReturn::ERROR);
  parameters.hardware_info.joints[0].parameters["control_mode"] = "current_velocity";
  VescSystem no_startup;
  EXPECT_EQ(no_startup.init(parameters), hardware_interface::CallbackReturn::ERROR);
  parameters.hardware_info.joints[0].parameters["startup_current_a"] = "2.0";
  parameters.hardware_info.joints[0].parameters["max_current_a"] = "1.0";
  VescSystem excess_current;
  EXPECT_EQ(excess_current.init(parameters), hardware_interface::CallbackReturn::ERROR);
}

}  // namespace
}  // namespace vesc_can_ros2_control
