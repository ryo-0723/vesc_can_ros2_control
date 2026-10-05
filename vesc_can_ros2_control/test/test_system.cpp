#include <memory>

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
}  // namespace
}  // namespace vesc_can_ros2_control
