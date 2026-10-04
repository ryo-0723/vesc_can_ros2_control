#pragma once

#include <memory>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "vesc_can_ros2_control/can_transport.hpp"

namespace vesc_can_ros2_control
{
class VescSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  enum class StateField {POSITION, VELOCITY, CURRENT, TEMP_FET, TEMP_MOTOR, VOLTAGE, DUTY};
  struct StateHandle
  {
    std::size_t motor{};
    StateField field{};
    hardware_interface::StateInterface::SharedPtr handle;
  };
  std::vector<MotorConfig> configs_;
  std::vector<MotorState> states_;
  std::vector<double> commands_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles_;
  std::vector<StateHandle> state_handles_;
  TransportOptions options_;
  std::chrono::milliseconds activation_timeout_{2000};
  rclcpp::Node::SharedPtr io_node_;
  std::unique_ptr<CanTransport> transport_;
  bool active_{false};
};
}  // namespace vesc_can_ros2_control
