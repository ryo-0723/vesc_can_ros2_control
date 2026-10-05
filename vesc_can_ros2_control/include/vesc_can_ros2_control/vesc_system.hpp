#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "vesc_can_ros2_control/can_transport.hpp"

namespace vesc_can_ros2_control
{
class VescSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  enum class StateField
  {
    Position,
    Velocity,
    Current,
    TemperatureFet,
    TemperatureMotor,
    InputVoltage,
    DutyCycle
  };
  struct StateHandle
  {
    std::size_t motor_index{};
    StateField field{};
    hardware_interface::StateInterface::SharedPtr handle;
  };
  static StateField state_field_from_name(const std::string &interface_name);
  static MotorConfig load_joint_config(const hardware_interface::ComponentInfo &joint);
  void cache_interface_handles();
  void release_can_transport();
  double read_state_value(const StateHandle &state_handle, TimePoint now) const;

  std::vector<MotorConfig> motor_configs_;
  std::vector<MotorState> motor_states_;
  std::vector<double> motor_commands_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_handles_;
  std::vector<StateHandle> state_handles_;
  TransportOptions transport_options_;
  std::chrono::milliseconds activation_timeout_{2000};
  std::unique_ptr<CanTransport> can_transport_;
  bool active_{false};
};
}  // namespace vesc_can_ros2_control
