#pragma once

namespace vesc_can_ros2_control
{
struct PositionControlConfig
{
  double kp{};  // A/m
  double ki{};  // A/(m s)
  double kd{};  // A s/m
  double integral_limit_a{};
  double max_current_a{};
  double min_position{};      // m
  double max_position{};      // m
  double max_velocity{};      // m/s: reference and measured-speed trip threshold
  double max_acceleration{};  // m/s^2: reference only
  double max_period_s{0.1};
};

/// @brief 加減速する参照位置を PID で追従し、関節方向の電流 [A] を作る
/// @note 異常後は reset が成功するまで出力しない。動的メモリ確保は行わない。
class PositionController
{
public:
  explicit PositionController(PositionControlConfig config);
  bool feedback_valid(double position, double velocity) const;
  bool reset(double position, double velocity);
  void clear();
  /// @return 指令・状態・周期が不正、または数値異常なら false（電流出力は 0）
  bool update(double target, double position, double velocity, double period_s, double &current_a);
  double reference_position() const
  {
    return reference_position_;
  }
  double reference_velocity() const
  {
    return reference_velocity_;
  }
  double integral_current() const
  {
    return integral_current_;
  }

private:
  void advance_reference(double target, double period_s);
  PositionControlConfig config_;
  double reference_position_{};
  double reference_velocity_{};
  double integral_current_{};
  bool initialized_{false};
};
}  // namespace vesc_can_ros2_control
