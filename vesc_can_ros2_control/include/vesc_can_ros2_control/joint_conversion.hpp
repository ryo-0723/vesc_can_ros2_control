#pragma once

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace vesc_can_ros2_control
{
/// @brief 減速比・方向を適用済みの回転量と joint の SI 単位を相互変換する
class JointConversion
{
public:
  JointConversion() = default;  // Rotary joints retain rad and rad/s.

  /// @param lead_m_per_rev 減速機の出力軸 1 回転あたりの直動距離 [m]
  /// @param position_offset_m 積算カウント 0 に対応する直動位置 [m]
  JointConversion(double lead_m_per_rev, double position_offset_m)
    : scale_(lead_m_per_rev / (2.0 * std::numbers::pi_v<double>)), offset_(position_offset_m)
  {
    if (!std::isfinite(scale_) || scale_ <= 0.0 || !std::isfinite(1.0 / scale_) ||
        !std::isfinite(offset_)) {
      throw std::invalid_argument("Invalid linear lead or position offset");
    }
  }

  double position_from_rotary(double angle) const
  {
    return angle * scale_ + offset_;
  }
  double velocity_from_rotary(double velocity) const
  {
    return velocity * scale_;
  }
  double velocity_to_rotary(double velocity) const
  {
    return velocity / scale_;
  }
  double position_to_rotary(double position) const
  {
    return (position - offset_) / scale_;
  }

private:
  double scale_{1.0};
  double offset_{0.0};
};
}  // namespace vesc_can_ros2_control
