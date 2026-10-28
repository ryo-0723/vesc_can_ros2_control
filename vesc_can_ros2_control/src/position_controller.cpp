#include "vesc_can_ros2_control/position_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace vesc_can_ros2_control
{
PositionController::PositionController(PositionControlConfig config) : config_(config)
{
  const auto &c = config_;
  for (double value : std::array{c.kp, c.ki, c.kd, c.integral_limit_a, c.max_current_a,
         c.min_position, c.max_position, c.max_velocity, c.max_acceleration, c.max_period_s}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Position PID settings must be finite");
    }
  }
  if (c.kp <= 0.0 || c.ki < 0.0 || c.kd < 0.0 || c.integral_limit_a < 0.0 ||
      (c.ki > 0.0 && c.integral_limit_a <= 0.0) || c.max_current_a <= 0.0 ||
      c.integral_limit_a > c.max_current_a || c.min_position >= c.max_position ||
      c.max_velocity <= 0.0 || c.max_acceleration <= 0.0 || c.max_period_s <= 0.0 ||
      !std::isfinite(c.max_position - c.min_position) ||
      !std::isfinite(c.max_velocity * c.max_velocity / c.max_acceleration)) {
    throw std::invalid_argument("Invalid position PID gains, limits or period");
  }
}

bool PositionController::feedback_valid(double position, double velocity) const
{
  return std::isfinite(position) && std::isfinite(velocity) && position >= config_.min_position &&
         position <= config_.max_position && std::abs(velocity) <= config_.max_velocity;
}

void PositionController::clear()
{
  initialized_ = false;
  reference_position_ = 0.0;
  reference_velocity_ = 0.0;
  integral_current_ = 0.0;
}

bool PositionController::reset(double position, double velocity)
{
  clear();
  if (!feedback_valid(position, velocity)) {
    return false;
  }
  const double stop_position =
    position + velocity * std::abs(velocity) / (2 * config_.max_acceleration);
  if (!std::isfinite(stop_position) || stop_position < config_.min_position ||
      stop_position > config_.max_position) {
    return false;
  }
  reference_position_ = position;
  reference_velocity_ = velocity;
  initialized_ = true;
  return true;
}

void PositionController::advance_reference(double target, double period_s)
{
  const double a = config_.max_acceleration;
  const auto integrate = [this](double acceleration, double duration) {
    reference_position_ +=
      reference_velocity_ * duration + 0.5 * acceleration * duration * duration;
    reference_velocity_ += acceleration * duration;
  };
  double distance = target - reference_position_;
  double direction = distance < 0.0 ? -1.0 : 1.0;
  const double speed = direction * reference_velocity_;
  const double stopping_distance = reference_velocity_ * reference_velocity_ / (2 * a);
  // A nearby/reversed target may be inside the braking distance. Brake first,
  // without snapping velocity or position, then plan the remaining move from rest.
  if (speed < 0.0 || stopping_distance > std::abs(distance)) {
    const double brake_time = std::abs(reference_velocity_) / a;
    const double duration = std::min(period_s, brake_time);
    if (duration > 0.0) {
      integrate(-std::copysign(a, reference_velocity_), duration);
    }
    if (period_s < brake_time) {
      return;
    }
    reference_velocity_ = 0.0;
    period_s -= brake_time;
    distance = target - reference_position_;
    direction = distance < 0.0 ? -1.0 : 1.0;
  }
  const double initial_speed = direction * reference_velocity_;
  const double peak_speed = std::min(
    config_.max_velocity, std::sqrt(a * std::abs(distance) + 0.5 * initial_speed * initial_speed));
  const double acceleration_time = std::max(0.0, (peak_speed - initial_speed) / a);
  const double cruise_distance = std::max(
    0.0, std::abs(distance) - (peak_speed * peak_speed - 0.5 * initial_speed * initial_speed) / a);
  const double cruise_time = peak_speed > 0.0 ? cruise_distance / peak_speed : 0.0;
  const double deceleration_time = peak_speed / a;
  for (const auto &[duration, acceleration] :
    std::array{std::pair{acceleration_time, direction * a}, std::pair{cruise_time, 0.0},
      std::pair{deceleration_time, -direction * a}}) {
    if (period_s < duration) {
      integrate(acceleration, period_s);
      return;
    }
    integrate(acceleration, duration);
    period_s -= duration;
  }
  reference_position_ = target;
  reference_velocity_ = 0.0;
}

bool PositionController::update(
  double target, double position, double velocity, double period_s, double &current_a)
{
  current_a = 0.0;
  if (!initialized_ || !feedback_valid(position, velocity) || !std::isfinite(target) ||
      target < config_.min_position || target > config_.max_position || !std::isfinite(period_s) ||
      period_s <= 0.0 || period_s > config_.max_period_s) {
    clear();
    return false;
  }
  advance_reference(target, period_s);
  if (!std::isfinite(reference_position_) || !std::isfinite(reference_velocity_) ||
      reference_position_ < config_.min_position || reference_position_ > config_.max_position) {
    clear();
    return false;
  }
  const double error = reference_position_ - position;
  const double increment = config_.ki * error * period_s;
  const double pd = config_.kp * error + config_.kd * (reference_velocity_ - velocity);
  if (!std::isfinite(increment) || !std::isfinite(pd)) {
    clear();
    return false;
  }
  const double candidate =
    std::clamp(integral_current_ + increment, -config_.integral_limit_a, config_.integral_limit_a);
  const double unconstrained = pd + candidate;
  if (!std::isfinite(unconstrained)) {
    clear();
    return false;
  }
  // Integrate only when it does not push an already saturated output further out.
  if (!((unconstrained > config_.max_current_a && increment > 0.0) ||
        (unconstrained < -config_.max_current_a && increment < 0.0))) {
    integral_current_ = candidate;
  }
  current_a = std::clamp(pd + integral_current_, -config_.max_current_a, config_.max_current_a);
  return true;
}
}  // namespace vesc_can_ros2_control
