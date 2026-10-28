#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "vesc_can_ros2_control/position_controller.hpp"

namespace vesc_can_ros2_control
{
namespace
{
PositionControlConfig configuration()
{
  PositionControlConfig c;
  c.kp = 1000;
  c.ki = 200;
  c.kd = 150;
  c.integral_limit_a = 1;
  c.max_current_a = 2;
  c.min_position = -1;
  c.max_position = 1;
  c.max_velocity = 0.1;
  c.max_acceleration = 0.02;
  return c;
}

TEST(PositionControllerTest, ProfileBoundsSurviveVariablePeriodsAndTargetReversals)
{
  auto c = configuration();
  c.max_velocity = 0.2;
  c.max_acceleration = 0.4;
  PositionController controller(c);
  ASSERT_TRUE(controller.reset(0, 0));
  double current = 0;
  double target = 0.8;
  for (int step = 0; step < 3000; ++step) {
    if (step == 25)
      target = -0.8;
    if (step == 50)
      target = controller.reference_position();  // Inside braking distance.
    if (step == 85)
      target = 1.0;
    if (step == 550)
      target = -1.0;
    if (step == 1250)
      target = 0.00001;
    const double dt = step % 3 == 0 ? 0.003 : (step % 3 == 1 ? 0.017 : 0.041);
    const double position = controller.reference_position();
    const double velocity = controller.reference_velocity();
    ASSERT_TRUE(controller.update(target, 0, 0, dt, current)) << step;
    EXPECT_LE(std::abs(controller.reference_velocity()), c.max_velocity + 1e-12);
    EXPECT_LE(
      std::abs(controller.reference_velocity() - velocity), c.max_acceleration * dt + 1e-12);
    EXPECT_LE(std::abs(controller.reference_position() - position), c.max_velocity * dt + 1e-12);
    EXPECT_LE(std::abs(current), c.max_current_a);
  }
  EXPECT_DOUBLE_EQ(controller.reference_position(), target);
  EXPECT_DOUBLE_EQ(controller.reference_velocity(), 0.0);
}

TEST(PositionControllerTest, StartsFromMeasuredMotionAndResetsWithoutOldIntegral)
{
  PositionController controller(configuration());
  ASSERT_TRUE(controller.reset(0.1, 0.03));
  EXPECT_DOUBLE_EQ(controller.reference_position(), 0.1);
  EXPECT_DOUBLE_EQ(controller.reference_velocity(), 0.03);
  double current;
  ASSERT_TRUE(controller.update(0.1, 0.1, 0.03, 0.01, current));
  EXPECT_NEAR(controller.reference_velocity(), 0.0298, 1e-12);
  controller.clear();
  EXPECT_FALSE(controller.update(0.1, 0.1, 0, 0.01, current));
  EXPECT_EQ(current, 0);
  ASSERT_TRUE(controller.reset(-0.2, 0));
  EXPECT_EQ(controller.integral_current(), 0);
  ASSERT_TRUE(controller.update(-0.2, -0.2, 0, 0.01, current));
  EXPECT_EQ(current, 0);
  EXPECT_FALSE(controller.reset(1.0, 0.01));  // Cannot stop inside the travel range.
}

TEST(PositionControllerTest, SaturationDoesNotWindUpAndIntegralHoldsAgainstAStaticLoad)
{
  auto c = configuration();
  c.kp = 100;
  c.ki = 100;
  c.kd = 0;
  c.max_current_a = 1;
  c.integral_limit_a = 0.2;
  PositionController controller(c);
  ASSERT_TRUE(controller.reset(0, 0));
  double current;
  for (int step = 0; step < 1000; ++step) {
    ASSERT_TRUE(controller.update(0.1, 0, 0, 0.01, current));
    EXPECT_LE(std::abs(current), 1.0);
    EXPECT_LE(std::abs(controller.integral_current()), 0.2);
  }
  EXPECT_DOUBLE_EQ(current, 1.0);
  const double previous_integral = controller.integral_current();
  ASSERT_TRUE(controller.update(0.1, 0, 0, 0.01, current));
  EXPECT_DOUBLE_EQ(controller.integral_current(), previous_integral);
  ASSERT_TRUE(controller.update(0.1, 0.099, 0, 0.01, current));
  EXPECT_GE(controller.integral_current(), previous_integral);
  ASSERT_TRUE(controller.update(0.1, 0.101, 0, 0.01, current));
  EXPECT_LT(controller.integral_current(), previous_integral);
  for (int step = 0; step < 1000; ++step) {
    ASSERT_TRUE(controller.update(0.1, 0.099, 0, 0.01, current));
  }
  EXPECT_DOUBLE_EQ(controller.integral_current(), 0.2);
  // A stationary target continues producing holding current; it is not a disable command.
  ASSERT_TRUE(controller.update(0.1, 0.1, 0, 0.01, current));
  EXPECT_DOUBLE_EQ(current, 0.2);
}

TEST(PositionControllerTest, ClosedLoopConvergesAndHoldsWithDisturbance)
{
  PositionController controller(configuration());
  ASSERT_TRUE(controller.reset(0, 0));
  double position = 0;
  double velocity = 0;
  double current;
  constexpr double dt = 0.001;
  // Deterministic current-driven mass with viscous drag and a constant external load.
  for (int step = 0; step < 60000; ++step) {
    const double target = step < 30000 ? 0.02 : -0.01;
    ASSERT_TRUE(controller.update(target, position, velocity, dt, current)) << step;
    const double acceleration = 0.1 * current - 2 * velocity - 0.02;
    position += velocity * dt + 0.5 * acceleration * dt * dt;
    velocity += acceleration * dt;
    if (step == 29999) {
      EXPECT_NEAR(position, 0.02, 1e-5);
      EXPECT_NEAR(velocity, 0, 1e-5);
    }
  }
  EXPECT_NEAR(position, -0.01, 1e-5);
  EXPECT_NEAR(velocity, 0, 1e-5);
  EXPECT_NEAR(current, 0.2, 0.005);
}

TEST(PositionControllerTest, InvalidFeedbackTargetAndPeriodLatchZeroUntilReset)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int scenario = 0; scenario < 10; ++scenario) {
    PositionController controller(configuration());
    ASSERT_TRUE(controller.reset(0, 0));
    double current = 10;
    const double target = scenario == 0 ? nan : (scenario == 1 ? 1.01 : 0.1);
    const double position = scenario == 2 ? nan : (scenario == 3 ? -1.01 : 0);
    const double velocity = scenario == 4 ? nan : (scenario == 5 ? 0.101 : 0);
    const double dt =
      scenario == 6
        ? 0
        : (scenario == 7 ? -0.01 : (scenario == 8 ? nan : (scenario == 9 ? 0.101 : 0.01)));
    EXPECT_FALSE(controller.update(target, position, velocity, dt, current));
    EXPECT_DOUBLE_EQ(current, 0);
    EXPECT_FALSE(controller.update(0.1, 0, 0, 0.01, current));
    ASSERT_TRUE(controller.reset(0, 0));
    EXPECT_TRUE(controller.update(0.1, 0, 0, 0.01, current));
  }
}

TEST(PositionControllerTest, RejectsInvalidGainsLimitsAndNonFiniteSettings)
{
  for (int scenario = 0; scenario < 10; ++scenario) {
    auto c = configuration();
    if (scenario == 0)
      c.kp = 0;
    if (scenario == 1)
      c.ki = -1;
    if (scenario == 2)
      c.kd = -1;
    if (scenario == 3)
      c.integral_limit_a = 0;
    if (scenario == 4)
      c.max_current_a = 0;
    if (scenario == 5)
      c.max_position = c.min_position;
    if (scenario == 6)
      c.max_velocity = 0;
    if (scenario == 7)
      c.max_acceleration = 0;
    if (scenario == 8)
      c.max_period_s = 0;
    if (scenario == 9)
      c.kp = std::numeric_limits<double>::infinity();
    EXPECT_THROW(PositionController{c}, std::invalid_argument);
  }
}
}  // namespace
}  // namespace vesc_can_ros2_control
