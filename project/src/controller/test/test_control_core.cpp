#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "controller/control_core.hpp"

namespace
{

using controller::Point2D;

TEST(ControlCore, TransformsWorldPointIntoVehicleFrame)
{
  constexpr double half_pi = 1.57079632679489661923;
  const Point2D result = controller::world_to_vehicle(
    Point2D{0.0, 5.0}, Point2D{0.0, 0.0}, half_pi);
  EXPECT_NEAR(result.x, 5.0, 1.0e-9);
  EXPECT_NEAR(result.y, 0.0, 1.0e-9);
}

TEST(ControlCore, FindsForwardLookaheadAlongPath)
{
  const std::vector<Point2D> path{{-1.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}, {4.0, 1.0}};
  Point2D target;
  ASSERT_TRUE(controller::find_lookahead_point(
      path, 1, 3.0, Point2D{0.0, 0.0}, 0.0, 0.2, target));
  EXPECT_NEAR(target.x, 4.0, 1.0e-9);
  EXPECT_NEAR(target.y, 1.0, 1.0e-9);
}

TEST(ControlCore, RejectsPathWithoutForwardPoint)
{
  const std::vector<Point2D> path{{-2.0, 0.0}, {-1.0, 0.0}};
  Point2D target;
  EXPECT_FALSE(controller::find_lookahead_point(
      path, 1, 2.0, Point2D{0.0, 0.0}, 0.0, 0.2, target));
}

TEST(ControlCore, PurePursuitIsZeroOnStraightPath)
{
  controller::PurePursuitConfig config;
  EXPECT_NEAR(controller::pure_pursuit_steering(Point2D{5.0, 0.0}, config), 0.0, 1.0e-9);
}

TEST(ControlCore, PurePursuitHasCorrectTurnSigns)
{
  controller::PurePursuitConfig config;
  EXPECT_GT(controller::pure_pursuit_steering(Point2D{5.0, 2.0}, config), 0.0);
  EXPECT_LT(controller::pure_pursuit_steering(Point2D{5.0, -2.0}, config), 0.0);
}

TEST(ControlCore, SteeringSignCanBeReversed)
{
  controller::PurePursuitConfig config;
  config.steering_sign = -1.0;
  EXPECT_LT(controller::pure_pursuit_steering(Point2D{5.0, 2.0}, config), 0.0);
}

TEST(ControlCore, SpeedControllerUsesThrottleBelowTarget)
{
  controller::SpeedController controller(controller::SpeedControllerConfig{});
  const auto command = controller.update(4.0, 1.0, 0.1);
  EXPECT_DOUBLE_EQ(command.throttle, 0.7);
  EXPECT_DOUBLE_EQ(command.brake, 0.0);
}

TEST(ControlCore, SpeedControllerUsesBrakeAboveTarget)
{
  controller::SpeedController controller(controller::SpeedControllerConfig{});
  const auto command = controller.update(1.0, 4.0, 0.1);
  EXPECT_DOUBLE_EQ(command.throttle, 0.0);
  EXPECT_GT(command.brake, 0.0);
}

TEST(ControlCore, TargetStepDoesNotCreateDerivativeBrakeKick)
{
  controller::SpeedControllerConfig config;
  config.kp = 0.35;
  config.ki = 0.0;
  config.kd = 0.02;
  controller::SpeedController controller(config);

  controller.update(14.0, 6.4, 0.05);
  const auto command = controller.update(6.4, 6.4, 0.05);

  EXPECT_DOUBLE_EQ(command.throttle, 0.0);
  EXPECT_DOUBLE_EQ(command.brake, 0.0);
}

TEST(ControlCore, SpeedControllerNeverCommandsThrottleAndBrakeTogether)
{
  controller::SpeedController controller(controller::SpeedControllerConfig{});
  for (double speed = 0.0; speed <= 8.0; speed += 0.25) {
    const auto command = controller.update(4.0, speed, 0.05);
    EXPECT_FALSE(command.throttle > 0.0 && command.brake > 0.0);
  }
}

TEST(ControlCore, InvalidInputProducesFullBrake)
{
  controller::SpeedController controller(controller::SpeedControllerConfig{});
  const auto command = controller.update(NAN, 1.0, 0.1);
  EXPECT_DOUBLE_EQ(command.throttle, 0.0);
  EXPECT_DOUBLE_EQ(command.brake, 1.0);
}

}  // namespace
