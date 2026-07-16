#ifndef CONTROLLER__CONTROL_CORE_HPP_
#define CONTROLLER__CONTROL_CORE_HPP_

#include <cstddef>
#include <vector>

namespace controller
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct LongitudinalCommand
{
  double throttle{0.0};
  double brake{0.0};
};

struct PurePursuitConfig
{
  double wheelbase{2.63};
  double maximum_steering_angle{0.52};
  double steering_sign{1.0};
  bool normalize_steering{true};
};

struct SpeedControllerConfig
{
  double kp{0.35};
  double ki{0.05};
  double kd{0.02};
  double integral_limit{2.0};
  double error_deadband{0.05};
  double maximum_throttle{0.35};
  double maximum_brake{1.0};
};

double clamp_value(double value, double minimum, double maximum);
double normalize_angle(double angle);

Point2D world_to_vehicle(
  const Point2D & world_point,
  const Point2D & vehicle_position,
  double vehicle_yaw);

std::size_t find_nearest_point(
  const std::vector<Point2D> & path,
  const Point2D & vehicle_position);

bool find_lookahead_point(
  const std::vector<Point2D> & path,
  std::size_t nearest_index,
  double lookahead_distance,
  const Point2D & vehicle_position,
  double vehicle_yaw,
  double minimum_forward_distance,
  Point2D & target_in_vehicle_frame);

double pure_pursuit_steering(
  const Point2D & target_in_vehicle_frame,
  const PurePursuitConfig & config);

class SpeedController
{
public:
  explicit SpeedController(const SpeedControllerConfig & config);

  LongitudinalCommand update(double target_speed, double current_speed, double dt);
  void reset();

private:
  SpeedControllerConfig config_;
  double integral_{0.0};
  double previous_error_{0.0};
  bool has_previous_error_{false};
};

}  // namespace controller

#endif  // CONTROLLER__CONTROL_CORE_HPP_
