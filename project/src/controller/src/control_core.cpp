// Copyright 2026 Team 20

#include "controller/control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace controller
{

double clamp_value(const double value, const double minimum, const double maximum)
{
  return std::clamp(value, minimum, maximum);
}

double normalize_angle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {
    angle -= 2.0 * pi;
  }
  while (angle < -pi) {
    angle += 2.0 * pi;
  }
  return angle;
}

Point2D world_to_vehicle(
  const Point2D & world_point,
  const Point2D & vehicle_position,
  const double vehicle_yaw)
{
  const double dx = world_point.x - vehicle_position.x;
  const double dy = world_point.y - vehicle_position.y;
  const double cosine = std::cos(vehicle_yaw);
  const double sine = std::sin(vehicle_yaw);

  return Point2D{
    cosine * dx + sine * dy,
    -sine * dx + cosine * dy};
}

std::size_t find_nearest_point(
  const std::vector<Point2D> & path,
  const Point2D & vehicle_position)
{
  std::size_t nearest_index = 0;
  double nearest_squared_distance = std::numeric_limits<double>::max();

  for (std::size_t index = 0; index < path.size(); ++index) {
    const double dx = path[index].x - vehicle_position.x;
    const double dy = path[index].y - vehicle_position.y;
    const double squared_distance = dx * dx + dy * dy;
    if (squared_distance < nearest_squared_distance) {
      nearest_squared_distance = squared_distance;
      nearest_index = index;
    }
  }

  return nearest_index;
}

bool find_lookahead_point(
  const std::vector<Point2D> & path,
  const std::size_t nearest_index,
  const double lookahead_distance,
  const Point2D & vehicle_position,
  const double vehicle_yaw,
  const double minimum_forward_distance,
  Point2D & target_in_vehicle_frame)
{
  if (path.empty() || nearest_index >= path.size()) {
    return false;
  }

  std::size_t candidate_index = nearest_index;
  double accumulated_distance = 0.0;
  while (candidate_index + 1 < path.size() && accumulated_distance < lookahead_distance) {
    const Point2D & current = path[candidate_index];
    const Point2D & next = path[candidate_index + 1];
    accumulated_distance += std::hypot(next.x - current.x, next.y - current.y);
    ++candidate_index;
  }

  for (std::size_t index = candidate_index; index < path.size(); ++index) {
    const Point2D candidate = world_to_vehicle(path[index], vehicle_position, vehicle_yaw);
    if (candidate.x >= minimum_forward_distance) {
      target_in_vehicle_frame = candidate;
      return true;
    }
  }

  return false;
}

double pure_pursuit_steering(
  const Point2D & target,
  const PurePursuitConfig & config)
{
  const double squared_distance = target.x * target.x + target.y * target.y;
  if (squared_distance < 1.0e-9 || target.x <= 0.0) {
    return 0.0;
  }

  const double wheelbase = std::max(config.wheelbase, 0.01);
  const double maximum_angle = std::max(config.maximum_steering_angle, 0.01);
  const double steering_angle = std::atan2(2.0 * wheelbase * target.y, squared_distance);
  double command = clamp_value(steering_angle, -maximum_angle, maximum_angle);
  if (config.normalize_steering) {
    command /= maximum_angle;
  }
  command *= config.steering_sign >= 0.0 ? 1.0 : -1.0;
  return clamp_value(command, -1.0, 1.0);
}

SpeedController::SpeedController(const SpeedControllerConfig & config)
: config_(config)
{
  config_.kp = std::max(config_.kp, 0.0);
  config_.ki = std::max(config_.ki, 0.0);
  config_.kd = std::max(config_.kd, 0.0);
  config_.integral_limit = std::max(config_.integral_limit, 0.0);
  config_.error_deadband = std::max(config_.error_deadband, 0.0);
  config_.maximum_throttle = clamp_value(config_.maximum_throttle, 0.0, 1.0);
  config_.maximum_brake = clamp_value(config_.maximum_brake, 0.0, 1.0);
}

LongitudinalCommand SpeedController::update(
  const double target_speed,
  const double current_speed,
  const double dt)
{
  if (!std::isfinite(target_speed) || !std::isfinite(current_speed) ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    reset();
    return LongitudinalCommand{0.0, config_.maximum_brake};
  }

  double error = std::max(target_speed, 0.0) - std::max(current_speed, 0.0);
  if (std::abs(error) < config_.error_deadband) {
    error = 0.0;
  }

  const double candidate_integral = clamp_value(
    integral_ + error * dt, -config_.integral_limit, config_.integral_limit);
  // Differentiate the measured speed instead of the tracking error. This is
  // the standard derivative-on-measurement form: it retains damping while
  // preventing a target-speed change (for example CRUISE -> FOLLOWING) from
  // producing a one-cycle full-brake "derivative kick".
  const double sanitized_speed = std::max(current_speed, 0.0);
  const double derivative =
    has_previous_speed_ ? -(sanitized_speed - previous_speed_) / dt : 0.0;
  const double unsaturated =
    config_.kp * error + config_.ki * candidate_integral + config_.kd * derivative;
  const double saturated = clamp_value(
    unsaturated, -config_.maximum_brake, config_.maximum_throttle);

  const bool is_saturated = std::abs(unsaturated - saturated) > 1.0e-9;
  const bool above_throttle_limit = unsaturated > config_.maximum_throttle;
  const bool below_brake_limit = unsaturated < -config_.maximum_brake;
  const bool moves_out_of_saturation =
    (above_throttle_limit && error < 0.0) ||
    (below_brake_limit && error > 0.0);
  if (!is_saturated || moves_out_of_saturation) {
    integral_ = candidate_integral;
  }

  previous_speed_ = sanitized_speed;
  has_previous_speed_ = true;

  if (saturated >= 0.0) {
    return LongitudinalCommand{saturated, 0.0};
  }
  return LongitudinalCommand{0.0, -saturated};
}

void SpeedController::reset()
{
  integral_ = 0.0;
  previous_speed_ = 0.0;
  has_previous_speed_ = false;
}

}  // namespace controller
