#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simulation/msg/vehicle_control.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }

  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }

  return angle;
}
}  // namespace

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_runtime_parameters();

    path_subscription_ =
      create_subscription<nav_msgs::msg::Path>(
      "/planning/path",
      10,
      std::bind(
        &ControllerNode::path_callback,
        this,
        std::placeholders::_1));

    target_speed_subscription_ =
      create_subscription<std_msgs::msg::Float32>(
      "/decision/target_speed",
      10,
      std::bind(
        &ControllerNode::target_speed_callback,
        this,
        std::placeholders::_1));

    stop_required_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
      "/decision/stop_required",
      10,
      std::bind(
        &ControllerNode::stop_required_callback,
        this,
        std::placeholders::_1));

    command_publisher_ =
      create_publisher<simulation::msg::VehicleControl>(
      "/car_command",
      10);

    control_timer_ =
      create_wall_timer(
      100ms,
      std::bind(
        &ControllerNode::control_loop,
        this));

    const auto now = std::chrono::steady_clock::now();
    last_control_time_ = now;
    last_parameter_refresh_time_ = now;

    RCLCPP_INFO(get_logger(), "Controller node started.");
    RCLCPP_INFO(get_logger(), "Path input: /planning/path");
    RCLCPP_INFO(get_logger(), "Speed input: /decision/target_speed");
    RCLCPP_INFO(get_logger(), "Stop input: /decision/stop_required");
    RCLCPP_INFO(get_logger(), "TF input: world -> OurCar/Center");
    RCLCPP_INFO(get_logger(), "Command output: /car_command");
    RCLCPP_INFO(get_logger(), "Wheelbase: %.2f m", wheelbase_);
    RCLCPP_INFO(
      get_logger(),
      "Safety speed limit: %.2f m/s",
      maximum_target_speed_);

    if (enable_motion_) {
      RCLCPP_WARN(
        get_logger(),
        "MOTION ENABLED: real commands will be published.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "DRY-RUN MODE: controls are calculated, but safe stop is published.");
    }
  }

private:
  void declare_parameters()
  {
    declare_parameter<bool>("enable_motion", false);

    declare_parameter<double>("wheelbase", 2.63);

    /*
     * The observed Center TF yaw is approximately zero while the
     * path initially points toward world -x. A default offset of pi
     * therefore aligns the controller's forward axis with the path.
     * Verify this in dry-run mode before enabling motion.
     */
    declare_parameter<double>("vehicle_yaw_offset", 0.0);

    declare_parameter<double>("lookahead_base", 4.0);
    declare_parameter<double>("lookahead_speed_gain", 0.5);
    declare_parameter<double>("lookahead_min", 3.0);
    declare_parameter<double>("lookahead_max", 8.0);
    declare_parameter<double>("minimum_forward_target_x", 0.20);

    declare_parameter<double>("maximum_target_speed", 2.0);
    declare_parameter<double>("hard_speed_limit", 2.0);
    declare_parameter<double>("goal_stop_distance", 1.0);

    declare_parameter<bool>("steering_is_normalized", true);
    declare_parameter<double>("steering_sign", 1.0);
    declare_parameter<double>("maximum_steering_angle", 0.52);
    declare_parameter<double>("maximum_steering_command", 1.0);
    declare_parameter<double>("maximum_steering_rate", 2.0);

    declare_parameter<double>("speed_kp", 0.35);
    declare_parameter<double>("speed_ki", 0.05);
    declare_parameter<double>("speed_kd", 0.02);
    declare_parameter<double>("speed_integral_limit", 2.0);
    declare_parameter<double>("speed_error_deadband", 0.05);
    declare_parameter<double>("maximum_throttle", 0.35);
    declare_parameter<double>("maximum_brake", 1.0);
    declare_parameter<double>("safe_stop_brake", 1.0);

    declare_parameter<double>("speed_filter_alpha", 0.30);
    declare_parameter<double>("maximum_pose_jump", 3.0);

    declare_parameter<double>("path_timeout", 3.0);
    declare_parameter<double>("decision_timeout", 1.0);
  }

  void load_runtime_parameters()
  {
    const bool previous_enable_motion = enable_motion_;

    get_parameter("enable_motion", enable_motion_);
    get_parameter("wheelbase", wheelbase_);
    get_parameter("vehicle_yaw_offset", vehicle_yaw_offset_);

    get_parameter("lookahead_base", lookahead_base_);
    get_parameter("lookahead_speed_gain", lookahead_speed_gain_);
    get_parameter("lookahead_min", lookahead_min_);
    get_parameter("lookahead_max", lookahead_max_);
    get_parameter("minimum_forward_target_x", minimum_forward_target_x_);

    get_parameter("maximum_target_speed", maximum_target_speed_);
    get_parameter("hard_speed_limit", hard_speed_limit_);
    get_parameter("goal_stop_distance", goal_stop_distance_);

    get_parameter("steering_is_normalized", steering_is_normalized_);
    get_parameter("steering_sign", steering_sign_);
    get_parameter("maximum_steering_angle", maximum_steering_angle_);
    get_parameter("maximum_steering_command", maximum_steering_command_);
    get_parameter("maximum_steering_rate", maximum_steering_rate_);

    get_parameter("speed_kp", speed_kp_);
    get_parameter("speed_ki", speed_ki_);
    get_parameter("speed_kd", speed_kd_);
    get_parameter("speed_integral_limit", speed_integral_limit_);
    get_parameter("speed_error_deadband", speed_error_deadband_);
    get_parameter("maximum_throttle", maximum_throttle_);
    get_parameter("maximum_brake", maximum_brake_);
    get_parameter("safe_stop_brake", safe_stop_brake_);

    get_parameter("speed_filter_alpha", speed_filter_alpha_);
    get_parameter("maximum_pose_jump", maximum_pose_jump_);

    get_parameter("path_timeout", path_timeout_);
    get_parameter("decision_timeout", decision_timeout_);

    sanitize_parameters();

    if (enable_motion_ != previous_enable_motion) {
      reset_controller_state();

      if (enable_motion_) {
        RCLCPP_WARN(
          get_logger(),
          "enable_motion changed to true. Real vehicle commands are active.");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "enable_motion changed to false. Safe stop is active.");
      }
    }
  }

  void sanitize_parameters()
  {
    wheelbase_ = std::max(wheelbase_, 0.10);

    lookahead_min_ = std::max(lookahead_min_, 0.50);
    lookahead_max_ = std::max(lookahead_max_, lookahead_min_);
    lookahead_base_ =
      std::clamp(lookahead_base_, lookahead_min_, lookahead_max_);
    lookahead_speed_gain_ = std::max(lookahead_speed_gain_, 0.0);
    minimum_forward_target_x_ = std::max(minimum_forward_target_x_, 0.0);

    hard_speed_limit_ =
      std::clamp(hard_speed_limit_, 0.0, 20.0);
    maximum_target_speed_ =
      std::clamp(maximum_target_speed_, 0.0, hard_speed_limit_);
    goal_stop_distance_ = std::max(goal_stop_distance_, 0.10);

    steering_sign_ = steering_sign_ >= 0.0 ? 1.0 : -1.0;
    maximum_steering_angle_ = std::max(maximum_steering_angle_, 0.01);
    maximum_steering_command_ = std::max(maximum_steering_command_, 0.01);
    maximum_steering_rate_ = std::max(maximum_steering_rate_, 0.01);

    speed_kp_ = std::max(speed_kp_, 0.0);
    speed_ki_ = std::max(speed_ki_, 0.0);
    speed_kd_ = std::max(speed_kd_, 0.0);
    speed_integral_limit_ = std::max(speed_integral_limit_, 0.0);
    speed_error_deadband_ = std::max(speed_error_deadband_, 0.0);
    maximum_throttle_ = std::clamp(maximum_throttle_, 0.0, 1.0);
    maximum_brake_ = std::clamp(maximum_brake_, 0.0, 1.0);
    safe_stop_brake_ = std::clamp(safe_stop_brake_, 0.0, 1.0);

    speed_filter_alpha_ = std::clamp(speed_filter_alpha_, 0.0, 1.0);
    maximum_pose_jump_ = std::max(maximum_pose_jump_, 0.10);

    path_timeout_ = std::max(path_timeout_, 0.10);
    decision_timeout_ = std::max(decision_timeout_, 0.10);
  }

  void refresh_parameters_if_needed(
    const std::chrono::steady_clock::time_point & now)
  {
    const double elapsed =
      std::chrono::duration<double>(
      now - last_parameter_refresh_time_).count();

    if (elapsed >= 0.5) {
      load_runtime_parameters();
      last_parameter_refresh_time_ = now;
    }
  }

  void path_callback(
    const nav_msgs::msg::Path::SharedPtr msg)
  {
    latest_path_ = *msg;
    has_path_ = !msg->poses.empty();
    last_path_time_ = std::chrono::steady_clock::now();

    if (msg->header.frame_id != "world") {
      has_path_ = false;

      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Rejected /planning/path in frame '%s'; expected 'world'.",
        msg->header.frame_id.c_str());

      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received path with %zu poses, frame: %s",
      msg->poses.size(),
      msg->header.frame_id.c_str());
  }

  void target_speed_callback(
    const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Rejected non-finite /decision/target_speed.");
      return;
    }

    target_speed_ = msg->data;
    has_target_speed_ = true;
    last_target_speed_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received decision target speed: %.2f m/s",
      target_speed_);
  }

  void stop_required_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    stop_required_ = msg->data;
    has_stop_required_ = true;
    last_stop_required_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received decision stop request: %s",
      stop_required_ ? "true" : "false");
  }

  bool update_vehicle_pose_and_speed(
    const std::chrono::steady_clock::time_point & now)
  {
    try {
      const auto transform =
        tf_buffer_.lookupTransform(
        "world",
        "OurCar/Center",
        tf2::TimePointZero);

      const double new_x =
        transform.transform.translation.x;
      const double new_y =
        transform.transform.translation.y;

      tf2::Quaternion quaternion(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);

      double roll = 0.0;
      double pitch = 0.0;
      double raw_yaw = 0.0;

      tf2::Matrix3x3(quaternion).getRPY(
        roll,
        pitch,
        raw_yaw);

      if (
        !std::isfinite(new_x) ||
        !std::isfinite(new_y) ||
        !std::isfinite(raw_yaw))
      {
        has_tf_ = false;
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          3000,
          "Rejected non-finite vehicle TF.");
        return false;
      }

      const double new_heading =
        normalize_angle(raw_yaw + vehicle_yaw_offset_);

      if (has_previous_pose_) {
        const double dt =
          std::chrono::duration<double>(
          now - previous_pose_time_).count();

        const double dx = new_x - previous_vehicle_x_;
        const double dy = new_y - previous_vehicle_y_;
        const double displacement = std::hypot(dx, dy);

        if (displacement > maximum_pose_jump_) {
          has_speed_estimate_ = false;
          current_speed_ = 0.0;

          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            3000,
            "Pose jump %.2f m detected; speed estimate reset.",
            displacement);
        } else if (dt > 1.0e-4 && dt < 1.0) {
          const double forward_displacement =
            dx * std::cos(new_heading) +
            dy * std::sin(new_heading);

          const double raw_speed =
            forward_displacement / dt;

          if (std::isfinite(raw_speed)) {
            if (!has_speed_estimate_) {
              current_speed_ = raw_speed;
            } else {
              current_speed_ =
                speed_filter_alpha_ * raw_speed +
                (1.0 - speed_filter_alpha_) * current_speed_;
            }

            has_speed_estimate_ = true;
          }
        }
      }

      previous_vehicle_x_ = new_x;
      previous_vehicle_y_ = new_y;
      previous_pose_time_ = now;
      has_previous_pose_ = true;

      vehicle_x_ = new_x;
      vehicle_y_ = new_y;
      raw_vehicle_yaw_ = raw_yaw;
      vehicle_heading_ = new_heading;
      has_tf_ = true;

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Pose x=%.2f y=%.2f raw_yaw=%.2f heading=%.2f speed=%.2f m/s",
        vehicle_x_,
        vehicle_y_,
        raw_vehicle_yaw_,
        vehicle_heading_,
        current_speed_);

      return true;
    }
    catch (const tf2::TransformException & exception) {
      has_tf_ = false;
      has_speed_estimate_ = false;

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Waiting for TF world -> OurCar/Center: %s",
        exception.what());

      return false;
    }
  }

  std::size_t find_nearest_path_index() const
  {
    std::size_t nearest_index = 0;
    double minimum_squared_distance =
      std::numeric_limits<double>::max();

    for (
      std::size_t i = 0;
      i < latest_path_.poses.size();
      ++i)
    {
      const double dx =
        latest_path_.poses[i].pose.position.x - vehicle_x_;
      const double dy =
        latest_path_.poses[i].pose.position.y - vehicle_y_;
      const double squared_distance = dx * dx + dy * dy;

      if (squared_distance < minimum_squared_distance) {
        minimum_squared_distance = squared_distance;
        nearest_index = i;
      }
    }

    return nearest_index;
  }

  double calculate_lookahead_distance() const
  {
    const double speed_for_lookahead =
      std::max(std::abs(current_speed_), 0.0);

    return std::clamp(
      lookahead_base_ +
      lookahead_speed_gain_ * speed_for_lookahead,
      lookahead_min_,
      lookahead_max_);
  }

  std::size_t find_lookahead_index(
    const std::size_t nearest_index,
    const double desired_lookahead_distance) const
  {
    if (latest_path_.poses.empty()) {
      return 0;
    }

    double accumulated_distance = 0.0;

    for (
      std::size_t i = nearest_index;
      i + 1 < latest_path_.poses.size();
      ++i)
    {
      const auto & current =
        latest_path_.poses[i].pose.position;
      const auto & next =
        latest_path_.poses[i + 1].pose.position;

      accumulated_distance +=
        std::hypot(
        next.x - current.x,
        next.y - current.y);

      if (accumulated_distance >= desired_lookahead_distance) {
        return i + 1;
      }
    }

    return latest_path_.poses.size() - 1;
  }

  void transform_world_to_vehicle(
    const double world_x,
    const double world_y,
    double & vehicle_frame_x,
    double & vehicle_frame_y) const
  {
    const double dx = world_x - vehicle_x_;
    const double dy = world_y - vehicle_y_;

    const double cos_heading = std::cos(vehicle_heading_);
    const double sin_heading = std::sin(vehicle_heading_);

    vehicle_frame_x =
      cos_heading * dx +
      sin_heading * dy;

    vehicle_frame_y =
      -sin_heading * dx +
      cos_heading * dy;
  }

  bool find_forward_lookahead_index(
    const std::size_t nearest_index,
    const double desired_lookahead_distance,
    std::size_t & lookahead_index,
    double & lookahead_x_vehicle,
    double & lookahead_y_vehicle) const
  {
    lookahead_index =
      find_lookahead_index(
      nearest_index,
      desired_lookahead_distance);

    const std::size_t final_index =
      latest_path_.poses.size() - 1;

    for (
      std::size_t i = lookahead_index;
      i <= final_index;
      ++i)
    {
      const auto & point =
        latest_path_.poses[i].pose.position;

      transform_world_to_vehicle(
        point.x,
        point.y,
        lookahead_x_vehicle,
        lookahead_y_vehicle);

      if (lookahead_x_vehicle >= minimum_forward_target_x_) {
        lookahead_index = i;
        return true;
      }

      if (i == final_index) {
        break;
      }
    }

    return false;
  }

  double calculate_steering_command(
    const double target_x_vehicle,
    const double target_y_vehicle) const
  {
    const double squared_distance =
      target_x_vehicle * target_x_vehicle +
      target_y_vehicle * target_y_vehicle;

    if (
      squared_distance < 1.0e-6 ||
      target_x_vehicle < minimum_forward_target_x_)
    {
      return 0.0;
    }

    const double raw_steering_angle =
      std::atan2(
      2.0 * wheelbase_ * target_y_vehicle,
      squared_distance);

    const double steering_angle =
      std::clamp(
      raw_steering_angle,
      -maximum_steering_angle_,
      maximum_steering_angle_);

    double steering_command = steering_angle;

    if (steering_is_normalized_) {
      steering_command =
        steering_angle / maximum_steering_angle_;
    }

    steering_command *= steering_sign_;

    return std::clamp(
      steering_command,
      -maximum_steering_command_,
      maximum_steering_command_);
  }

  double rate_limit_steering(
    const double desired_command,
    const double dt)
  {
    const double maximum_change =
      maximum_steering_rate_ * std::max(dt, 0.0);

    const double limited_command =
      std::clamp(
      desired_command,
      previous_steering_command_ - maximum_change,
      previous_steering_command_ + maximum_change);

    previous_steering_command_ = limited_command;
    return limited_command;
  }

  void calculate_longitudinal_control(
    const double target_speed,
    const double dt,
    double & throttle,
    double & brake)
  {
    throttle = 0.0;
    brake = 0.0;

    double speed_error =
      target_speed - current_speed_;

    if (std::abs(speed_error) < speed_error_deadband_) {
      speed_error = 0.0;
    }

    const double derivative =
      has_previous_speed_error_ && dt > 1.0e-4 ?
      (speed_error - previous_speed_error_) / dt :
      0.0;

    const double candidate_integral =
      std::clamp(
      speed_integral_ + speed_error * std::max(dt, 0.0),
      -speed_integral_limit_,
      speed_integral_limit_);

    const double unsaturated_output =
      speed_kp_ * speed_error +
      speed_ki_ * candidate_integral +
      speed_kd_ * derivative;

    const double saturated_output =
      std::clamp(
      unsaturated_output,
      -maximum_brake_,
      maximum_throttle_);

    const bool not_saturated =
      std::abs(unsaturated_output - saturated_output) < 1.0e-9;

    const bool saturation_will_recover =
      (unsaturated_output > maximum_throttle_ && speed_error < 0.0) ||
      (unsaturated_output < -maximum_brake_ && speed_error > 0.0);

    if (not_saturated || saturation_will_recover) {
      speed_integral_ = candidate_integral;
    }

    previous_speed_error_ = speed_error;
    has_previous_speed_error_ = true;

    if (saturated_output >= 0.0) {
      throttle = saturated_output;
      brake = 0.0;
    } else {
      throttle = 0.0;
      brake = -saturated_output;
    }
  }

  double distance_to_final_path_point() const
  {
    const auto & final_point =
      latest_path_.poses.back().pose.position;

    return std::hypot(
      final_point.x - vehicle_x_,
      final_point.y - vehicle_y_);
  }

  bool input_is_fresh(
    const std::chrono::steady_clock::time_point & now,
    const std::chrono::steady_clock::time_point & last_time,
    const double timeout) const
  {
    return
      std::chrono::duration<double>(
      now - last_time).count() <= timeout;
  }

  void publish_command(
    double throttle,
    double steering,
    double brake)
  {
    if (
      !std::isfinite(throttle) ||
      !std::isfinite(steering) ||
      !std::isfinite(brake))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Non-finite control output detected. Publishing safe stop.");

      publish_safe_stop();
      return;
    }

    throttle =
      std::clamp(throttle, 0.0, maximum_throttle_);
    steering =
      std::clamp(
      steering,
      -maximum_steering_command_,
      maximum_steering_command_);
    brake =
      std::clamp(brake, 0.0, maximum_brake_);

    if (brake > 0.0) {
      throttle = 0.0;
    }

    simulation::msg::VehicleControl command;
    command.throttle = static_cast<float>(throttle);
    command.steering = static_cast<float>(steering);
    command.brake = static_cast<float>(brake);
    command.reserved = 0.0F;

    command_publisher_->publish(command);
  }

  void publish_safe_stop()
  {
    simulation::msg::VehicleControl command;
    command.throttle = 0.0F;
    command.steering = 0.0F;
    command.brake = static_cast<float>(safe_stop_brake_);
    command.reserved = 0.0F;

    command_publisher_->publish(command);
  }

  void reset_controller_state()
  {
    speed_integral_ = 0.0;
    previous_speed_error_ = 0.0;
    has_previous_speed_error_ = false;
    previous_steering_command_ = 0.0;
  }

  void safe_stop_with_reason(
    const char * reason)
  {
    reset_controller_state();
    publish_safe_stop();

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Safe stop: %s",
      reason);
  }

  void control_loop()
  {
    const auto now = std::chrono::steady_clock::now();
    refresh_parameters_if_needed(now);

    double dt =
      std::chrono::duration<double>(
      now - last_control_time_).count();
    last_control_time_ = now;

    if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
      dt = 0.1;
    }

    const bool tf_available =
      update_vehicle_pose_and_speed(now);

    if (!has_path_) {
      safe_stop_with_reason("waiting for a non-empty world-frame path");
      return;
    }

    if (!input_is_fresh(now, last_path_time_, path_timeout_)) {
      safe_stop_with_reason("/planning/path is stale");
      return;
    }

    if (!has_target_speed_) {
      safe_stop_with_reason("waiting for /decision/target_speed");
      return;
    }

    if (!input_is_fresh(
        now,
        last_target_speed_time_,
        decision_timeout_))
    {
      safe_stop_with_reason("/decision/target_speed is stale");
      return;
    }

    if (!has_stop_required_) {
      safe_stop_with_reason("waiting for /decision/stop_required");
      return;
    }

    if (!input_is_fresh(
        now,
        last_stop_required_time_,
        decision_timeout_))
    {
      safe_stop_with_reason("/decision/stop_required is stale");
      return;
    }

    if (!tf_available) {
      safe_stop_with_reason("vehicle TF is unavailable");
      return;
    }

    if (!has_speed_estimate_) {
      safe_stop_with_reason("initializing current-speed estimate");
      return;
    }

    if (stop_required_) {
      safe_stop_with_reason("Decision Making requested a stop");
      return;
    }

    if (target_speed_ <= 0.0F) {
      safe_stop_with_reason("Decision target speed is zero");
      return;
    }

    if (distance_to_final_path_point() <= goal_stop_distance_) {
      safe_stop_with_reason("vehicle is near the final path point");
      return;
    }

    const double limited_target_speed =
      std::clamp(
      static_cast<double>(target_speed_),
      0.0,
      maximum_target_speed_);

    const std::size_t nearest_index =
      find_nearest_path_index();

    const double lookahead_distance =
      calculate_lookahead_distance();

    std::size_t lookahead_index = nearest_index;
    double lookahead_x_vehicle = 0.0;
    double lookahead_y_vehicle = 0.0;

    if (!find_forward_lookahead_index(
        nearest_index,
        lookahead_distance,
        lookahead_index,
        lookahead_x_vehicle,
        lookahead_y_vehicle))
    {
      safe_stop_with_reason("no forward lookahead point was found");
      return;
    }

    const auto & nearest_point =
      latest_path_.poses[nearest_index].pose.position;

    const auto & lookahead_point =
      latest_path_.poses[lookahead_index].pose.position;

    const double desired_steering =
      calculate_steering_command(
      lookahead_x_vehicle,
      lookahead_y_vehicle);

    const double steering =
      rate_limit_steering(
      desired_steering,
      dt);

    double throttle = 0.0;
    double brake = 0.0;

    calculate_longitudinal_control(
      limited_target_speed,
      dt,
      throttle,
      brake);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      500,
      "mode=%s nearest=%zu (%.2f,%.2f) lookahead=%zu "
      "world=(%.2f,%.2f) vehicle=(%.2f,%.2f) Ld=%.2f "
      "v=%.2f target=%.2f throttle=%.2f steering=%.3f brake=%.2f",
      enable_motion_ ? "ACTIVE" : "DRY_RUN",
      nearest_index,
      nearest_point.x,
      nearest_point.y,
      lookahead_index,
      lookahead_point.x,
      lookahead_point.y,
      lookahead_x_vehicle,
      lookahead_y_vehicle,
      lookahead_distance,
      current_speed_,
      limited_target_speed,
      throttle,
      steering,
      brake);

    if (!enable_motion_) {
      publish_safe_stop();
      return;
    }

    publish_command(
      throttle,
      steering,
      brake);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
    path_subscription_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    target_speed_subscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    stop_required_subscription_;

  rclcpp::Publisher<simulation::msg::VehicleControl>::SharedPtr
    command_publisher_;

  rclcpp::TimerBase::SharedPtr
    control_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::Path latest_path_;

  float target_speed_{0.0F};
  bool stop_required_{true};

  bool has_path_{false};
  bool has_target_speed_{false};
  bool has_stop_required_{false};
  bool has_tf_{false};

  std::chrono::steady_clock::time_point last_path_time_{};
  std::chrono::steady_clock::time_point last_target_speed_time_{};
  std::chrono::steady_clock::time_point last_stop_required_time_{};

  double vehicle_x_{0.0};
  double vehicle_y_{0.0};
  double raw_vehicle_yaw_{0.0};
  double vehicle_heading_{0.0};

  double previous_vehicle_x_{0.0};
  double previous_vehicle_y_{0.0};
  std::chrono::steady_clock::time_point previous_pose_time_{};
  bool has_previous_pose_{false};

  double current_speed_{0.0};
  bool has_speed_estimate_{false};

  double speed_integral_{0.0};
  double previous_speed_error_{0.0};
  bool has_previous_speed_error_{false};

  double previous_steering_command_{0.0};

  std::chrono::steady_clock::time_point last_control_time_{};
  std::chrono::steady_clock::time_point last_parameter_refresh_time_{};

  bool enable_motion_{false};

  double wheelbase_{2.63};
  double vehicle_yaw_offset_{0.0};

  double lookahead_base_{4.0};
  double lookahead_speed_gain_{0.5};
  double lookahead_min_{3.0};
  double lookahead_max_{8.0};
  double minimum_forward_target_x_{0.20};

  double maximum_target_speed_{2.0};
  double hard_speed_limit_{2.0};
  double goal_stop_distance_{1.0};

  bool steering_is_normalized_{true};
  double steering_sign_{1.0};
  double maximum_steering_angle_{0.52};
  double maximum_steering_command_{1.0};
  double maximum_steering_rate_{2.0};

  double speed_kp_{0.35};
  double speed_ki_{0.05};
  double speed_kd_{0.02};
  double speed_integral_limit_{2.0};
  double speed_error_deadband_{0.05};
  double maximum_throttle_{0.35};
  double maximum_brake_{1.0};
  double safe_stop_brake_{1.0};

  double speed_filter_alpha_{0.30};
  double maximum_pose_jump_{3.0};

  double path_timeout_{3.0};
  double decision_timeout_{1.0};
};

int main(
  int argc,
  char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ControllerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
