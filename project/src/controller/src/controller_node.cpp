// Copyright 2026 Team 20

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simulation/msg/vehicle_control.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "controller/control_core.hpp"

namespace controller
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_parameters();

    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      "/planning/path", 10,
      std::bind(&ControllerNode::path_callback, this, std::placeholders::_1));
    target_speed_subscription_ = create_subscription<std_msgs::msg::Float32>(
      "/decision/target_speed", 10,
      std::bind(&ControllerNode::target_speed_callback, this, std::placeholders::_1));
    stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/decision/stop_required", 10,
      std::bind(&ControllerNode::stop_callback, this, std::placeholders::_1));
    twist_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/OurCar/INS/twist", 10,
      std::bind(&ControllerNode::twist_callback, this, std::placeholders::_1));

    command_publisher_ = create_publisher<simulation::msg::VehicleControl>("/car_command", 10);

    const double period = 1.0 / std::max(control_rate_, 1.0);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&ControllerNode::control_loop, this));
    last_control_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "Controller ready: path=/planning/path speed=/decision/target_speed "
      "stop=/decision/stop_required twist=/OurCar/INS/twist output=/car_command");
    RCLCPP_INFO(
      get_logger(), "Frames: %s -> %s; motion=%s",
      map_frame_.c_str(), vehicle_frame_.c_str(), enable_motion_ ? "enabled" : "disabled");
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  void declare_parameters()
  {
    declare_parameter<std::string>("map_frame", "world");
    declare_parameter<std::string>("vehicle_frame", "OurCar/INS");
    declare_parameter<bool>("enable_motion", true);
    declare_parameter<double>("control_rate", 20.0);
    declare_parameter<double>("wheelbase", 2.63);
    declare_parameter<double>("lookahead_base", 3.0);
    declare_parameter<double>("lookahead_speed_gain", 0.5);
    declare_parameter<double>("lookahead_min", 2.0);
    declare_parameter<double>("lookahead_max", 7.0);
    declare_parameter<double>("minimum_forward_distance", 0.2);
    declare_parameter<double>("maximum_target_speed", 14.0);
    declare_parameter<bool>("normalize_steering", true);
    declare_parameter<double>("steering_sign", 1.0);
    declare_parameter<double>("maximum_steering_angle", 0.52);
    declare_parameter<double>("maximum_steering_rate", 2.0);
    declare_parameter<double>("speed_kp", 0.35);
    declare_parameter<double>("speed_ki", 0.05);
    declare_parameter<double>("speed_kd", 0.02);
    declare_parameter<double>("speed_integral_limit", 2.0);
    declare_parameter<double>("speed_error_deadband", 0.05);
    declare_parameter<double>("maximum_throttle", 0.70);
    declare_parameter<double>("maximum_brake", 1.0);
    declare_parameter<double>("safe_stop_brake", 1.0);
    declare_parameter<double>("path_timeout", 1.0);
    declare_parameter<double>("decision_timeout", 0.75);
    declare_parameter<double>("twist_timeout", 1.0);
  }

  void load_parameters()
  {
    map_frame_ = get_parameter("map_frame").as_string();
    vehicle_frame_ = get_parameter("vehicle_frame").as_string();
    enable_motion_ = get_parameter("enable_motion").as_bool();
    control_rate_ = std::max(get_parameter("control_rate").as_double(), 1.0);
    lookahead_base_ = std::max(get_parameter("lookahead_base").as_double(), 0.1);
    lookahead_speed_gain_ = std::max(get_parameter("lookahead_speed_gain").as_double(), 0.0);
    lookahead_min_ = std::max(get_parameter("lookahead_min").as_double(), 0.1);
    lookahead_max_ = std::max(get_parameter("lookahead_max").as_double(), lookahead_min_);
    minimum_forward_distance_ =
      std::max(get_parameter("minimum_forward_distance").as_double(), 0.0);
    maximum_target_speed_ =
      std::max(get_parameter("maximum_target_speed").as_double(), 0.0);
    maximum_steering_rate_ =
      std::max(get_parameter("maximum_steering_rate").as_double(), 0.01);
    safe_stop_brake_ = clamp_value(get_parameter("safe_stop_brake").as_double(), 0.0, 1.0);
    path_timeout_ = std::max(get_parameter("path_timeout").as_double(), 0.1);
    decision_timeout_ = std::max(get_parameter("decision_timeout").as_double(), 0.1);
    twist_timeout_ = std::max(get_parameter("twist_timeout").as_double(), 0.1);

    pursuit_config_.wheelbase = get_parameter("wheelbase").as_double();
    pursuit_config_.normalize_steering = get_parameter("normalize_steering").as_bool();
    pursuit_config_.steering_sign = get_parameter("steering_sign").as_double();
    pursuit_config_.maximum_steering_angle =
      get_parameter("maximum_steering_angle").as_double();

    SpeedControllerConfig speed_config;
    speed_config.kp = get_parameter("speed_kp").as_double();
    speed_config.ki = get_parameter("speed_ki").as_double();
    speed_config.kd = get_parameter("speed_kd").as_double();
    speed_config.integral_limit = get_parameter("speed_integral_limit").as_double();
    speed_config.error_deadband = get_parameter("speed_error_deadband").as_double();
    speed_config.maximum_throttle = get_parameter("maximum_throttle").as_double();
    speed_config.maximum_brake = get_parameter("maximum_brake").as_double();
    speed_controller_ = std::make_unique<SpeedController>(speed_config);
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr message)
  {
    last_path_time_ = std::chrono::steady_clock::now();
    path_.clear();

    if (message->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring path frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), map_frame_.c_str());
      has_path_ = false;
      return;
    }

    path_.reserve(message->poses.size());
    for (const auto & pose : message->poses) {
      const double x = pose.pose.position.x;
      const double y = pose.pose.position.y;
      if (!std::isfinite(x) || !std::isfinite(y)) {
        path_.clear();
        has_path_ = false;
        RCLCPP_WARN(get_logger(), "Ignoring path containing non-finite coordinates");
        return;
      }
      path_.push_back(Point2D{x, y});
    }
    has_path_ = !path_.empty();
  }

  void target_speed_callback(const std_msgs::msg::Float32::SharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      has_target_speed_ = false;
      RCLCPP_WARN(get_logger(), "Ignoring non-finite decision target speed");
      return;
    }
    target_speed_ = message->data;
    has_target_speed_ = true;
    last_target_speed_time_ = std::chrono::steady_clock::now();
  }

  void stop_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    stop_required_ = message->data;
    has_stop_state_ = true;
    last_stop_time_ = std::chrono::steady_clock::now();
  }

  void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    const auto & velocity = message->twist.linear;
    const double speed = std::hypot(velocity.x, velocity.y, velocity.z);
    if (!std::isfinite(speed)) {
      has_twist_ = false;
      RCLCPP_WARN(get_logger(), "Ignoring non-finite vehicle velocity");
      return;
    }
    current_speed_ = speed;
    has_twist_ = true;
    last_twist_time_ = std::chrono::steady_clock::now();
  }

  bool update_pose()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, vehicle_frame_, tf2::TimePointZero);
      vehicle_position_.x = transform.transform.translation.x;
      vehicle_position_.y = transform.transform.translation.y;
      tf2::Quaternion quaternion(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
      double roll = 0.0;
      double pitch = 0.0;
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, vehicle_yaw_);
      return std::isfinite(vehicle_position_.x) &&
             std::isfinite(vehicle_position_.y) && std::isfinite(vehicle_yaw_);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF %s -> %s unavailable: %s",
        map_frame_.c_str(), vehicle_frame_.c_str(), exception.what());
      return false;
    }
  }

  bool fresh(const SteadyTime & now, const SteadyTime & stamp, const double timeout) const
  {
    return std::chrono::duration<double>(now - stamp).count() <= timeout;
  }

  void publish_command(double throttle, double steering, double brake)
  {
    simulation::msg::VehicleControl message;
    message.throttle = static_cast<float>(clamp_value(throttle, 0.0, 1.0));
    message.steering = static_cast<float>(clamp_value(steering, -1.0, 1.0));
    message.brake = static_cast<float>(clamp_value(brake, 0.0, 1.0));
    if (message.brake > 0.0F) {
      message.throttle = 0.0F;
    }
    message.reserved = 0.0F;
    command_publisher_->publish(message);
  }

  void safe_stop(const char * reason)
  {
    speed_controller_->reset();
    previous_steering_ = 0.0;
    publish_command(0.0, 0.0, safe_stop_brake_);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Safe stop: %s", reason);
  }

  void control_loop()
  {
    const SteadyTime now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_control_time_).count();
    last_control_time_ = now;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / control_rate_;
    }

    if (!enable_motion_) {
      safe_stop("motion disabled by parameter");
      return;
    }
    if (!has_path_ || !fresh(now, last_path_time_, path_timeout_)) {
      safe_stop("path missing or stale");
      return;
    }
    if (!has_target_speed_ || !fresh(now, last_target_speed_time_, decision_timeout_)) {
      safe_stop("decision target speed missing or stale");
      return;
    }
    if (!has_stop_state_ || !fresh(now, last_stop_time_, decision_timeout_)) {
      safe_stop("decision stop state missing or stale");
      return;
    }
    if (!has_twist_ || !fresh(now, last_twist_time_, twist_timeout_)) {
      safe_stop("vehicle velocity missing or stale");
      return;
    }
    if (stop_required_ || target_speed_ <= 0.0F) {
      safe_stop("stop requested by decision layer");
      return;
    }
    if (!update_pose()) {
      safe_stop("vehicle pose unavailable");
      return;
    }

    const std::size_t nearest_index = find_nearest_point(path_, vehicle_position_);
    const double lookahead = clamp_value(
      lookahead_base_ + lookahead_speed_gain_ * current_speed_,
      lookahead_min_, lookahead_max_);
    Point2D target;
    if (!find_lookahead_point(
        path_, nearest_index, lookahead, vehicle_position_, vehicle_yaw_,
        minimum_forward_distance_, target))
    {
      safe_stop("no forward lookahead point");
      return;
    }

    const double desired_steering = pure_pursuit_steering(target, pursuit_config_);
    const double maximum_change = maximum_steering_rate_ * dt;
    const double steering = clamp_value(
      desired_steering, previous_steering_ - maximum_change,
      previous_steering_ + maximum_change);
    previous_steering_ = steering;

    const double limited_target_speed = clamp_value(target_speed_, 0.0, maximum_target_speed_);
    const LongitudinalCommand longitudinal =
      speed_controller_->update(limited_target_speed, current_speed_, dt);
    publish_command(longitudinal.throttle, steering, longitudinal.brake);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "v=%.2f target=%.2f lookahead=%.2f steering=%.3f throttle=%.2f brake=%.2f",
      current_speed_, limited_target_speed, lookahead, steering,
      longitudinal.throttle, longitudinal.brake);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_speed_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscription_;
  rclcpp::Publisher<simulation::msg::VehicleControl>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<Point2D> path_;
  Point2D vehicle_position_;
  double vehicle_yaw_{0.0};
  double target_speed_{0.0};
  double current_speed_{0.0};
  double previous_steering_{0.0};
  bool stop_required_{true};
  bool has_path_{false};
  bool has_target_speed_{false};
  bool has_stop_state_{false};
  bool has_twist_{false};

  SteadyTime last_path_time_{};
  SteadyTime last_target_speed_time_{};
  SteadyTime last_stop_time_{};
  SteadyTime last_twist_time_{};
  SteadyTime last_control_time_{};

  std::string map_frame_;
  std::string vehicle_frame_;
  bool enable_motion_{true};
  double control_rate_{20.0};
  double lookahead_base_{3.0};
  double lookahead_speed_gain_{0.5};
  double lookahead_min_{2.0};
  double lookahead_max_{7.0};
  double minimum_forward_distance_{0.2};
  double maximum_target_speed_{14.0};
  double maximum_steering_rate_{2.0};
  double safe_stop_brake_{1.0};
  double path_timeout_{1.0};
  double decision_timeout_{0.75};
  double twist_timeout_{1.0};
  PurePursuitConfig pursuit_config_;
  std::unique_ptr<SpeedController> speed_controller_;
};

}  // namespace controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
