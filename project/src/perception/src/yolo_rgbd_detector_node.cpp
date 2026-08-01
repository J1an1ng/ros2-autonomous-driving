// Copyright 2026 Team 20

#include <onnxruntime_cxx_api.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "perception/traffic_light_stability_filter.hpp"
#include "perception/traffic_light_route_gate.hpp"
#include "perception/yolo_postprocess.hpp"

namespace
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct RigidTransform
{
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double r00{1.0};
  double r01{0.0};
  double r02{0.0};
  double r10{0.0};
  double r11{1.0};
  double r12{0.0};
  double r20{0.0};
  double r21{0.0};
  double r22{1.0};

  Point3 apply(const Point3 & point) const
  {
    return Point3{
      r00 * point.x + r01 * point.y + r02 * point.z + tx,
        r10 * point.x + r11 * point.y + r12 * point.z + ty,
        r20 * point.x + r21 * point.y + r22 * point.z + tz};
  }
};

struct PinholeCamera
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};
};

struct ProjectedDepthSample
{
  float image_x{0.0F};
  float image_y{0.0F};
  float range_m{0.0F};
  float lateral_m{0.0F};
};

struct VehicleMeasurement
{
  perception::yolo::Detection detection;
  int valid_depth_points{0};
  double distance_m{std::numeric_limits<double>::infinity()};
  double lateral_m{std::numeric_limits<double>::infinity()};
  bool accepted{false};
};

struct TrafficLightRoiProfile
{
  double min_x_ratio{0.0};
  double max_x_ratio{1.0};
  double min_y_ratio{0.0};
  double max_y_ratio{1.0};
  double max_aspect_ratio{1.0};
  double target_x_at_stop_ratio{0.5};
  double target_x_per_meter{0.0};
  double target_y_at_stop_ratio{0.34};
  double target_y_per_meter{0.0};
  double target_radius{0.1};
  double target_y_tolerance{0.1};
};

double StampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

bool IsColourEncodingSupported(const std::string & encoding)
{
  return encoding == "rgb8" || encoding == "bgr8" ||
         encoding == "rgba8" || encoding == "bgra8";
}

bool IsDepthEncodingSupported(const std::string & encoding)
{
  return encoding == "16UC1" || encoding == "mono16" ||
         encoding == "32FC1";
}

cv::Mat ToBgr(const sensor_msgs::msg::Image & image)
{
  if (!IsColourEncodingSupported(image.encoding) ||
    image.width == 0 || image.height == 0)
  {
    return {};
  }
  const bool alpha =
    image.encoding == "rgba8" || image.encoding == "bgra8";
  const int type = alpha ? CV_8UC4 : CV_8UC3;
  const size_t required =
    static_cast<size_t>(image.step) * static_cast<size_t>(image.height);
  if (image.data.size() < required) {
    return {};
  }
  const cv::Mat view(
    static_cast<int>(image.height), static_cast<int>(image.width), type,
    const_cast<std::uint8_t *>(image.data.data()), image.step);
  cv::Mat bgr;
  if (image.encoding == "rgb8") {
    cv::cvtColor(view, bgr, cv::COLOR_RGB2BGR);
  } else if (image.encoding == "rgba8") {
    cv::cvtColor(view, bgr, cv::COLOR_RGBA2BGR);
  } else if (image.encoding == "bgra8") {
    cv::cvtColor(view, bgr, cv::COLOR_BGRA2BGR);
  } else {
    bgr = view.clone();
  }
  return bgr;
}

double ReadDepthMeters(
  const sensor_msgs::msg::Image & image, const int x, const int y)
{
  if (!IsDepthEncodingSupported(image.encoding) ||
    x < 0 || y < 0 ||
    x >= static_cast<int>(image.width) ||
    y >= static_cast<int>(image.height))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (image.encoding == "16UC1" || image.encoding == "mono16") {
    const size_t index =
      static_cast<size_t>(y) * image.step +
      static_cast<size_t>(x) * sizeof(std::uint16_t);
    if (index + sizeof(std::uint16_t) > image.data.size()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    std::uint16_t raw = 0;
    if (image.is_bigendian) {
      raw = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(image.data[index]) << 8U) |
        static_cast<std::uint16_t>(image.data[index + 1]));
    } else {
      raw = static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(image.data[index]) |
        (static_cast<std::uint16_t>(image.data[index + 1]) << 8U));
    }
    return static_cast<double>(raw) * 0.001;
  }

  const size_t index =
    static_cast<size_t>(y) * image.step +
    static_cast<size_t>(x) * sizeof(float);
  if (index + sizeof(float) > image.data.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  float raw = 0.0F;
  std::memcpy(&raw, image.data.data() + index, sizeof(float));
  return static_cast<double>(raw);
}

RigidTransform ToRigidTransform(
  const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & message = transform.transform;
  const double norm = std::sqrt(
    message.rotation.x * message.rotation.x +
    message.rotation.y * message.rotation.y +
    message.rotation.z * message.rotation.z +
    message.rotation.w * message.rotation.w);
  const double x = norm > 1.0e-9 ? message.rotation.x / norm : 0.0;
  const double y = norm > 1.0e-9 ? message.rotation.y / norm : 0.0;
  const double z = norm > 1.0e-9 ? message.rotation.z / norm : 0.0;
  const double w = norm > 1.0e-9 ? message.rotation.w / norm : 1.0;

  RigidTransform result;
  result.tx = message.translation.x;
  result.ty = message.translation.y;
  result.tz = message.translation.z;
  result.r00 = 1.0 - 2.0 * (y * y + z * z);
  result.r01 = 2.0 * (x * y - z * w);
  result.r02 = 2.0 * (x * z + y * w);
  result.r10 = 2.0 * (x * y + z * w);
  result.r11 = 1.0 - 2.0 * (x * x + z * z);
  result.r12 = 2.0 * (y * z - x * w);
  result.r20 = 2.0 * (x * z - y * w);
  result.r21 = 2.0 * (y * z + x * w);
  result.r22 = 1.0 - 2.0 * (x * x + y * y);
  return result;
}

std::set<int> ToIntSet(const std::vector<std::int64_t> & values)
{
  std::set<int> result;
  for (const auto value : values) {
    result.insert(static_cast<int>(value));
  }
  return result;
}

std::string StateName(const std::uint8_t state)
{
  switch (state) {
    case perception::yolo::kRed:
      return "RED";
    case perception::yolo::kYellow:
      return "YELLOW";
    case perception::yolo::kGreen:
      return "GREEN";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

class YoloRgbdDetectorNode : public rclcpp::Node
{
public:
  YoloRgbdDetectorNode()
  : Node("yolo_rgbd_detector_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(
      std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
  {
    const std::string default_model =
      ament_index_cpp::get_package_share_directory("perception") +
      "/models/yolov8n.onnx";

    model_path_ = declare_parameter<std::string>("model_path", default_model);
    input_width_ = std::max(
      32, static_cast<int>(declare_parameter<int>("input_width", 640)));
    input_height_ = std::max(
      32, static_cast<int>(declare_parameter<int>("input_height", 640)));
    class_count_ = std::max(
      1, static_cast<int>(declare_parameter<int>("class_count", 80)));
    confidence_threshold_ = static_cast<float>(
      std::clamp(
        declare_parameter<double>("confidence_threshold", 0.25),
        0.0, 1.0));
    nms_threshold_ = static_cast<float>(
      std::clamp(
        declare_parameter<double>("nms_threshold", 0.45),
        0.0, 1.0));
    vehicle_class_ids_ = ToIntSet(
      declare_parameter<std::vector<std::int64_t>>(
        "vehicle_class_ids", {2, 5, 7}));
    traffic_light_class_ids_ = ToIntSet(
      declare_parameter<std::vector<std::int64_t>>(
        "traffic_light_class_ids", {9}));
    traffic_light_confidence_threshold_ = static_cast<float>(
      std::clamp(
        declare_parameter<double>(
          "traffic_light_confidence_threshold", 0.01),
        0.0, 1.0));
    allowed_class_ids_ = vehicle_class_ids_;
    allowed_class_ids_.insert(
      traffic_light_class_ids_.begin(), traffic_light_class_ids_.end());

    rgb_topic_ = declare_parameter<std::string>(
      "rgb_topic", "/OurCar/Sensors/RGBCameraLeft/image_raw");
    rgb_camera_info_topic_ = declare_parameter<std::string>(
      "rgb_camera_info_topic",
      "/OurCar/Sensors/RGBCameraLeft/camera_info");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/OurCar/Sensors/DepthCamera/image_raw");
    depth_camera_info_topic_ = declare_parameter<std::string>(
      "depth_camera_info_topic",
      "/OurCar/Sensors/DepthCamera/camera_info");
    use_latest_tf_ = declare_parameter<bool>("use_latest_tf", true);
    maximum_depth_age_s_ = std::max(
      0.0, declare_parameter<double>("maximum_depth_age_s", 1.0));
    depth_sample_step_ = std::max(
      1, static_cast<int>(
        declare_parameter<int>("depth_sample_step", 2)));
    minimum_depth_points_ = std::max(
      1, static_cast<int>(
        declare_parameter<int>("minimum_depth_points", 20)));
    distance_percentile_ = std::clamp(
      declare_parameter<double>("distance_percentile", 0.20),
      0.0, 1.0);
    minimum_range_m_ = std::max(
      0.0, declare_parameter<double>("minimum_range_m", 0.5));
    maximum_range_m_ = std::max(
      minimum_range_m_,
      declare_parameter<double>("maximum_range_m", 35.0));
    maximum_abs_lateral_m_ = std::max(
      0.0, declare_parameter<double>("maximum_abs_lateral_m", 2.8));
    bbox_inner_x_ratio_ = std::clamp(
      declare_parameter<double>("bbox_inner_x_ratio", 0.08),
      0.0, 0.45);
    bbox_top_trim_ratio_ = std::clamp(
      declare_parameter<double>("bbox_top_trim_ratio", 0.08),
      0.0, 0.45);
    bbox_bottom_trim_ratio_ = std::clamp(
      declare_parameter<double>("bbox_bottom_trim_ratio", 0.15),
      0.0, 0.45);

    traffic_light_waypoint_ids_ = ToIntSet(
      declare_parameter<std::vector<std::int64_t>>(
        "traffic_light_waypoint_ids", {25, 32, 52, 59}));
    const int traffic_light_post_waypoint_hold_count = std::max(
      0, static_cast<int>(declare_parameter<int>(
        "traffic_light_post_waypoint_hold_count", 1)));
    traffic_route_gate_ =
      std::make_unique<perception::TrafficLightRouteGate>(
      traffic_light_waypoint_ids_, traffic_light_post_waypoint_hold_count);
    traffic_light_activation_distance_m_ = std::max(
      0.0, declare_parameter<double>(
        "traffic_light_activation_distance_m", 20.0));
    traffic_light_max_y_ratio_ = std::clamp(
      declare_parameter<double>("traffic_light_max_y_ratio", 0.60),
      0.0, 1.0);
    const auto traffic_light_roi_waypoint_ids =
      declare_parameter<std::vector<std::int64_t>>(
      "traffic_light_roi_waypoint_ids", {25, 52});
    const auto roi_min_x = declare_parameter<std::vector<double>>(
      "traffic_light_roi_min_x_ratios", {0.30, 0.46});
    const auto roi_max_x = declare_parameter<std::vector<double>>(
      "traffic_light_roi_max_x_ratios", {0.60, 0.60});
    const auto roi_min_y = declare_parameter<std::vector<double>>(
      "traffic_light_roi_min_y_ratios", {0.28, 0.25});
    const auto roi_max_y = declare_parameter<std::vector<double>>(
      "traffic_light_roi_max_y_ratios", {0.43, 0.48});
    const auto roi_max_aspect = declare_parameter<std::vector<double>>(
      "traffic_light_roi_max_aspect_ratios", {0.80, 0.80});
    const auto roi_target_x_at_stop =
      declare_parameter<std::vector<double>>(
      "traffic_light_roi_target_x_at_stop_ratios", {0.40, 0.515});
    const auto roi_target_x_per_meter =
      declare_parameter<std::vector<double>>(
      "traffic_light_roi_target_x_per_meters", {0.014, 0.0});
    const auto roi_target_y_at_stop =
      declare_parameter<std::vector<double>>(
      "traffic_light_roi_target_y_at_stop_ratios", {0.34, 0.30});
    const auto roi_target_y_per_meter =
      declare_parameter<std::vector<double>>(
      "traffic_light_roi_target_y_per_meters", {0.005, 0.010});
    const auto roi_target_radius = declare_parameter<std::vector<double>>(
      "traffic_light_roi_target_radii", {0.10, 0.10});
    const auto roi_target_y_tolerance =
      declare_parameter<std::vector<double>>(
      "traffic_light_roi_target_y_tolerances", {0.06, 0.07});
    const size_t roi_profile_count = traffic_light_roi_waypoint_ids.size();
    const bool roi_sizes_valid =
      roi_min_x.size() == roi_profile_count &&
      roi_max_x.size() == roi_profile_count &&
      roi_min_y.size() == roi_profile_count &&
      roi_max_y.size() == roi_profile_count &&
      roi_max_aspect.size() == roi_profile_count &&
      roi_target_x_at_stop.size() == roi_profile_count &&
      roi_target_x_per_meter.size() == roi_profile_count &&
      roi_target_y_at_stop.size() == roi_profile_count &&
      roi_target_y_per_meter.size() == roi_profile_count &&
      roi_target_radius.size() == roi_profile_count &&
      roi_target_y_tolerance.size() == roi_profile_count;
    if (!roi_sizes_valid) {
      throw std::invalid_argument(
              "traffic-light ROI parameter arrays must have the same length");
    }
    for (size_t index = 0; index < roi_profile_count; ++index) {
      TrafficLightRoiProfile profile;
      profile.min_x_ratio = std::clamp(roi_min_x[index], 0.0, 1.0);
      profile.max_x_ratio = std::clamp(
        roi_max_x[index], profile.min_x_ratio, 1.0);
      profile.min_y_ratio = std::clamp(roi_min_y[index], 0.0, 1.0);
      profile.max_y_ratio = std::clamp(
        roi_max_y[index], profile.min_y_ratio, 1.0);
      profile.max_aspect_ratio = std::max(0.0, roi_max_aspect[index]);
      profile.target_x_at_stop_ratio = std::clamp(
        roi_target_x_at_stop[index], 0.0, 1.0);
      profile.target_x_per_meter = roi_target_x_per_meter[index];
      profile.target_y_at_stop_ratio = std::clamp(
        roi_target_y_at_stop[index], 0.0, 1.0);
      profile.target_y_per_meter = roi_target_y_per_meter[index];
      profile.target_radius = std::max(0.0, roi_target_radius[index]);
      profile.target_y_tolerance = std::max(
        0.0, roi_target_y_tolerance[index]);
      traffic_light_roi_profiles_.insert_or_assign(
        static_cast<int>(traffic_light_roi_waypoint_ids[index]), profile);
    }
    traffic_target_x_ratio_ = std::clamp(
      declare_parameter<double>("traffic_target_x_ratio", 0.50),
      0.0, 1.0);
    traffic_target_y_ratio_ = std::clamp(
      declare_parameter<double>("traffic_target_y_ratio", 0.34),
      0.0, 1.0);
    traffic_center_penalty_ = std::max(
      0.0, declare_parameter<double>("traffic_center_penalty", 0.50));
    traffic_post_hold_fallback_waypoint_ids_ = ToIntSet(
      declare_parameter<std::vector<std::int64_t>>(
        "traffic_post_hold_fallback_waypoint_ids", {25}));
    traffic_post_hold_fallback_width_ratio_ = std::clamp(
      declare_parameter<double>(
        "traffic_post_hold_fallback_width_ratio", 0.035),
      0.005, 0.20);
    traffic_post_hold_fallback_height_ratio_ = std::clamp(
      declare_parameter<double>(
        "traffic_post_hold_fallback_height_ratio", 0.095),
      0.01, 0.30);
    traffic_post_hold_fallback_x_offset_ratio_ = std::clamp(
      declare_parameter<double>(
        "traffic_post_hold_fallback_x_offset_ratio", 0.003),
      -0.10, 0.10);
    traffic_post_hold_fallback_y_offset_ratio_ = std::clamp(
      declare_parameter<double>(
        "traffic_post_hold_fallback_y_offset_ratio", 0.010),
      -0.10, 0.10);
    const int consecutive_frames = std::max(
      1, static_cast<int>(
        declare_parameter<int>("traffic_consecutive_frames", 4)));
    const int confident_green_frames = std::max(
      1, static_cast<int>(
        declare_parameter<int>("traffic_confident_green_frames", 2)));
    const int confident_green_max_red_score = std::max(
      0, static_cast<int>(declare_parameter<int>(
        "traffic_confident_green_max_red_score", 4)));
    const int confident_green_min_score = std::max(
      0, static_cast<int>(declare_parameter<int>(
        "traffic_confident_green_min_score", 20)));
    traffic_filter_ =
      std::make_unique<perception::TrafficLightStabilityFilter>(
      consecutive_frames, confident_green_frames,
      confident_green_max_red_score, confident_green_min_score);
    publish_debug_image_ =
      declare_parameter<bool>("publish_debug_image", true);

    vehicle_detected_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/perception/front_vehicle_detected", 10);
    vehicle_distance_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/perception/front_vehicle_distance", 10);
    vehicle_debug_pub_ = create_publisher<std_msgs::msg::String>(
      "/perception/front_vehicle_debug", 10);
    traffic_state_pub_ = create_publisher<std_msgs::msg::UInt8>(
      "/perception/traffic_light_state", 10);
    traffic_scores_pub_ =
      create_publisher<std_msgs::msg::Int32MultiArray>(
      "/perception/traffic_light_scores", 10);
    traffic_debug_pub_ = create_publisher<std_msgs::msg::String>(
      "/perception/traffic_light_debug", 10);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/perception/yolo/debug_image", rclcpp::SensorDataQoS());

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_, rclcpp::SensorDataQoS(),
      std::bind(
        &YoloRgbdDetectorNode::OnRgb, this, std::placeholders::_1));
    rgb_camera_info_sub_ =
      create_subscription<sensor_msgs::msg::CameraInfo>(
      rgb_camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr message) {
        latest_rgb_camera_info_ = std::move(message);
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr message) {
        if (IsDepthEncodingSupported(message->encoding)) {
          latest_depth_ = std::move(message);
        }
      });
    depth_camera_info_sub_ =
      create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr message) {
        latest_depth_camera_info_ = std::move(message);
      });
    waypoint_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/route/current_waypoint_id", 10,
      [this](std_msgs::msg::Int32::SharedPtr message) {
        current_waypoint_id_ = message->data;
        traffic_route_gate_->updateWaypoint(current_waypoint_id_);
      });
    waypoint_distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/route/current_waypoint_distance", 10,
      [this](std_msgs::msg::Float32::SharedPtr message) {
        current_waypoint_distance_m_ = message->data;
      });

    LoadModel();
    RCLCPP_INFO(
      get_logger(),
      "YOLO RGB-D detector: rgb=%s depth=%s model=%s input=%dx%d",
      rgb_topic_.c_str(), depth_topic_.c_str(), model_path_.c_str(),
      input_width_, input_height_);
  }

private:
  void LoadModel()
  {
    model_ready_ = false;
    if (!std::filesystem::is_regular_file(model_path_)) {
      model_status_ = "model_missing";
      RCLCPP_ERROR(
        get_logger(),
        "YOLO ONNX model does not exist: '%s'. The node will publish safe "
        "UNKNOWN/no-vehicle outputs until a valid model is installed.",
        model_path_.c_str());
      return;
    }
    try {
      session_options_.SetIntraOpNumThreads(4);
      session_options_.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);
      session_ = std::make_unique<Ort::Session>(
        ort_environment_, model_path_.c_str(), session_options_);
      Ort::AllocatorWithDefaultOptions allocator;
      const auto input_name =
        session_->GetInputNameAllocated(0, allocator);
      input_name_ = input_name.get();
      output_name_storage_.clear();
      output_names_.clear();
      const size_t output_count = session_->GetOutputCount();
      output_name_storage_.reserve(output_count);
      output_names_.reserve(output_count);
      for (size_t index = 0; index < output_count; ++index) {
        const auto output_name =
          session_->GetOutputNameAllocated(index, allocator);
        output_name_storage_.emplace_back(output_name.get());
      }
      for (const auto & output_name : output_name_storage_) {
        output_names_.push_back(output_name.c_str());
      }
      model_ready_ = true;
      model_status_ = "ready";
      RCLCPP_INFO(
        get_logger(), "Loaded YOLO ONNX model from %s", model_path_.c_str());
    } catch (const Ort::Exception & error) {
      model_status_ = std::string("model_load_error:") + error.what();
      RCLCPP_ERROR(
        get_logger(), "Failed to load YOLO ONNX model '%s': %s",
        model_path_.c_str(), error.what());
    }
  }

  PinholeCamera CameraFromInfo(
    const sensor_msgs::msg::CameraInfo & information,
    const int image_width,
    const int image_height) const
  {
    const double scale_x =
      static_cast<double>(image_width) /
      static_cast<double>(std::max<std::uint32_t>(1, information.width));
    const double scale_y =
      static_cast<double>(image_height) /
      static_cast<double>(std::max<std::uint32_t>(1, information.height));
    return PinholeCamera{
      information.k[0] * scale_x,
      information.k[4] * scale_y,
      information.k[2] * scale_x,
      information.k[5] * scale_y,
      image_width,
      image_height};
  }

  std::optional<RigidTransform> LookupDepthToRgb(
    const sensor_msgs::msg::Image & rgb,
    const sensor_msgs::msg::Image & depth)
  {
    if (rgb.header.frame_id.empty() || depth.header.frame_id.empty()) {
      return std::nullopt;
    }
    try {
      const rclcpp::Time lookup_time =
        use_latest_tf_ ?
        rclcpp::Time(0, 0, get_clock()->get_clock_type()) :
        rclcpp::Time(depth.header.stamp);
      return ToRigidTransform(tf_buffer_->lookupTransform(
          rgb.header.frame_id, depth.header.frame_id, lookup_time));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot transform depth frame '%s' to RGB frame '%s': %s",
        depth.header.frame_id.c_str(), rgb.header.frame_id.c_str(),
        error.what());
      return std::nullopt;
    }
  }

  std::vector<ProjectedDepthSample> ProjectDepthToRgb(
    const sensor_msgs::msg::Image & rgb,
    std::string & status,
    double & stamp_delta_s)
  {
    stamp_delta_s = std::numeric_limits<double>::infinity();
    if (!latest_depth_) {
      status = "waiting_for_depth";
      return {};
    }
    if (!latest_depth_camera_info_ || !latest_rgb_camera_info_) {
      status = "waiting_for_camera_info";
      return {};
    }
    const auto & depth = *latest_depth_;
    if (!IsDepthEncodingSupported(depth.encoding)) {
      status = "unsupported_depth_encoding";
      return {};
    }
    stamp_delta_s = std::abs(
      StampSeconds(rgb.header.stamp) -
      StampSeconds(depth.header.stamp));
    if (
      maximum_depth_age_s_ > 0.0 &&
      stamp_delta_s > maximum_depth_age_s_)
    {
      status = "stale_depth";
      return {};
    }
    const auto transform = LookupDepthToRgb(rgb, depth);
    if (!transform) {
      status = "depth_to_rgb_tf_unavailable";
      return {};
    }

    const PinholeCamera depth_camera = CameraFromInfo(
      *latest_depth_camera_info_,
      static_cast<int>(depth.width), static_cast<int>(depth.height));
    const PinholeCamera rgb_camera = CameraFromInfo(
      *latest_rgb_camera_info_,
      static_cast<int>(rgb.width), static_cast<int>(rgb.height));
    if (
      depth_camera.fx <= 1.0e-9 || depth_camera.fy <= 1.0e-9 ||
      rgb_camera.fx <= 1.0e-9 || rgb_camera.fy <= 1.0e-9)
    {
      status = "invalid_camera_intrinsics";
      return {};
    }

    std::vector<ProjectedDepthSample> samples;
    samples.reserve(
      static_cast<size_t>(
        (depth.width / depth_sample_step_ + 1) *
        (depth.height / depth_sample_step_ + 1)));
    for (
      int v = 0; v < static_cast<int>(depth.height);
      v += depth_sample_step_)
    {
      for (
        int u = 0; u < static_cast<int>(depth.width);
        u += depth_sample_step_)
      {
        const double range = ReadDepthMeters(depth, u, v);
        if (
          !std::isfinite(range) ||
          range < minimum_range_m_ || range > maximum_range_m_)
        {
          continue;
        }
        const Point3 depth_point{
          (static_cast<double>(u) - depth_camera.cx) *
          range / depth_camera.fx,
          (static_cast<double>(v) - depth_camera.cy) *
          range / depth_camera.fy,
          range};
        const Point3 rgb_point = transform->apply(depth_point);
        if (!std::isfinite(rgb_point.z) || rgb_point.z <= 1.0e-6) {
          continue;
        }
        const double image_x =
          rgb_camera.fx * rgb_point.x / rgb_point.z + rgb_camera.cx;
        const double image_y =
          rgb_camera.fy * rgb_point.y / rgb_point.z + rgb_camera.cy;
        if (
          image_x < 0.0 || image_y < 0.0 ||
          image_x >= rgb_camera.width || image_y >= rgb_camera.height)
        {
          continue;
        }
        samples.push_back(ProjectedDepthSample{
          static_cast<float>(image_x),
          static_cast<float>(image_y),
          static_cast<float>(rgb_point.z),
          static_cast<float>(rgb_point.x)});
      }
    }
    status = "ok";
    return samples;
  }

  VehicleMeasurement MeasureVehicle(
    const perception::yolo::Detection & detection,
    const std::vector<ProjectedDepthSample> & samples) const
  {
    VehicleMeasurement measurement;
    measurement.detection = detection;
    const int x_trim = static_cast<int>(
      std::round(bbox_inner_x_ratio_ * detection.box.width));
    const int top_trim = static_cast<int>(
      std::round(bbox_top_trim_ratio_ * detection.box.height));
    const int bottom_trim = static_cast<int>(
      std::round(bbox_bottom_trim_ratio_ * detection.box.height));
    const cv::Rect interior(
      detection.box.x + x_trim,
      detection.box.y + top_trim,
      std::max(1, detection.box.width - 2 * x_trim),
      std::max(1, detection.box.height - top_trim - bottom_trim));

    std::vector<std::pair<float, float>> ranges;
    for (const auto & sample : samples) {
      if (
        sample.image_x >= interior.x &&
        sample.image_y >= interior.y &&
        sample.image_x < interior.x + interior.width &&
        sample.image_y < interior.y + interior.height)
      {
        ranges.emplace_back(sample.range_m, sample.lateral_m);
      }
    }
    measurement.valid_depth_points = static_cast<int>(ranges.size());
    if (measurement.valid_depth_points < minimum_depth_points_) {
      return measurement;
    }
    std::sort(
      ranges.begin(), ranges.end(),
      [](const auto & left, const auto & right) {
        return left.first < right.first;
      });
    const size_t index = std::min(
      ranges.size() - 1,
      static_cast<size_t>(std::floor(
          distance_percentile_ *
          static_cast<double>(ranges.size() - 1))));
    measurement.distance_m = ranges[index].first;
    measurement.lateral_m = ranges[index].second;
    measurement.accepted =
      std::isfinite(measurement.lateral_m) &&
      std::abs(measurement.lateral_m) <= maximum_abs_lateral_m_;
    return measurement;
  }

  std::optional<perception::yolo::Detection> SelectTrafficLight(
    const std::vector<perception::yolo::Detection> & detections,
    const cv::Size & image_size) const
  {
    std::optional<perception::yolo::Detection> best;
    double best_score = -std::numeric_limits<double>::infinity();
    for (const auto & detection : detections) {
      if (traffic_light_class_ids_.count(detection.class_id) == 0) {
        continue;
      }
      if (detection.confidence < traffic_light_confidence_threshold_) {
        continue;
      }
      const double center_x =
        detection.box.x + 0.5 * detection.box.width;
      const double center_y =
        detection.box.y + 0.5 * detection.box.height;
      if (
        center_y >
        traffic_light_max_y_ratio_ * static_cast<double>(image_size.height))
      {
        continue;
      }
      const double normalized_x =
        center_x / static_cast<double>(std::max(1, image_size.width));
      const double normalized_y =
        center_y / static_cast<double>(std::max(1, image_size.height));
      const auto roi_profile = traffic_light_roi_profiles_.find(
        traffic_route_gate_->activeWaypointId());
      const bool restricted_roi =
        roi_profile != traffic_light_roi_profiles_.end();
      double target_x_ratio = traffic_target_x_ratio_;
      double target_y_ratio = traffic_target_y_ratio_;
      if (restricted_roi) {
        const auto & profile = roi_profile->second;
        const double aspect_ratio =
          static_cast<double>(detection.box.width) /
          static_cast<double>(std::max(1, detection.box.height));
        if (
          normalized_x < profile.min_x_ratio ||
          normalized_x > profile.max_x_ratio ||
          normalized_y < profile.min_y_ratio ||
          normalized_y > profile.max_y_ratio ||
          aspect_ratio > profile.max_aspect_ratio)
        {
          continue;
        }
        double approach_distance_m = 0.0;
        if (
          !traffic_route_gate_->isPostWaypointHold() &&
          std::isfinite(current_waypoint_distance_m_))
        {
          approach_distance_m = std::clamp(
            current_waypoint_distance_m_, 0.0,
            traffic_light_activation_distance_m_);
        }
        target_x_ratio = std::clamp(
          profile.target_x_at_stop_ratio +
          profile.target_x_per_meter * approach_distance_m,
          0.0, 1.0);
        target_y_ratio = std::clamp(
          profile.target_y_at_stop_ratio +
          profile.target_y_per_meter * approach_distance_m,
          0.0, 1.0);
        if (
          std::abs(normalized_y - target_y_ratio) >
          profile.target_y_tolerance ||
          std::hypot(
            normalized_x - target_x_ratio,
            normalized_y - target_y_ratio) >
          profile.target_radius)
        {
          continue;
        }
      }
      const double target_distance = std::hypot(
        normalized_x - target_x_ratio,
        normalized_y - target_y_ratio);
      const double score =
        detection.confidence - traffic_center_penalty_ * target_distance;
      if (score > best_score) {
        best_score = score;
        best = detection;
      }
    }
    return best;
  }

  bool TrafficControlActive() const
  {
    return traffic_route_gate_->active(
      current_waypoint_distance_m_, traffic_light_activation_distance_m_);
  }

  std::optional<cv::Rect> PostHoldFallbackRoi(
    const cv::Size & image_size) const
  {
    const int controlled_waypoint = traffic_route_gate_->activeWaypointId();
    if (
      !traffic_route_gate_->isPostWaypointHold() ||
      traffic_post_hold_fallback_waypoint_ids_.count(controlled_waypoint) == 0)
    {
      return std::nullopt;
    }
    const auto profile = traffic_light_roi_profiles_.find(controlled_waypoint);
    if (profile == traffic_light_roi_profiles_.end()) {
      return std::nullopt;
    }

    const int width = std::max(
      3, static_cast<int>(std::round(
        traffic_post_hold_fallback_width_ratio_ * image_size.width)));
    const int height = std::max(
      9, static_cast<int>(std::round(
        traffic_post_hold_fallback_height_ratio_ * image_size.height)));
    const double center_x_ratio =
      profile->second.target_x_at_stop_ratio +
      traffic_post_hold_fallback_x_offset_ratio_;
    const double center_y_ratio =
      profile->second.target_y_at_stop_ratio +
      traffic_post_hold_fallback_y_offset_ratio_;
    const int center_x = static_cast<int>(std::round(
      center_x_ratio * static_cast<double>(image_size.width)));
    const int center_y = static_cast<int>(std::round(
      center_y_ratio * static_cast<double>(image_size.height)));
    const int left = std::clamp(
      center_x - width / 2, 0, std::max(0, image_size.width - 1));
    const int top = std::clamp(
      center_y - height / 2, 0, std::max(0, image_size.height - 1));
    const int right = std::clamp(
      left + width, left + 1, std::max(left + 1, image_size.width));
    const int bottom = std::clamp(
      top + height, top + 1, std::max(top + 1, image_size.height));
    return cv::Rect(left, top, right - left, bottom - top);
  }

  void PublishVehicle(
    const std::vector<VehicleMeasurement> & measurements,
    const std::string & depth_status,
    const double depth_stamp_delta_s,
    const double inference_ms)
  {
    const VehicleMeasurement * nearest = nullptr;
    for (const auto & measurement : measurements) {
      if (!measurement.accepted || !std::isfinite(measurement.distance_m)) {
        continue;
      }
      if (
        nearest == nullptr ||
        measurement.distance_m < nearest->distance_m)
      {
        nearest = &measurement;
      }
    }

    std_msgs::msg::Bool detected;
    detected.data = nearest != nullptr;
    vehicle_detected_pub_->publish(detected);
    std_msgs::msg::Float32 distance;
    distance.data = nearest == nullptr ?
      std::numeric_limits<float>::infinity() :
      static_cast<float>(nearest->distance_m);
    vehicle_distance_pub_->publish(distance);

    std::ostringstream debug;
    debug << std::fixed << std::setprecision(3)
          << "source=yolo_rgbd"
          << " model_status=" << model_status_
          << " detected=" << (detected.data ? "true" : "false")
          << " distance_m=";
    if (nearest) {
      debug << nearest->distance_m;
    } else {
      debug << "inf";
    }
    debug << " depth_status=" << depth_status
          << " depth_stamp_delta_s=";
    if (std::isfinite(depth_stamp_delta_s)) {
      debug << depth_stamp_delta_s;
    } else {
      debug << "inf";
    }
    debug << " inference_ms=" << inference_ms
          << " vehicle_boxes=" << measurements.size()
          << " measurements=[";
    for (size_t index = 0; index < measurements.size(); ++index) {
      if (index > 0) {
        debug << ";";
      }
      const auto & measurement = measurements[index];
      debug << "class=" << measurement.detection.class_id
            << ",conf=" << measurement.detection.confidence
            << ",bbox=(" << measurement.detection.box.x
            << "," << measurement.detection.box.y
            << "," << measurement.detection.box.width
            << "," << measurement.detection.box.height << ")"
            << ",depth_points=" << measurement.valid_depth_points
            << ",distance=";
      if (std::isfinite(measurement.distance_m)) {
        debug << measurement.distance_m;
      } else {
        debug << "inf";
      }
      debug << ",lateral=";
      if (std::isfinite(measurement.lateral_m)) {
        debug << measurement.lateral_m;
      } else {
        debug << "inf";
      }
      debug << ",accepted="
            << (measurement.accepted ? "true" : "false");
    }
    debug << "]";
    std_msgs::msg::String message;
    message.data = debug.str();
    vehicle_debug_pub_->publish(message);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "%s", message.data.c_str());
  }

  void PublishTrafficLight(
    const cv::Mat & image,
    const std::vector<perception::yolo::Detection> & detections,
    std::optional<perception::yolo::Detection> & selected_detection,
    perception::yolo::TrafficLightScores & scores)
  {
    const bool active = TrafficControlActive();
    bool fallback_used = false;
    if (active) {
      selected_detection = SelectTrafficLight(detections, image.size());
      if (selected_detection) {
        const cv::Rect original_box = selected_detection->box;
        scores = perception::yolo::ClassifyTrafficLight(
          image(original_box));
        const cv::Rect colour_box =
          perception::yolo::ExpandTrafficLightColourBox(
          original_box, image.size());
        if (scores.state != perception::yolo::kGreen) {
          cv::Mat colour_hsv;
          cv::cvtColor(
            image(colour_box), colour_hsv, cv::COLOR_BGR2HSV);
          const int extended_green_score =
            perception::yolo::SupplementalGreenLampScore(
            colour_hsv, original_box.area());
          if (extended_green_score > 0) {
            scores.red = 0;
            scores.yellow = 0;
            scores.green = extended_green_score;
            scores.state = perception::yolo::kGreen;
          }
        }
        selected_detection->box = colour_box;
      }
      const auto fallback_roi = PostHoldFallbackRoi(image.size());
      if (fallback_roi && scores.state != perception::yolo::kGreen) {
        const int fallback_green_score =
          perception::yolo::PredictedRoiGreenLampScore(
          image(*fallback_roi));
        if (fallback_green_score > 0) {
          scores.red = 0;
          scores.yellow = 0;
          scores.green = fallback_green_score;
          scores.state = perception::yolo::kGreen;
          perception::yolo::Detection fallback_detection;
          fallback_detection.class_id = -1;
          fallback_detection.confidence = 0.0F;
          fallback_detection.box = *fallback_roi;
          selected_detection = fallback_detection;
          fallback_used = true;
        }
      }
    }
    const std::uint8_t stable_state = traffic_filter_->update(
      scores.state, scores.red, scores.green);

    std_msgs::msg::UInt8 state;
    state.data = stable_state;
    traffic_state_pub_->publish(state);
    std_msgs::msg::Int32MultiArray score_message;
    score_message.data = {scores.red, scores.green};
    traffic_scores_pub_->publish(score_message);

    std::ostringstream debug;
    debug << std::fixed << std::setprecision(3)
          << "source=yolo_rgb"
          << " model_status=" << model_status_
          << " waypoint=" << current_waypoint_id_
          << " waypoint_distance_m=";
    if (std::isfinite(current_waypoint_distance_m_)) {
      debug << current_waypoint_distance_m_;
    } else {
      debug << "inf";
    }
    debug << " route_gate=" << (active ? "enabled" : "disabled")
          << " controlled_waypoint="
          << traffic_route_gate_->activeWaypointId()
          << " gate_phase="
          << (traffic_route_gate_->isPostWaypointHold() ? "post" : "approach")
          << " restricted_roi="
          << (traffic_light_roi_profiles_.count(
          traffic_route_gate_->activeWaypointId()) > 0 ? "true" : "false")
          << " candidate=" << (selected_detection ? "true" : "false")
          << " fallback=" << (fallback_used ? "true" : "false")
          << " raw=" << StateName(scores.state)
          << " stable=" << StateName(stable_state)
          << " red_score=" << scores.red
          << " yellow_score=" << scores.yellow
          << " green_score=" << scores.green;
    if (selected_detection) {
      debug << " class=" << selected_detection->class_id
            << " confidence=" << selected_detection->confidence
            << " bbox=(" << selected_detection->box.x
            << "," << selected_detection->box.y
            << "," << selected_detection->box.width
            << "," << selected_detection->box.height << ")";
    }
    std_msgs::msg::String message;
    message.data = debug.str();
    traffic_debug_pub_->publish(message);
  }

  void PublishDebugImage(
    const sensor_msgs::msg::Image & source,
    const cv::Mat & bgr,
    const std::vector<VehicleMeasurement> & vehicles,
    const std::optional<perception::yolo::Detection> & traffic_light,
    const perception::yolo::TrafficLightScores & traffic_scores)
  {
    if (
      !publish_debug_image_ ||
      debug_image_pub_->get_subscription_count() == 0)
    {
      return;
    }
    cv::Mat annotated = bgr.clone();
    for (const auto & vehicle : vehicles) {
      const cv::Scalar colour =
        vehicle.accepted ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
      cv::rectangle(annotated, vehicle.detection.box, colour, 2);
      std::ostringstream label;
      label << "vehicle " << std::fixed << std::setprecision(2)
            << vehicle.detection.confidence << " d=";
      if (std::isfinite(vehicle.distance_m)) {
        label << vehicle.distance_m << "m";
      } else {
        label << "inf";
      }
      cv::putText(
        annotated, label.str(),
        cv::Point(
          vehicle.detection.box.x,
          std::max(12, vehicle.detection.box.y - 4)),
        cv::FONT_HERSHEY_SIMPLEX, 0.42, colour, 1, cv::LINE_AA);
    }
    if (traffic_light) {
      cv::rectangle(
        annotated, traffic_light->box, cv::Scalar(255, 255, 0), 2);
      cv::putText(
        annotated, StateName(traffic_scores.state),
        cv::Point(
          traffic_light->box.x,
          std::max(12, traffic_light->box.y - 4)),
        cv::FONT_HERSHEY_SIMPLEX, 0.42,
        cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    }

    cv::Mat rgb;
    cv::cvtColor(annotated, rgb, cv::COLOR_BGR2RGB);
    sensor_msgs::msg::Image message;
    message.header = source.header;
    message.height = static_cast<std::uint32_t>(rgb.rows);
    message.width = static_cast<std::uint32_t>(rgb.cols);
    message.encoding = "rgb8";
    message.is_bigendian = false;
    message.step = static_cast<std::uint32_t>(rgb.cols * 3);
    message.data.assign(
      rgb.data,
      rgb.data + static_cast<size_t>(message.step) * message.height);
    debug_image_pub_->publish(message);
  }

  void PublishSafeOutputs(const std::string & reason)
  {
    std_msgs::msg::Bool detected;
    detected.data = false;
    vehicle_detected_pub_->publish(detected);
    std_msgs::msg::Float32 distance;
    distance.data = std::numeric_limits<float>::infinity();
    vehicle_distance_pub_->publish(distance);
    std_msgs::msg::String vehicle_debug;
    vehicle_debug.data = "source=yolo_rgbd status=" + reason;
    vehicle_debug_pub_->publish(vehicle_debug);

    const auto stable = traffic_filter_->update(
      perception::yolo::kUnknown, 0, 0);
    std_msgs::msg::UInt8 state;
    state.data = stable;
    traffic_state_pub_->publish(state);
    std_msgs::msg::Int32MultiArray scores;
    scores.data = {0, 0};
    traffic_scores_pub_->publish(scores);
    std_msgs::msg::String traffic_debug;
    traffic_debug.data = "source=yolo_rgb status=" + reason;
    traffic_debug_pub_->publish(traffic_debug);
  }

  void OnRgb(const sensor_msgs::msg::Image::SharedPtr message)
  {
    if (!model_ready_) {
      PublishSafeOutputs(model_status_);
      return;
    }
    const cv::Mat bgr = ToBgr(*message);
    if (bgr.empty()) {
      PublishSafeOutputs(
        "unsupported_or_invalid_rgb_encoding:" + message->encoding);
      return;
    }

    perception::yolo::LetterboxTransform letterbox_transform;
    const cv::Mat input = perception::yolo::Letterbox(
      bgr, input_width_, input_height_, letterbox_transform);
    const cv::Mat blob = cv::dnn::blobFromImage(
      input, 1.0 / 255.0,
      cv::Size(input_width_, input_height_),
      cv::Scalar(), true, false, CV_32F);

    const auto start = std::chrono::steady_clock::now();
    std::vector<Ort::Value> outputs;
    try {
      const std::array<std::int64_t, 4> input_shape{
        1, 3, input_height_, input_width_};
      auto memory = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);
      auto tensor = Ort::Value::CreateTensor<float>(
        memory, const_cast<float *>(blob.ptr<float>()), blob.total(),
        input_shape.data(), input_shape.size());
      const char * input_names[] = {input_name_.c_str()};
      outputs = session_->Run(
        Ort::RunOptions{nullptr},
        input_names, &tensor, 1,
        output_names_.data(), output_names_.size());
    } catch (const Ort::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "YOLO inference failed: %s", error.what());
      PublishSafeOutputs(std::string("inference_error:") + error.what());
      return;
    }
    const double inference_ms =
      std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();

    std::vector<perception::yolo::Detection> detections;
    for (const auto & output : outputs) {
      const auto shape =
        output.GetTensorTypeAndShapeInfo().GetShape();
      if (shape.size() != 2 && shape.size() != 3) {
        continue;
      }
      std::vector<int> dimensions;
      dimensions.reserve(shape.size());
      bool valid_shape = true;
      for (const auto value : shape) {
        if (
          value <= 0 ||
          value > static_cast<std::int64_t>(
            std::numeric_limits<int>::max()))
        {
          valid_shape = false;
          break;
        }
        dimensions.push_back(static_cast<int>(value));
      }
      if (!valid_shape) {
        continue;
      }
      cv::Mat output_view(
        static_cast<int>(dimensions.size()),
        dimensions.data(), CV_32F,
        const_cast<float *>(output.GetTensorData<float>()));
      const auto parsed = perception::yolo::ParseDetections(
        output_view, letterbox_transform, class_count_,
        std::min(
          confidence_threshold_, traffic_light_confidence_threshold_),
        nms_threshold_, allowed_class_ids_);
      detections.insert(detections.end(), parsed.begin(), parsed.end());
    }

    std::string depth_status;
    double depth_stamp_delta_s = std::numeric_limits<double>::infinity();
    const auto depth_samples = ProjectDepthToRgb(
      *message, depth_status, depth_stamp_delta_s);
    std::vector<VehicleMeasurement> vehicle_measurements;
    for (const auto & detection : detections) {
      if (
        vehicle_class_ids_.count(detection.class_id) > 0 &&
        detection.confidence >= confidence_threshold_)
      {
        vehicle_measurements.push_back(
          MeasureVehicle(detection, depth_samples));
      }
    }
    PublishVehicle(
      vehicle_measurements, depth_status,
      depth_stamp_delta_s, inference_ms);

    std::optional<perception::yolo::Detection> selected_traffic_light;
    perception::yolo::TrafficLightScores traffic_scores;
    PublishTrafficLight(
      bgr, detections, selected_traffic_light, traffic_scores);
    PublishDebugImage(
      *message, bgr, vehicle_measurements,
      selected_traffic_light, traffic_scores);
  }

  std::string model_path_;
  std::string model_status_{"not_loaded"};
  bool model_ready_{false};
  Ort::Env ort_environment_{
    ORT_LOGGING_LEVEL_WARNING, "perception_yolo"};
  Ort::SessionOptions session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::vector<std::string> output_name_storage_;
  std::vector<const char *> output_names_;
  int input_width_{640};
  int input_height_{640};
  int class_count_{80};
  float confidence_threshold_{0.25F};
  float nms_threshold_{0.45F};
  std::set<int> vehicle_class_ids_;
  std::set<int> traffic_light_class_ids_;
  std::set<int> allowed_class_ids_;

  std::string rgb_topic_;
  std::string rgb_camera_info_topic_;
  std::string depth_topic_;
  std::string depth_camera_info_topic_;
  bool use_latest_tf_{true};
  double maximum_depth_age_s_{1.0};
  int depth_sample_step_{2};
  int minimum_depth_points_{20};
  double distance_percentile_{0.20};
  double minimum_range_m_{0.5};
  double maximum_range_m_{35.0};
  double maximum_abs_lateral_m_{2.8};
  double bbox_inner_x_ratio_{0.08};
  double bbox_top_trim_ratio_{0.08};
  double bbox_bottom_trim_ratio_{0.15};

  std::set<int> traffic_light_waypoint_ids_;
  std::unique_ptr<perception::TrafficLightRouteGate> traffic_route_gate_;
  float traffic_light_confidence_threshold_{0.01F};
  double traffic_light_activation_distance_m_{20.0};
  double traffic_light_max_y_ratio_{0.60};
  std::map<int, TrafficLightRoiProfile> traffic_light_roi_profiles_;
  double traffic_target_x_ratio_{0.50};
  double traffic_target_y_ratio_{0.34};
  double traffic_center_penalty_{0.50};
  std::set<int> traffic_post_hold_fallback_waypoint_ids_;
  double traffic_post_hold_fallback_width_ratio_{0.035};
  double traffic_post_hold_fallback_height_ratio_{0.095};
  double traffic_post_hold_fallback_x_offset_ratio_{0.003};
  double traffic_post_hold_fallback_y_offset_ratio_{0.010};
  std::unique_ptr<perception::TrafficLightStabilityFilter> traffic_filter_;
  bool publish_debug_image_{true};
  int current_waypoint_id_{-1};
  double current_waypoint_distance_m_{
    std::numeric_limits<double>::infinity()};

  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_depth_camera_info_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_rgb_camera_info_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vehicle_detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vehicle_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehicle_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr traffic_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
    traffic_scores_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr traffic_debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    rgb_camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    depth_camera_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    waypoint_distance_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloRgbdDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
