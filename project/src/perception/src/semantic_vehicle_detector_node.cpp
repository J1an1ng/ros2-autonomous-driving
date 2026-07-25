#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

namespace {

struct Rgb {
  uint8_t r{0};
  uint8_t g{0};
  uint8_t b{0};
};

struct Roi {
  int x_min{0};
  int y_min{0};
  int x_max{0};
  int y_max{0};
};

struct Component {
  int pixels{0};
  int min_x{0};
  int min_y{0};
  int max_x{0};
  int max_y{0};
  std::vector<int> image_indices;
  int valid_depth_pixels{0};
  double distance_m{std::numeric_limits<double>::infinity()};
  double lateral_m{std::numeric_limits<double>::infinity()};
  bool in_lateral_corridor{false};
};

double StampSeconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

bool IsColorEncodingSupported(const std::string &encoding) {
  return encoding == "rgb8" || encoding == "bgr8" ||
         encoding == "rgba8" || encoding == "bgra8";
}

bool IsDepthEncodingSupported(const std::string &encoding) {
  return encoding == "16UC1" || encoding == "mono16" ||
         encoding == "32FC1";
}

bool ReadRgb(const sensor_msgs::msg::Image &image, int x, int y, Rgb &rgb) {
  if (!IsColorEncodingSupported(image.encoding) ||
      x < 0 || y < 0 ||
      x >= static_cast<int>(image.width) ||
      y >= static_cast<int>(image.height)) {
    return false;
  }

  const bool has_alpha =
      image.encoding == "rgba8" || image.encoding == "bgra8";
  const bool is_bgr =
      image.encoding == "bgr8" || image.encoding == "bgra8";
  const size_t channels = has_alpha ? 4U : 3U;
  const size_t index =
      static_cast<size_t>(y) * image.step +
      static_cast<size_t>(x) * channels;
  if (index + channels > image.data.size()) {
    return false;
  }

  if (is_bgr) {
    rgb = Rgb{image.data[index + 2], image.data[index + 1],
              image.data[index]};
  } else {
    rgb = Rgb{image.data[index], image.data[index + 1],
              image.data[index + 2]};
  }
  return true;
}

double ReadDepthMeters(const sensor_msgs::msg::Image &image, int x, int y) {
  if (!IsDepthEncodingSupported(image.encoding) ||
      x < 0 || y < 0 ||
      x >= static_cast<int>(image.width) ||
      y >= static_cast<int>(image.height)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (image.encoding == "16UC1" || image.encoding == "mono16") {
    const size_t index =
        static_cast<size_t>(y) * image.step +
        static_cast<size_t>(x) * sizeof(uint16_t);
    if (index + sizeof(uint16_t) > image.data.size()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    uint16_t raw = 0;
    if (image.is_bigendian) {
      raw = static_cast<uint16_t>(
          (static_cast<uint16_t>(image.data[index]) << 8U) |
          static_cast<uint16_t>(image.data[index + 1]));
    } else {
      raw = static_cast<uint16_t>(
          static_cast<uint16_t>(image.data[index]) |
          (static_cast<uint16_t>(image.data[index + 1]) << 8U));
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

void AddNeighbor(int x, int y, int width, int height,
                 const std::vector<uint8_t> &mask,
                 std::vector<uint8_t> &visited,
                 std::vector<int> &stack) {
  if (x < 0 || y < 0 || x >= width || y >= height) {
    return;
  }
  const int index = y * width + x;
  if (!mask[static_cast<size_t>(index)] ||
      visited[static_cast<size_t>(index)]) {
    return;
  }
  visited[static_cast<size_t>(index)] = 1;
  stack.push_back(index);
}

}  // namespace

class SemanticVehicleDetectorNode : public rclcpp::Node {
public:
  SemanticVehicleDetectorNode()
      : Node("semantic_vehicle_detector_node") {
    semantic_topic_ = declare_parameter<std::string>(
        "semantic_topic", "/OurCar/Sensors/SemanticCamera/image_raw");
    depth_topic_ = declare_parameter<std::string>(
        "depth_topic", "/OurCar/Sensors/DepthCamera/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
        "camera_info_topic", "/OurCar/Sensors/DepthCamera/camera_info");
    detected_topic_ = declare_parameter<std::string>(
        "detected_topic", "/perception/front_vehicle_detected");
    distance_topic_ = declare_parameter<std::string>(
        "distance_topic", "/perception/front_vehicle_distance");
    debug_topic_ = declare_parameter<std::string>(
        "debug_topic", "/perception/front_vehicle_debug");

    semantic_r_ = declare_parameter<int>("semantic_r", 255);
    semantic_g_ = declare_parameter<int>("semantic_g", 0);
    semantic_b_ = declare_parameter<int>("semantic_b", 0);
    semantic_tolerance_ = declare_parameter<int>("semantic_tolerance", 45);
    roi_x_min_ = declare_parameter<double>("roi_x_min", 0.12);
    roi_y_min_ = declare_parameter<double>("roi_y_min", 0.10);
    roi_x_max_ = declare_parameter<double>("roi_x_max", 0.88);
    roi_y_max_ = declare_parameter<double>("roi_y_max", 0.98);
    min_component_pixels_ =
        std::max(1, static_cast<int>(
                        declare_parameter<int64_t>("min_component_pixels", 80)));
    max_component_pixels_ =
        std::max(min_component_pixels_,
                 static_cast<int>(declare_parameter<int64_t>(
                     "max_component_pixels", 200000)));
    min_valid_depth_pixels_ =
        std::max(1, static_cast<int>(declare_parameter<int64_t>(
                        "min_valid_depth_pixels", 20)));
    depth_sample_step_ =
        std::max(1, static_cast<int>(
                        declare_parameter<int64_t>("depth_sample_step", 2)));
    distance_percentile_ =
        std::clamp(declare_parameter<double>("distance_percentile", 0.20),
                   0.0, 1.0);
    min_range_m_ = declare_parameter<double>("min_range_m", 0.5);
    max_range_m_ = declare_parameter<double>("max_range_m", 35.0);
    max_depth_stamp_delta_s_ =
        declare_parameter<double>("max_depth_stamp_delta_s", 1.0);
    max_abs_lateral_m_ =
        std::max(0.0, declare_parameter<double>("max_abs_lateral_m", 2.8));
    fallback_focal_length_ratio_ =
        std::max(1e-3, declare_parameter<double>(
                           "fallback_focal_length_ratio", 0.375));

    detected_pub_ =
        create_publisher<std_msgs::msg::Bool>(detected_topic_, 10);
    distance_pub_ =
        create_publisher<std_msgs::msg::Float32>(distance_topic_, 10);
    debug_pub_ =
        create_publisher<std_msgs::msg::String>(debug_topic_, 10);

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SemanticVehicleDetectorNode::OnDepth, this,
                  std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SemanticVehicleDetectorNode::OnCameraInfo, this,
                  std::placeholders::_1));
    semantic_sub_ = create_subscription<sensor_msgs::msg::Image>(
        semantic_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SemanticVehicleDetectorNode::OnSemantic, this,
                  std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Semantic vehicle detector started: semantic=%s depth=%s camera_info=%s "
        "vehicle_rgb=(%d,%d,%d) tolerance=%d",
        semantic_topic_.c_str(), depth_topic_.c_str(),
        camera_info_topic_.c_str(),
        semantic_r_, semantic_g_, semantic_b_, semantic_tolerance_);
    RCLCPP_INFO(
        get_logger(),
        "Diagnostic outputs only: detected=%s distance=%s debug=%s",
        detected_topic_.c_str(), distance_topic_.c_str(),
        debug_topic_.c_str());
  }

private:
  void OnDepth(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!IsDepthEncodingSupported(msg->encoding)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Unsupported depth encoding '%s'. Supported: 16UC1, mono16, 32FC1.",
          msg->encoding.c_str());
      return;
    }
    latest_depth_ = msg;
  }

  void OnCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->width == 0 || msg->height == 0 ||
        !std::isfinite(msg->k[0]) || msg->k[0] <= 0.0 ||
        !std::isfinite(msg->k[2])) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring invalid depth CameraInfo.");
      return;
    }
    latest_camera_info_ = msg;
  }

  void OnSemantic(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!IsColorEncodingSupported(msg->encoding)) {
      PublishNoDetection(
          "status=unsupported_semantic_encoding encoding=" + msg->encoding);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Unsupported semantic encoding '%s'. Supported: rgb8, bgr8, rgba8, bgra8.",
          msg->encoding.c_str());
      return;
    }

    const Roi roi = BuildRoi(*msg);
    if (roi.x_max <= roi.x_min || roi.y_max <= roi.y_min) {
      PublishNoDetection("status=invalid_roi");
      return;
    }

    int raw_red_pixels = 0;
    int rejected_small = 0;
    int rejected_large = 0;
    std::vector<Component> components =
        ExtractComponents(*msg, roi, raw_red_pixels,
                          rejected_small, rejected_large);

    double stamp_delta_s = std::numeric_limits<double>::infinity();
    std::string depth_status = "waiting_for_depth";
    bool depth_is_usable = false;
    if (latest_depth_) {
      stamp_delta_s = std::abs(
          StampSeconds(msg->header.stamp) -
          StampSeconds(latest_depth_->header.stamp));
      if (!IsDepthEncodingSupported(latest_depth_->encoding)) {
        depth_status = "unsupported_depth_encoding";
      } else if (latest_depth_->width == 0 || latest_depth_->height == 0) {
        depth_status = "invalid_depth_size";
      } else if (max_depth_stamp_delta_s_ > 0.0 &&
                 stamp_delta_s > max_depth_stamp_delta_s_) {
        depth_status = "stale_depth";
      } else {
        depth_status = "ok";
        depth_is_usable = true;
      }
    }

    if (depth_is_usable) {
      for (auto &component : components) {
        FuseDepth(*msg, *latest_depth_, component);
      }
    }

    const Component *nearest = nullptr;
    int lateral_corridor_components = 0;
    for (const auto &component : components) {
      if (!component.in_lateral_corridor ||
          !std::isfinite(component.distance_m)) {
        continue;
      }
      ++lateral_corridor_components;
      if (nearest == nullptr ||
          component.distance_m < nearest->distance_m) {
        nearest = &component;
      }
    }
    const bool detected = nearest != nullptr;
    const double distance_m = detected
        ? nearest->distance_m
        : std::numeric_limits<double>::infinity();

    std_msgs::msg::Bool detected_msg;
    detected_msg.data = detected;
    detected_pub_->publish(detected_msg);

    std_msgs::msg::Float32 distance_msg;
    distance_msg.data = static_cast<float>(distance_m);
    distance_pub_->publish(distance_msg);

    std_msgs::msg::String debug_msg;
    debug_msg.data = BuildDebug(
        *msg, roi, components, detected, distance_m, depth_status,
        stamp_delta_s, raw_red_pixels, rejected_small, rejected_large,
        lateral_corridor_components);
    debug_pub_->publish(debug_msg);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "%s", debug_msg.data.c_str());
  }

  Roi BuildRoi(const sensor_msgs::msg::Image &image) const {
    const auto clamp01 = [](double value) {
      return std::clamp(value, 0.0, 1.0);
    };
    Roi roi;
    roi.x_min = std::clamp(
        static_cast<int>(std::floor(clamp01(roi_x_min_) * image.width)),
        0, static_cast<int>(image.width));
    roi.y_min = std::clamp(
        static_cast<int>(std::floor(clamp01(roi_y_min_) * image.height)),
        0, static_cast<int>(image.height));
    roi.x_max = std::clamp(
        static_cast<int>(std::ceil(clamp01(roi_x_max_) * image.width)),
        0, static_cast<int>(image.width));
    roi.y_max = std::clamp(
        static_cast<int>(std::ceil(clamp01(roi_y_max_) * image.height)),
        0, static_cast<int>(image.height));
    return roi;
  }

  bool IsVehicleColor(const Rgb &rgb) const {
    return std::abs(static_cast<int>(rgb.r) - semantic_r_) <=
               semantic_tolerance_ &&
           std::abs(static_cast<int>(rgb.g) - semantic_g_) <=
               semantic_tolerance_ &&
           std::abs(static_cast<int>(rgb.b) - semantic_b_) <=
               semantic_tolerance_;
  }

  std::vector<Component> ExtractComponents(
      const sensor_msgs::msg::Image &image, const Roi &roi,
      int &raw_red_pixels, int &rejected_small, int &rejected_large) const {
    const int width = roi.x_max - roi.x_min;
    const int height = roi.y_max - roi.y_min;
    std::vector<uint8_t> mask(
        static_cast<size_t>(width * height), 0);
    raw_red_pixels = 0;

    for (int y = roi.y_min; y < roi.y_max; ++y) {
      for (int x = roi.x_min; x < roi.x_max; ++x) {
        Rgb rgb;
        if (!ReadRgb(image, x, y, rgb) || !IsVehicleColor(rgb)) {
          continue;
        }
        const int local_index =
            (y - roi.y_min) * width + (x - roi.x_min);
        mask[static_cast<size_t>(local_index)] = 1;
        ++raw_red_pixels;
      }
    }

    std::vector<Component> components;
    std::vector<uint8_t> visited(mask.size(), 0);
    std::vector<int> stack;
    stack.reserve(1024);
    rejected_small = 0;
    rejected_large = 0;

    for (int local_y = 0; local_y < height; ++local_y) {
      for (int local_x = 0; local_x < width; ++local_x) {
        const int start = local_y * width + local_x;
        if (!mask[static_cast<size_t>(start)] ||
            visited[static_cast<size_t>(start)]) {
          continue;
        }

        Component component;
        component.min_x = component.max_x = local_x + roi.x_min;
        component.min_y = component.max_y = local_y + roi.y_min;
        stack.clear();
        stack.push_back(start);
        visited[static_cast<size_t>(start)] = 1;

        while (!stack.empty()) {
          const int current = stack.back();
          stack.pop_back();
          const int current_x = current % width;
          const int current_y = current / width;
          const int image_x = current_x + roi.x_min;
          const int image_y = current_y + roi.y_min;

          ++component.pixels;
          component.min_x = std::min(component.min_x, image_x);
          component.max_x = std::max(component.max_x, image_x);
          component.min_y = std::min(component.min_y, image_y);
          component.max_y = std::max(component.max_y, image_y);
          component.image_indices.push_back(
              image_y * static_cast<int>(image.width) + image_x);

          AddNeighbor(current_x - 1, current_y, width, height,
                      mask, visited, stack);
          AddNeighbor(current_x + 1, current_y, width, height,
                      mask, visited, stack);
          AddNeighbor(current_x, current_y - 1, width, height,
                      mask, visited, stack);
          AddNeighbor(current_x, current_y + 1, width, height,
                      mask, visited, stack);
        }

        if (component.pixels < min_component_pixels_) {
          ++rejected_small;
        } else if (component.pixels > max_component_pixels_) {
          ++rejected_large;
        } else {
          components.push_back(std::move(component));
        }
      }
    }
    return components;
  }

  void FuseDepth(const sensor_msgs::msg::Image &semantic,
                 const sensor_msgs::msg::Image &depth,
                 Component &component) const {
    const double scale_x =
        static_cast<double>(depth.width) /
        static_cast<double>(semantic.width);
    const double scale_y =
        static_cast<double>(depth.height) /
        static_cast<double>(semantic.height);
    std::vector<double> distances;
    distances.reserve(component.image_indices.size() /
                      static_cast<size_t>(depth_sample_step_) + 1U);

    for (size_t i = 0; i < component.image_indices.size();
         i += static_cast<size_t>(depth_sample_step_)) {
      const int semantic_index = component.image_indices[i];
      const int semantic_x =
          semantic_index % static_cast<int>(semantic.width);
      const int semantic_y =
          semantic_index / static_cast<int>(semantic.width);
      const int depth_x = std::clamp(
          static_cast<int>(std::floor(
              (static_cast<double>(semantic_x) + 0.5) * scale_x)),
          0, static_cast<int>(depth.width) - 1);
      const int depth_y = std::clamp(
          static_cast<int>(std::floor(
              (static_cast<double>(semantic_y) + 0.5) * scale_y)),
          0, static_cast<int>(depth.height) - 1);
      const double distance_m = ReadDepthMeters(depth, depth_x, depth_y);
      if (std::isfinite(distance_m) &&
          distance_m >= min_range_m_ &&
          distance_m <= max_range_m_) {
        distances.push_back(distance_m);
      }
    }

    component.valid_depth_pixels = static_cast<int>(distances.size());
    if (component.valid_depth_pixels < min_valid_depth_pixels_) {
      return;
    }
    std::sort(distances.begin(), distances.end());
    const size_t percentile_index = std::min(
        distances.size() - 1U,
        static_cast<size_t>(std::floor(
            distance_percentile_ *
            static_cast<double>(distances.size() - 1U))));
    component.distance_m = distances[percentile_index];

    double fx =
        fallback_focal_length_ratio_ * static_cast<double>(depth.width);
    double cx = 0.5 * static_cast<double>(depth.width);
    if (latest_camera_info_ &&
        latest_camera_info_->width > 0 &&
        latest_camera_info_->k[0] > 0.0) {
      const double info_scale =
          static_cast<double>(depth.width) /
          static_cast<double>(latest_camera_info_->width);
      fx = latest_camera_info_->k[0] * info_scale;
      cx = latest_camera_info_->k[2] * info_scale;
    }
    const double semantic_center_x =
        0.5 * static_cast<double>(component.min_x + component.max_x);
    const double depth_center_x =
        (semantic_center_x + 0.5) * scale_x;
    component.lateral_m =
        (depth_center_x - cx) * component.distance_m / fx;
    component.in_lateral_corridor =
        std::abs(component.lateral_m) <= max_abs_lateral_m_;
  }

  std::string BuildDebug(
      const sensor_msgs::msg::Image &semantic, const Roi &roi,
      const std::vector<Component> &components, bool detected,
      double distance_m, const std::string &depth_status,
      double stamp_delta_s, int raw_red_pixels,
      int rejected_small, int rejected_large,
      int lateral_corridor_components) const {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "detected=" << (detected ? "true" : "false")
        << " distance_m=";
    if (std::isfinite(distance_m)) {
      out << distance_m;
    } else {
      out << "inf";
    }
    out << " depth_status=" << depth_status
        << " depth_stamp_delta_s=";
    if (std::isfinite(stamp_delta_s)) {
      out << stamp_delta_s;
    } else {
      out << "inf";
    }
    out << " semantic_size=" << semantic.width << "x" << semantic.height
        << " encoding=" << semantic.encoding
        << " roi=[" << roi.x_min << "," << roi.y_min
        << "," << roi.x_max << "," << roi.y_max << "]"
        << " raw_red_pixels=" << raw_red_pixels
        << " accepted_components=" << components.size()
        << " lateral_corridor_components="
        << lateral_corridor_components
        << " max_abs_lateral_m=" << max_abs_lateral_m_
        << " rejected_small=" << rejected_small
        << " rejected_large=" << rejected_large
        << " components=[";
    for (size_t i = 0; i < components.size(); ++i) {
      if (i > 0) {
        out << ";";
      }
      const auto &component = components[i];
      out << i
          << ":bbox(" << component.min_x << "," << component.min_y
          << "," << component.max_x << "," << component.max_y << ")"
          << ",px=" << component.pixels
          << ",depth_px=" << component.valid_depth_pixels
          << ",d=";
      if (std::isfinite(component.distance_m)) {
        out << component.distance_m;
      } else {
        out << "inf";
      }
      out << ",lateral=";
      if (std::isfinite(component.lateral_m)) {
        out << component.lateral_m;
      } else {
        out << "inf";
      }
      out << ",in_corridor="
          << (component.in_lateral_corridor ? "true" : "false");
    }
    out << "]";
    return out.str();
  }

  void PublishNoDetection(const std::string &debug) {
    std_msgs::msg::Bool detected_msg;
    detected_msg.data = false;
    detected_pub_->publish(detected_msg);
    std_msgs::msg::Float32 distance_msg;
    distance_msg.data = std::numeric_limits<float>::infinity();
    distance_pub_->publish(distance_msg);
    std_msgs::msg::String debug_msg;
    debug_msg.data = debug;
    debug_pub_->publish(debug_msg);
  }

  std::string semantic_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string detected_topic_;
  std::string distance_topic_;
  std::string debug_topic_;

  int semantic_r_{255};
  int semantic_g_{0};
  int semantic_b_{0};
  int semantic_tolerance_{45};
  double roi_x_min_{0.12};
  double roi_y_min_{0.10};
  double roi_x_max_{0.88};
  double roi_y_max_{0.98};
  int min_component_pixels_{80};
  int max_component_pixels_{200000};
  int min_valid_depth_pixels_{20};
  int depth_sample_step_{2};
  double distance_percentile_{0.20};
  double min_range_m_{0.5};
  double max_range_m_{35.0};
  double max_depth_stamp_delta_s_{1.0};
  double max_abs_lateral_m_{2.8};
  double fallback_focal_length_ratio_{0.375};

  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr semantic_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticVehicleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
