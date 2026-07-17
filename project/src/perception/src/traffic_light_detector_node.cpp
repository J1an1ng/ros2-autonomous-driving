#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace {

enum TrafficLightState : uint8_t {
  UNKNOWN = 0,
  RED = 1,
  YELLOW = 2,
  GREEN = 3,
};

std::string StateName(uint8_t state) {
  switch (state) {
    case RED:
      return "RED";
    case YELLOW:
      return "YELLOW";
    case GREEN:
      return "GREEN";
    default:
      return "UNKNOWN";
  }
}

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

struct Hsv {
  double h_deg;
  double s;
  double v;
};

Hsv RgbToHsv(const Rgb &rgb) {
  const double r = static_cast<double>(rgb.r) / 255.0;
  const double g = static_cast<double>(rgb.g) / 255.0;
  const double b = static_cast<double>(rgb.b) / 255.0;

  const double max_value = std::max({r, g, b});
  const double min_value = std::min({r, g, b});
  const double delta = max_value - min_value;

  double h = 0.0;
  if (delta > 1e-9) {
    if (max_value == r) {
      h = 60.0 * std::fmod(((g - b) / delta), 6.0);
    } else if (max_value == g) {
      h = 60.0 * (((b - r) / delta) + 2.0);
    } else {
      h = 60.0 * (((r - g) / delta) + 4.0);
    }
  }
  if (h < 0.0) {
    h += 360.0;
  }

  const double s = max_value <= 1e-9 ? 0.0 : delta / max_value;
  return Hsv{h, s * 255.0, max_value * 255.0};
}

struct Score {
  int accepted_pixels{0};
  int accepted_blobs{0};
  int max_blob_pixels{0};
};

int WeightedScore(const Score &score) {
  return score.accepted_pixels + 3 * score.max_blob_pixels;
}

struct Roi {
  int x_min{0};
  int y_min{0};
  int x_max{0};
  int y_max{0};
};

struct SemanticComponent {
  int pixels{0};
  int min_x{0};
  int max_x{0};
  int min_y{0};
  int max_y{0};
  std::vector<int> indices;
};

}  // namespace

class TrafficLightDetectorNode : public rclcpp::Node {
public:
  TrafficLightDetectorNode()
      : Node("traffic_light_detector_node") {
    image_topic_ = declare_parameter<std::string>(
        "image_topic", "/OurCar/Sensors/RGBCameraLeft/image_raw");
    semantic_topic_ = declare_parameter<std::string>(
        "semantic_topic", "/OurCar/Sensors/SemanticCamera/image_raw");
    state_topic_ = declare_parameter<std::string>(
        "state_topic", "/perception/traffic_light_state");
    debug_topic_ = declare_parameter<std::string>(
        "debug_topic", "/perception/traffic_light_debug");

    use_semantic_gate_ = declare_parameter<bool>("use_semantic_gate", true);
    semantic_r_ = declare_parameter<int>("semantic_r", 255);
    semantic_g_ = declare_parameter<int>("semantic_g", 255);
    semantic_b_ = declare_parameter<int>("semantic_b", 0);
    semantic_tolerance_ = declare_parameter<int>("semantic_tolerance", 45);
    semantic_use_hsv_ = declare_parameter<bool>("semantic_use_hsv", true);
    semantic_h_min_ = declare_parameter<double>("semantic_h_min", 30.0);
    semantic_h_max_ = declare_parameter<double>("semantic_h_max", 80.0);
    semantic_min_saturation_ = declare_parameter<int>("semantic_min_saturation", 60);
    semantic_min_value_ = declare_parameter<int>("semantic_min_value", 80);
    semantic_debug_colors_ = declare_parameter<bool>("semantic_debug_colors", false);
    max_semantic_stamp_delta_s_ =
        declare_parameter<double>("max_semantic_stamp_delta_s", 1.0);
    min_semantic_blob_pixels_ = declare_parameter<int>("min_semantic_blob_pixels", 20);
    max_semantic_blob_pixels_ = declare_parameter<int>("max_semantic_blob_pixels", 2500);
    semantic_merge_gap_px_ = declare_parameter<int>("semantic_merge_gap_px", 12);
    target_light_ = declare_parameter<std::string>("target_light", "strict_center");
    center_distance_penalty_ = declare_parameter<double>("center_distance_penalty", 0.8);

    roi_x_min_ = declare_parameter<double>("roi_x_min", 0.0);
    roi_y_min_ = declare_parameter<double>("roi_y_min", 0.0);
    roi_x_max_ = declare_parameter<double>("roi_x_max", 1.0);
    roi_y_max_ = declare_parameter<double>("roi_y_max", 0.55);

    min_saturation_ = declare_parameter<int>("min_saturation", 80);
    red_min_saturation_ = declare_parameter<int>("red_min_saturation", 70);
    red_min_value_ = declare_parameter<int>("red_min_value", 100);
    yellow_min_saturation_ = declare_parameter<int>("yellow_min_saturation", 110);
    yellow_min_value_ = declare_parameter<int>("yellow_min_value", 175);
    green_min_saturation_ = declare_parameter<int>("green_min_saturation", 45);
    green_min_value_ = declare_parameter<int>("green_min_value", 70);
    red_h_high_ = declare_parameter<double>("red_h_high", 25.0);
    red_h_low_ = declare_parameter<double>("red_h_low", 330.0);
    yellow_h_min_ = declare_parameter<double>("yellow_h_min", 35.0);
    yellow_h_max_ = declare_parameter<double>("yellow_h_max", 65.0);
    green_h_min_ = declare_parameter<double>("green_h_min", 60.0);
    green_h_max_ = declare_parameter<double>("green_h_max", 190.0);
    min_blob_pixels_ = declare_parameter<int>("min_blob_pixels", 3);
    max_blob_pixels_ = declare_parameter<int>("max_blob_pixels", 900);
    max_red_blob_pixels_ = declare_parameter<int>("max_red_blob_pixels", 180);
    max_yellow_blob_pixels_ = declare_parameter<int>("max_yellow_blob_pixels", 120);
    max_green_blob_pixels_ = declare_parameter<int>("max_green_blob_pixels", 90);
    min_score_pixels_ = declare_parameter<int>("min_score_pixels", 6);
    min_red_score_ = declare_parameter<int>("min_red_score", 6);
    min_red_pixels_ = declare_parameter<int>("min_red_pixels", 6);
    min_yellow_score_ = declare_parameter<int>("min_yellow_score", 16);
    min_yellow_pixels_ = declare_parameter<int>("min_yellow_pixels", 8);
    min_green_score_ = declare_parameter<int>("min_green_score", 3);
    yellow_required_ratio_ = declare_parameter<double>("yellow_required_ratio", 1.8);
    yellow_over_green_ratio_ = declare_parameter<double>("yellow_over_green_ratio", 0.75);
    green_over_red_ratio_ = declare_parameter<double>("green_over_red_ratio", 4.0);
    consecutive_frames_ = declare_parameter<int>("consecutive_frames", 4);

    state_pub_ = create_publisher<std_msgs::msg::UInt8>(state_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, 10);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&TrafficLightDetectorNode::OnImage, this, std::placeholders::_1));

    if (use_semantic_gate_) {
      semantic_sub_ = create_subscription<sensor_msgs::msg::Image>(
          semantic_topic_, rclcpp::SensorDataQoS(),
          std::bind(&TrafficLightDetectorNode::OnSemanticImage, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(), "Traffic light detector started.");
    RCLCPP_INFO(get_logger(), "Subscribing image: %s", image_topic_.c_str());
    if (use_semantic_gate_) {
      RCLCPP_INFO(get_logger(), "Using semantic gate from: %s", semantic_topic_.c_str());
      RCLCPP_INFO(get_logger(),
                  "Semantic traffic-light color: HSV yellow gate=%s, H=[%.1f, %.1f], S>=%d, V>=%d. "
                  "RGB fallback=(%d,%d,%d), tolerance=%d",
                  semantic_use_hsv_ ? "true" : "false",
                  semantic_h_min_, semantic_h_max_,
                  semantic_min_saturation_, semantic_min_value_,
                  semantic_r_, semantic_g_, semantic_b_, semantic_tolerance_);
      RCLCPP_INFO(get_logger(), "Target traffic light selector: %s, center_distance_penalty=%.2f",
                  target_light_.c_str(), center_distance_penalty_);
    }
    RCLCPP_INFO(get_logger(), "Publishing state: %s (0 UNKNOWN, 1 RED, 2 YELLOW, 3 GREEN)",
                state_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing debug: %s", debug_topic_.c_str());
  }

private:
  void OnSemanticImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!IsSupportedEncoding(msg->encoding)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Unsupported semantic image encoding '%s'. Supported: rgb8, bgr8, rgba8, bgra8.",
          msg->encoding.c_str());
      return;
    }
    latest_semantic_ = msg;
  }

  void OnImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!IsSupportedEncoding(msg->encoding)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Unsupported image encoding '%s'. Supported: rgb8, bgr8, rgba8, bgra8.",
          msg->encoding.c_str());
      return;
    }

    const Roi roi = BuildRoi(*msg);
    if (roi.x_max <= roi.x_min || roi.y_max <= roi.y_min) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid ROI.");
      return;
    }

    std::vector<uint8_t> semantic_gate;
    std::string gate_debug;
    if (!BuildSemanticGate(*msg, roi, semantic_gate, gate_debug)) {
      PublishStateAndDebug(*msg, roi, UNKNOWN, UNKNOWN, Score{}, Score{}, Score{}, gate_debug);
      return;
    }

    std::vector<uint8_t> red_mask;
    std::vector<uint8_t> yellow_mask;
    std::vector<uint8_t> green_mask;
    BuildMasks(*msg, roi, semantic_gate, red_mask, yellow_mask, green_mask);

    const int roi_width = roi.x_max - roi.x_min;
    const int roi_height = roi.y_max - roi.y_min;
    const Score red = ScoreMask(red_mask, roi_width, roi_height, max_red_blob_pixels_);
    const Score yellow = ScoreMask(yellow_mask, roi_width, roi_height, max_yellow_blob_pixels_);
    const Score green = ScoreMask(green_mask, roi_width, roi_height, max_green_blob_pixels_);

    const uint8_t raw_state = ChooseState(red, yellow, green);
    const uint8_t stable_state = UpdateStableState(raw_state);

    PublishStateAndDebug(*msg, roi, raw_state, stable_state, red, yellow, green, gate_debug);
  }

  void PublishStateAndDebug(const sensor_msgs::msg::Image &msg,
                            const Roi &roi,
                            uint8_t raw_state,
                            uint8_t stable_state,
                            const Score &red,
                            const Score &yellow,
                            const Score &green,
                            const std::string &gate_debug) {
    std_msgs::msg::UInt8 state_msg;
    state_msg.data = stable_state;
    state_pub_->publish(state_msg);

    std_msgs::msg::String debug_msg;
    debug_msg.data = BuildDebugString(msg, roi, raw_state, stable_state, red, yellow, green, gate_debug);
    debug_pub_->publish(debug_msg);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s", debug_msg.data.c_str());
  }

  bool IsSupportedEncoding(const std::string &encoding) const {
    return encoding == "rgb8" || encoding == "bgr8" ||
           encoding == "rgba8" || encoding == "bgra8";
  }

  Roi BuildRoi(const sensor_msgs::msg::Image &msg) const {
    const auto clamp01 = [](double value) {
      return std::clamp(value, 0.0, 1.0);
    };

    Roi roi;
    roi.x_min = static_cast<int>(std::floor(clamp01(roi_x_min_) * msg.width));
    roi.y_min = static_cast<int>(std::floor(clamp01(roi_y_min_) * msg.height));
    roi.x_max = static_cast<int>(std::ceil(clamp01(roi_x_max_) * msg.width));
    roi.y_max = static_cast<int>(std::ceil(clamp01(roi_y_max_) * msg.height));

    roi.x_min = std::clamp(roi.x_min, 0, static_cast<int>(msg.width));
    roi.x_max = std::clamp(roi.x_max, 0, static_cast<int>(msg.width));
    roi.y_min = std::clamp(roi.y_min, 0, static_cast<int>(msg.height));
    roi.y_max = std::clamp(roi.y_max, 0, static_cast<int>(msg.height));
    return roi;
  }

  bool BuildSemanticGate(const sensor_msgs::msg::Image &rgb_msg,
                         const Roi &roi,
                         std::vector<uint8_t> &gate,
                         std::string &gate_debug) const {
    const int width = roi.x_max - roi.x_min;
    const int height = roi.y_max - roi.y_min;
    gate.assign(static_cast<size_t>(width * height), use_semantic_gate_ ? 0 : 1);

    if (!use_semantic_gate_) {
      gate_debug = "semantic_gate=disabled";
      return true;
    }

    if (!latest_semantic_) {
      gate_debug = "semantic_gate=waiting_for_semantic";
      return false;
    }

    const auto &semantic = *latest_semantic_;
    const double rgb_stamp =
        static_cast<double>(rgb_msg.header.stamp.sec) +
        static_cast<double>(rgb_msg.header.stamp.nanosec) * 1e-9;
    const double semantic_stamp =
        static_cast<double>(semantic.header.stamp.sec) +
        static_cast<double>(semantic.header.stamp.nanosec) * 1e-9;
    const double stamp_delta = std::abs(rgb_stamp - semantic_stamp);
    if (max_semantic_stamp_delta_s_ > 0.0 &&
        stamp_delta > max_semantic_stamp_delta_s_) {
      std::ostringstream out;
      out << "semantic_gate=stale"
          << " semantic_stamp_delta_s=" << stamp_delta
          << " max_semantic_stamp_delta_s=" << max_semantic_stamp_delta_s_;
      gate_debug = out.str();
      return false;
    }
    const double semantic_scale_x =
        static_cast<double>(semantic.width) / static_cast<double>(rgb_msg.width);
    const double semantic_scale_y =
        static_cast<double>(semantic.height) / static_cast<double>(rgb_msg.height);

    std::vector<uint8_t> semantic_mask(static_cast<size_t>(width * height), 0);
    int raw_semantic_pixels = 0;
    std::map<int, int> color_histogram;
    for (int y = roi.y_min; y < roi.y_max; ++y) {
      for (int x = roi.x_min; x < roi.x_max; ++x) {
        const int semantic_x = std::clamp(
            static_cast<int>(std::floor((static_cast<double>(x) + 0.5) * semantic_scale_x)),
            0, static_cast<int>(semantic.width) - 1);
        const int semantic_y = std::clamp(
            static_cast<int>(std::floor((static_cast<double>(y) + 0.5) * semantic_scale_y)),
            0, static_cast<int>(semantic.height) - 1);
        const Rgb pixel = ReadPixel(semantic, semantic_x, semantic_y);
        if (semantic_debug_colors_) {
          AccumulateSemanticDebugColor(pixel, color_histogram);
        }
        if (IsSemanticTrafficLightColor(pixel)) {
          const int local_x = x - roi.x_min;
          const int local_y = y - roi.y_min;
          semantic_mask[static_cast<size_t>(local_y * width + local_x)] = 1;
          ++raw_semantic_pixels;
        }
      }
    }

    std::vector<uint8_t> visited(semantic_mask.size(), 0);
    std::vector<int> stack;
    stack.reserve(256);

    int accepted_components = 0;
    int accepted_pixels = 0;
    int rejected_small_components = 0;
    int rejected_large_components = 0;
    std::vector<SemanticComponent> components;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int start = y * width + x;
        if (!semantic_mask[start] || visited[start]) {
          continue;
        }

        int blob_pixels = 0;
        int min_x = x;
        int max_x = x;
        int min_y = y;
        int max_y = y;
        std::vector<int> component_indices;
        stack.clear();
        stack.push_back(start);
        visited[start] = 1;

        while (!stack.empty()) {
          const int current = stack.back();
          stack.pop_back();
          ++blob_pixels;
          component_indices.push_back(current);

          const int cx = current % width;
          const int cy = current / width;
          min_x = std::min(min_x, cx);
          max_x = std::max(max_x, cx);
          min_y = std::min(min_y, cy);
          max_y = std::max(max_y, cy);

          AddNeighbor(cx - 1, cy, width, height, semantic_mask, visited, stack);
          AddNeighbor(cx + 1, cy, width, height, semantic_mask, visited, stack);
          AddNeighbor(cx, cy - 1, width, height, semantic_mask, visited, stack);
          AddNeighbor(cx, cy + 1, width, height, semantic_mask, visited, stack);
        }

        if (blob_pixels < min_semantic_blob_pixels_ ||
            blob_pixels > max_semantic_blob_pixels_) {
          if (blob_pixels < min_semantic_blob_pixels_) {
            ++rejected_small_components;
          } else {
            ++rejected_large_components;
          }
          continue;
        }

        components.push_back(
            SemanticComponent{blob_pixels, min_x, max_x, min_y, max_y, std::move(component_indices)});
        ++accepted_components;
        accepted_pixels += blob_pixels;
      }
    }
    MergeSemanticComponents(components);

    const int selected_component = SelectSemanticComponent(components, width);
    if (selected_component >= 0) {
      AddSemanticComponentToGate(components[static_cast<size_t>(selected_component)],
                                 width, height, gate);
    } else if (target_light_ == "all") {
      for (const auto &component : components) {
        AddSemanticComponentToGate(component, width, height, gate);
      }
    }

    std::ostringstream out;
    out << "semantic_gate=enabled"
        << " semantic_size=" << semantic.width << "x" << semantic.height
        << " semantic_stamp_delta_s=" << stamp_delta
        << " semantic_scale=" << semantic_scale_x << "x" << semantic_scale_y
        << " semantic_raw_pixels=" << raw_semantic_pixels
        << " semantic_components=" << accepted_components
        << " semantic_pixels=" << accepted_pixels
        << " semantic_components_after_merge=" << components.size()
        << " target_light=" << target_light_
        << " selected_component=" << selected_component
        << " " << BuildSemanticComponentsDebug(components);
    if (semantic_debug_colors_) {
      out << " " << BuildSemanticColorDebug(color_histogram);
    }
    if (accepted_components == 0) {
      out << " semantic_rejected_small=" << rejected_small_components
          << " semantic_rejected_large=" << rejected_large_components
          << " no_valid_candidate";
    }
    gate_debug = out.str();
    return accepted_components > 0;
  }

  void AddSemanticComponentToGate(const SemanticComponent &component,
                                  int width,
                                  int height,
                                  std::vector<uint8_t> &gate) const {
    for (const int index : component.indices) {
      if (index < 0 || index >= width * height) {
        continue;
      }
      gate[static_cast<size_t>(index)] = 1;
    }
  }

  void MergeSemanticComponents(std::vector<SemanticComponent> &components) const {
    bool merged = true;
    while (merged) {
      merged = false;
      for (size_t i = 0; i < components.size() && !merged; ++i) {
        for (size_t j = i + 1; j < components.size(); ++j) {
          if (!ShouldMergeComponents(components[i], components[j])) {
            continue;
          }
          components[i].pixels += components[j].pixels;
          components[i].min_x = std::min(components[i].min_x, components[j].min_x);
          components[i].max_x = std::max(components[i].max_x, components[j].max_x);
          components[i].min_y = std::min(components[i].min_y, components[j].min_y);
          components[i].max_y = std::max(components[i].max_y, components[j].max_y);
          components[i].indices.insert(components[i].indices.end(),
                                       components[j].indices.begin(),
                                       components[j].indices.end());
          components.erase(components.begin() + static_cast<std::ptrdiff_t>(j));
          merged = true;
          break;
        }
      }
    }
  }

  bool ShouldMergeComponents(const SemanticComponent &a, const SemanticComponent &b) const {
    const int horizontal_gap =
        std::max({0, b.min_x - a.max_x, a.min_x - b.max_x});
    const int vertical_gap =
        std::max({0, b.min_y - a.max_y, a.min_y - b.max_y});

    const bool close_boxes =
        horizontal_gap <= semantic_merge_gap_px_ &&
        vertical_gap <= semantic_merge_gap_px_;

    const int overlap_x =
        std::min(a.max_x, b.max_x) - std::max(a.min_x, b.min_x);
    const int overlap_y =
        std::min(a.max_y, b.max_y) - std::max(a.min_y, b.min_y);
    const bool aligned =
        overlap_x >= -semantic_merge_gap_px_ ||
        overlap_y >= -semantic_merge_gap_px_;

    return close_boxes && aligned;
  }

  int SelectSemanticComponent(const std::vector<SemanticComponent> &components, int width) const {
    if (components.empty() || target_light_ == "all") {
      return -1;
    }

    std::vector<int> indices(components.size());
    for (size_t i = 0; i < components.size(); ++i) {
      indices[i] = static_cast<int>(i);
    }

    if (target_light_ == "left" || target_light_ == "center" ||
        target_light_ == "strict_center" || target_light_ == "right") {
      std::sort(indices.begin(), indices.end(), [&components](int a, int b) {
        const double center_a = 0.5 * (components[static_cast<size_t>(a)].min_x +
                                       components[static_cast<size_t>(a)].max_x);
        const double center_b = 0.5 * (components[static_cast<size_t>(b)].min_x +
                                       components[static_cast<size_t>(b)].max_x);
        return center_a < center_b;
      });

      if (target_light_ == "left") {
        return indices.front();
      }
      if (target_light_ == "right") {
        return indices.back();
      }

      const double image_center = 0.5 * static_cast<double>(width);
      if (target_light_ == "strict_center") {
        return *std::min_element(indices.begin(), indices.end(), [&](int a, int b) {
          const double center_a = 0.5 * (components[static_cast<size_t>(a)].min_x +
                                         components[static_cast<size_t>(a)].max_x);
          const double center_b = 0.5 * (components[static_cast<size_t>(b)].min_x +
                                         components[static_cast<size_t>(b)].max_x);
          return std::abs(center_a - image_center) < std::abs(center_b - image_center);
        });
      }

      return *std::min_element(indices.begin(), indices.end(), [&](int a, int b) {
        const double center_a = 0.5 * (components[static_cast<size_t>(a)].min_x +
                                       components[static_cast<size_t>(a)].max_x);
        const double center_b = 0.5 * (components[static_cast<size_t>(b)].min_x +
                                       components[static_cast<size_t>(b)].max_x);
        const double score_a =
            static_cast<double>(components[static_cast<size_t>(a)].pixels) -
            center_distance_penalty_ * std::abs(center_a - image_center);
        const double score_b =
            static_cast<double>(components[static_cast<size_t>(b)].pixels) -
            center_distance_penalty_ * std::abs(center_b - image_center);
        return score_a > score_b;
      });
    }

    // Fallback/default: largest component.
    return static_cast<int>(
        std::distance(components.begin(),
                      std::max_element(components.begin(), components.end(),
                                       [](const auto &a, const auto &b) {
                                         return a.pixels < b.pixels;
                                       })));
  }

  std::string BuildSemanticComponentsDebug(const std::vector<SemanticComponent> &components) const {
    std::ostringstream out;
    out << "components=[";
    for (size_t i = 0; i < components.size(); ++i) {
      const auto &c = components[i];
      if (i > 0) {
        out << ";";
      }
      out << i << ":bbox(" << c.min_x << "," << c.min_y << "," << c.max_x << "," << c.max_y
          << "),px=" << c.pixels
          << ",cx=" << 0.5 * (c.min_x + c.max_x);
    }
    out << "]";
    return out.str();
  }

  bool IsSemanticTrafficLightColor(const Rgb &rgb) const {
    if (semantic_use_hsv_) {
      const Hsv hsv = RgbToHsv(rgb);
      return hsv.h_deg >= semantic_h_min_ &&
             hsv.h_deg <= semantic_h_max_ &&
             hsv.s >= semantic_min_saturation_ &&
             hsv.v >= semantic_min_value_;
    }

    return std::abs(static_cast<int>(rgb.r) - semantic_r_) <= semantic_tolerance_ &&
           std::abs(static_cast<int>(rgb.g) - semantic_g_) <= semantic_tolerance_ &&
           std::abs(static_cast<int>(rgb.b) - semantic_b_) <= semantic_tolerance_;
  }

  void AccumulateSemanticDebugColor(const Rgb &rgb, std::map<int, int> &histogram) const {
    const Hsv hsv = RgbToHsv(rgb);
    if (hsv.s < 30.0 || hsv.v < 30.0) {
      return;
    }

    // Quantize to 16-value buckets so tiny compression/noise differences still group together.
    const int qr = static_cast<int>(rgb.r) / 16;
    const int qg = static_cast<int>(rgb.g) / 16;
    const int qb = static_cast<int>(rgb.b) / 16;
    const int key = (qr << 8) | (qg << 4) | qb;
    ++histogram[key];
  }

  std::string BuildSemanticColorDebug(const std::map<int, int> &histogram) const {
    std::vector<std::pair<int, int>> colors;
    colors.reserve(histogram.size());
    for (const auto &[key, count] : histogram) {
      colors.emplace_back(key, count);
    }
    std::sort(colors.begin(), colors.end(), [](const auto &a, const auto &b) {
      return a.second > b.second;
    });

    std::ostringstream out;
    out << "semantic_top_colors=";
    const int max_colors = std::min<int>(5, colors.size());
    if (max_colors == 0) {
      out << "none";
      return out.str();
    }

    for (int i = 0; i < max_colors; ++i) {
      const int key = colors[i].first;
      const int qr = (key >> 8) & 0xF;
      const int qg = (key >> 4) & 0xF;
      const int qb = key & 0xF;
      const int r = qr * 16 + 8;
      const int g = qg * 16 + 8;
      const int b = qb * 16 + 8;
      if (i > 0) {
        out << ";";
      }
      out << "rgb(" << r << "," << g << "," << b << "):" << colors[i].second;
    }
    return out.str();
  }

  Rgb ReadPixel(const sensor_msgs::msg::Image &msg, int x, int y) const {
    const bool has_alpha = msg.encoding == "rgba8" || msg.encoding == "bgra8";
    const bool is_bgr = msg.encoding == "bgr8" || msg.encoding == "bgra8";
    const int channels = has_alpha ? 4 : 3;
    const size_t index = static_cast<size_t>(y) * msg.step + static_cast<size_t>(x) * channels;

    if (is_bgr) {
      return Rgb{msg.data[index + 2], msg.data[index + 1], msg.data[index + 0]};
    }
    return Rgb{msg.data[index + 0], msg.data[index + 1], msg.data[index + 2]};
  }

  void BuildMasks(const sensor_msgs::msg::Image &msg,
                  const Roi &roi,
                  const std::vector<uint8_t> &semantic_gate,
                  std::vector<uint8_t> &red_mask,
                  std::vector<uint8_t> &yellow_mask,
                  std::vector<uint8_t> &green_mask) const {
    const int width = roi.x_max - roi.x_min;
    const int height = roi.y_max - roi.y_min;
    red_mask.assign(static_cast<size_t>(width * height), 0);
    yellow_mask.assign(static_cast<size_t>(width * height), 0);
    green_mask.assign(static_cast<size_t>(width * height), 0);

    for (int y = roi.y_min; y < roi.y_max; ++y) {
      for (int x = roi.x_min; x < roi.x_max; ++x) {
        const int local_x = x - roi.x_min;
        const int local_y = y - roi.y_min;
        const size_t idx = static_cast<size_t>(local_y * width + local_x);
        if (!semantic_gate.empty() && !semantic_gate[idx]) {
          continue;
        }

        const Rgb rgb = ReadPixel(msg, x, y);
        const Hsv hsv = RgbToHsv(rgb);

        if (IsRed(rgb, hsv)) {
          red_mask[idx] = 1;
        } else if (IsYellow(rgb, hsv)) {
          yellow_mask[idx] = 1;
        } else if (IsGreen(rgb, hsv)) {
          green_mask[idx] = 1;
        }
      }
    }
  }

  bool PassesCommonColorGate(const Hsv &hsv, int min_saturation, int min_value) const {
    return hsv.s >= std::max(min_saturation_, min_saturation) &&
           hsv.v >= std::max(0, min_value);
  }

  bool IsRed(const Rgb &rgb, const Hsv &hsv) const {
    if (!PassesCommonColorGate(hsv, red_min_saturation_, red_min_value_)) {
      return false;
    }
    const bool hue_match = hsv.h_deg <= red_h_high_ || hsv.h_deg >= red_h_low_;
    const bool dominance = rgb.r > rgb.g + 18 && rgb.r > rgb.b + 18;
    return hue_match && dominance;
  }

  bool IsYellow(const Rgb &rgb, const Hsv &hsv) const {
    if (!PassesCommonColorGate(hsv, yellow_min_saturation_, yellow_min_value_)) {
      return false;
    }
    const bool hue_match = hsv.h_deg >= yellow_h_min_ && hsv.h_deg <= yellow_h_max_;
    const bool red_green_close = std::abs(static_cast<int>(rgb.r) - static_cast<int>(rgb.g)) < 65;
    const bool blue_low = rgb.b + 45 < std::max(rgb.r, rgb.g);
    return hue_match && red_green_close && blue_low;
  }

  bool IsGreen(const Rgb &rgb, const Hsv &hsv) const {
    if (!PassesCommonColorGate(hsv, green_min_saturation_, green_min_value_)) {
      return false;
    }
    const bool hue_match = hsv.h_deg >= green_h_min_ && hsv.h_deg <= green_h_max_;
    const bool dominance = rgb.g > rgb.r + 8 && rgb.g > rgb.b + 8;
    return hue_match && dominance;
  }

  Score ScoreMask(const std::vector<uint8_t> &mask,
                  int width,
                  int height,
                  int max_blob_pixels) const {
    Score score;
    std::vector<uint8_t> visited(mask.size(), 0);
    std::vector<int> stack;
    stack.reserve(256);

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int start = y * width + x;
        if (!mask[start] || visited[start]) {
          continue;
        }

        int blob_pixels = 0;
        stack.clear();
        stack.push_back(start);
        visited[start] = 1;

        while (!stack.empty()) {
          const int current = stack.back();
          stack.pop_back();
          ++blob_pixels;

          const int cx = current % width;
          const int cy = current / width;
          AddNeighbor(cx - 1, cy, width, height, mask, visited, stack);
          AddNeighbor(cx + 1, cy, width, height, mask, visited, stack);
          AddNeighbor(cx, cy - 1, width, height, mask, visited, stack);
          AddNeighbor(cx, cy + 1, width, height, mask, visited, stack);
        }

        if (blob_pixels >= min_blob_pixels_ &&
            blob_pixels <= std::min(max_blob_pixels_, max_blob_pixels)) {
          score.accepted_pixels += blob_pixels;
          ++score.accepted_blobs;
          score.max_blob_pixels = std::max(score.max_blob_pixels, blob_pixels);
        }
      }
    }

    return score;
  }

  void AddNeighbor(int x,
                   int y,
                   int width,
                   int height,
                   const std::vector<uint8_t> &mask,
                   std::vector<uint8_t> &visited,
                   std::vector<int> &stack) const {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      return;
    }
    const int idx = y * width + x;
    if (!mask[idx] || visited[idx]) {
      return;
    }
    visited[idx] = 1;
    stack.push_back(idx);
  }

  uint8_t ChooseState(const Score &red, const Score &yellow, const Score &green) const {
    const int red_score = WeightedScore(red);
    const int yellow_score = WeightedScore(yellow);
    const int green_score = WeightedScore(green);

    const bool red_valid =
        red_score >= std::max(min_score_pixels_, min_red_score_) &&
        red.accepted_pixels >= min_red_pixels_;
    const bool yellow_valid =
        yellow_score >= std::max(min_score_pixels_, min_yellow_score_) &&
        yellow.accepted_pixels >= min_yellow_pixels_;
    const bool green_valid = green_score >= std::max(min_score_pixels_, min_green_score_);

    if (!red_valid && !yellow_valid && !green_valid) {
      return UNKNOWN;
    }

    // Yellow pixels are easy to get from the traffic-light housing itself.
    // Only trust YELLOW when it clearly dominates red/green evidence.
    const int best_non_yellow = std::max(red_score, green_score);
    const bool yellow_dominates =
        yellow_valid &&
        static_cast<double>(yellow_score) >=
            yellow_required_ratio_ * static_cast<double>(std::max(1, best_non_yellow));

    const bool green_much_stronger_than_red =
        green_valid &&
        static_cast<double>(green_score) >=
            green_over_red_ratio_ * static_cast<double>(std::max(1, red_score));

    // Green road signs behind the traffic light can create persistent green evidence.
    // If a red bulb is visible, prefer RED unless green is overwhelmingly stronger.
    if (red_valid && !green_much_stronger_than_red && !yellow_dominates) {
      return RED;
    }

    const bool yellow_close_enough_to_green =
        yellow_valid &&
        (!green_valid ||
         static_cast<double>(yellow_score) >=
             yellow_over_green_ratio_ * static_cast<double>(std::max(1, green_score)));

    // Green road signs behind the traffic light can create persistent green
    // evidence. If there is no valid red bulb and the yellow bulb has a
    // comparable score, prefer YELLOW over that background green.
    if (!red_valid && yellow_close_enough_to_green) {
      return YELLOW;
    }

    if (green_valid && !yellow_dominates) {
      return GREEN;
    }

    if (yellow_dominates) {
      return YELLOW;
    }

    if (red_valid) {
      return RED;
    }
    if (green_valid) {
      return GREEN;
    }
    return UNKNOWN;
  }

  uint8_t UpdateStableState(uint8_t raw_state) {
    if (raw_state == candidate_state_) {
      ++candidate_count_;
    } else {
      candidate_state_ = raw_state;
      candidate_count_ = 1;
    }

    if (candidate_count_ >= std::max(1, consecutive_frames_)) {
      stable_state_ = candidate_state_;
    }
    return stable_state_;
  }

  std::string BuildDebugString(const sensor_msgs::msg::Image &msg,
                               const Roi &roi,
                               uint8_t raw_state,
                               uint8_t stable_state,
                               const Score &red,
                               const Score &yellow,
                               const Score &green,
                               const std::string &gate_debug) const {
    std::ostringstream out;
    out << "raw=" << StateName(raw_state)
        << " stable=" << StateName(stable_state)
        << " red_pixels=" << red.accepted_pixels
        << " red_blobs=" << red.accepted_blobs
        << " red_max_blob=" << red.max_blob_pixels
        << " red_score=" << WeightedScore(red)
        << " yellow_pixels=" << yellow.accepted_pixels
        << " yellow_blobs=" << yellow.accepted_blobs
        << " yellow_max_blob=" << yellow.max_blob_pixels
        << " yellow_score=" << WeightedScore(yellow)
        << " green_pixels=" << green.accepted_pixels
        << " green_blobs=" << green.accepted_blobs
        << " green_max_blob=" << green.max_blob_pixels
        << " green_score=" << WeightedScore(green)
        << " roi=[" << roi.x_min << "," << roi.y_min
        << "," << roi.x_max << "," << roi.y_max << "]"
        << " " << gate_debug
        << " image=" << msg.width << "x" << msg.height
        << " encoding=" << msg.encoding;
    return out.str();
  }

  std::string image_topic_;
  std::string semantic_topic_;
  std::string state_topic_;
  std::string debug_topic_;

  bool use_semantic_gate_{true};
  int semantic_r_{255};
  int semantic_g_{255};
  int semantic_b_{0};
  int semantic_tolerance_{45};
  bool semantic_use_hsv_{true};
  double semantic_h_min_{30.0};
  double semantic_h_max_{80.0};
  int semantic_min_saturation_{60};
  int semantic_min_value_{80};
  bool semantic_debug_colors_{false};
  double max_semantic_stamp_delta_s_{1.0};
  int min_semantic_blob_pixels_{20};
  int max_semantic_blob_pixels_{2500};
  int semantic_merge_gap_px_{12};
  std::string target_light_{"strict_center"};
  double center_distance_penalty_{0.8};

  double roi_x_min_{0.0};
  double roi_y_min_{0.0};
  double roi_x_max_{1.0};
  double roi_y_max_{0.55};

  int min_saturation_{80};
  int red_min_saturation_{70};
  int red_min_value_{100};
  int yellow_min_saturation_{110};
  int yellow_min_value_{175};
  int green_min_saturation_{45};
  int green_min_value_{70};
  double red_h_high_{25.0};
  double red_h_low_{330.0};
  double yellow_h_min_{35.0};
  double yellow_h_max_{65.0};
  double green_h_min_{60.0};
  double green_h_max_{190.0};
  int min_blob_pixels_{3};
  int max_blob_pixels_{900};
  int max_red_blob_pixels_{180};
  int max_yellow_blob_pixels_{120};
  int max_green_blob_pixels_{90};
  int min_score_pixels_{6};
  int min_red_score_{6};
  int min_red_pixels_{6};
  int min_yellow_score_{16};
  int min_yellow_pixels_{8};
  int min_green_score_{3};
  double yellow_required_ratio_{1.8};
  double yellow_over_green_ratio_{0.75};
  double green_over_red_ratio_{4.0};
  int consecutive_frames_{4};

  uint8_t candidate_state_{UNKNOWN};
  int candidate_count_{0};
  uint8_t stable_state_{UNKNOWN};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr semantic_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;

  sensor_msgs::msg::Image::SharedPtr latest_semantic_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrafficLightDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
