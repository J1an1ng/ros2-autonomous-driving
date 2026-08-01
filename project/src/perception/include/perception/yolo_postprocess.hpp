// Copyright 2026 Team 20

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace perception::yolo
{

constexpr std::uint8_t kUnknown = 0;
constexpr std::uint8_t kRed = 1;
constexpr std::uint8_t kYellow = 2;
constexpr std::uint8_t kGreen = 3;

struct LetterboxTransform
{
  float scale{1.0F};
  int pad_x{0};
  int pad_y{0};
  int input_width{0};
  int input_height{0};
  int image_width{0};
  int image_height{0};
};

struct Detection
{
  int class_id{-1};
  float confidence{0.0F};
  cv::Rect box;
};

struct TrafficLightScores
{
  int red{0};
  int yellow{0};
  int green{0};
  std::uint8_t state{kUnknown};
};

inline cv::Mat Letterbox(
  const cv::Mat & image,
  const int input_width,
  const int input_height,
  LetterboxTransform & transform)
{
  transform.input_width = std::max(1, input_width);
  transform.input_height = std::max(1, input_height);
  transform.image_width = image.cols;
  transform.image_height = image.rows;
  transform.scale = std::min(
    static_cast<float>(transform.input_width) /
    static_cast<float>(std::max(1, image.cols)),
    static_cast<float>(transform.input_height) /
    static_cast<float>(std::max(1, image.rows)));

  const int resized_width = std::max(
    1, static_cast<int>(std::round(image.cols * transform.scale)));
  const int resized_height = std::max(
    1, static_cast<int>(std::round(image.rows * transform.scale)));
  transform.pad_x = (transform.input_width - resized_width) / 2;
  transform.pad_y = (transform.input_height - resized_height) / 2;

  cv::Mat resized;
  cv::resize(image, resized, cv::Size(resized_width, resized_height));
  cv::Mat result(
    transform.input_height, transform.input_width, image.type(),
    cv::Scalar(114, 114, 114));
  resized.copyTo(
    result(cv::Rect(
      transform.pad_x, transform.pad_y, resized_width, resized_height)));
  return result;
}

inline cv::Rect MapBoxToImage(
  const float x_min,
  const float y_min,
  const float x_max,
  const float y_max,
  const LetterboxTransform & transform)
{
  const float inverse_scale =
    1.0F / std::max(1.0e-6F, transform.scale);
  const int left = static_cast<int>(std::floor(
      (x_min - transform.pad_x) * inverse_scale));
  const int top = static_cast<int>(std::floor(
      (y_min - transform.pad_y) * inverse_scale));
  const int right = static_cast<int>(std::ceil(
      (x_max - transform.pad_x) * inverse_scale));
  const int bottom = static_cast<int>(std::ceil(
      (y_max - transform.pad_y) * inverse_scale));

  const int clipped_left = std::clamp(left, 0, transform.image_width);
  const int clipped_top = std::clamp(top, 0, transform.image_height);
  const int clipped_right = std::clamp(right, 0, transform.image_width);
  const int clipped_bottom = std::clamp(bottom, 0, transform.image_height);
  return cv::Rect(
    clipped_left, clipped_top,
    std::max(0, clipped_right - clipped_left),
    std::max(0, clipped_bottom - clipped_top));
}

inline bool IsAllowedClass(const int class_id, const std::set<int> & allowed)
{
  return allowed.empty() || allowed.count(class_id) > 0;
}

inline std::vector<Detection> ParseDetections(
  const cv::Mat & output,
  const LetterboxTransform & transform,
  const int number_of_classes,
  const float confidence_threshold,
  const float nms_threshold,
  const std::set<int> & allowed_classes = {})
{
  if (output.empty() || output.type() != CV_32F) {
    return {};
  }

  cv::Mat rows;
  bool raw_features_first = false;
  if (output.dims == 2) {
    rows = output;
  } else if (output.dims == 3 && output.size[0] == 1) {
    const int first = output.size[1];
    const int second = output.size[2];
    cv::Mat view(first, second, CV_32F, const_cast<float *>(output.ptr<float>()));
    const bool features_first =
      first == number_of_classes + 4 ||
      first == number_of_classes + 5;
    raw_features_first = features_first;
    rows = features_first ? view.t() : view;
  } else {
    return {};
  }

  if (!rows.isContinuous()) {
    rows = rows.clone();
  }
  const int columns = rows.cols;
  // A raw two-class Ultralytics model also has six features
  // [x, y, w, h, class0, class1]. Its standard export is [1, 6, N],
  // whereas an end-to-end/NMS output is [1, N, 6].
  const bool end_to_end = columns == 6 && !raw_features_first;
  const bool has_objectness = columns == number_of_classes + 5;
  const int class_offset = has_objectness ? 5 : 4;
  if (!end_to_end && columns < class_offset + 1) {
    return {};
  }

  std::vector<Detection> candidates;
  for (int row_index = 0; row_index < rows.rows; ++row_index) {
    const float * values = rows.ptr<float>(row_index);
    int class_id = -1;
    float confidence = 0.0F;
    float x_min = 0.0F;
    float y_min = 0.0F;
    float x_max = 0.0F;
    float y_max = 0.0F;

    if (end_to_end) {
      x_min = values[0];
      y_min = values[1];
      x_max = values[2];
      y_max = values[3];
      confidence = values[4];
      class_id = static_cast<int>(std::round(values[5]));
    } else {
      float best_class_score = -1.0F;
      const int available_classes = std::min(
        number_of_classes, columns - class_offset);
      for (int index = 0; index < available_classes; ++index) {
        if (values[class_offset + index] > best_class_score) {
          best_class_score = values[class_offset + index];
          class_id = index;
        }
      }
      const float objectness = has_objectness ? values[4] : 1.0F;
      confidence = objectness * best_class_score;
      const float center_x = values[0];
      const float center_y = values[1];
      const float width = values[2];
      const float height = values[3];
      x_min = center_x - 0.5F * width;
      y_min = center_y - 0.5F * height;
      x_max = center_x + 0.5F * width;
      y_max = center_y + 0.5F * height;
    }

    if (
      class_id < 0 || confidence < confidence_threshold ||
      !IsAllowedClass(class_id, allowed_classes))
    {
      continue;
    }

    const float coordinate_max = std::max({x_max, y_max, x_min, y_min});
    if (coordinate_max <= 2.0F) {
      x_min *= transform.input_width;
      x_max *= transform.input_width;
      y_min *= transform.input_height;
      y_max *= transform.input_height;
    }
    const cv::Rect box = MapBoxToImage(
      x_min, y_min, x_max, y_max, transform);
    if (box.width < 2 || box.height < 2) {
      continue;
    }
    candidates.push_back(Detection{class_id, confidence, box});
  }

  std::map<int, std::vector<int>> indices_by_class;
  for (size_t index = 0; index < candidates.size(); ++index) {
    indices_by_class[candidates[index].class_id].push_back(
      static_cast<int>(index));
  }

  std::vector<Detection> selected;
  for (const auto & [class_id, class_indices] : indices_by_class) {
    (void)class_id;
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    boxes.reserve(class_indices.size());
    confidences.reserve(class_indices.size());
    for (const int candidate_index : class_indices) {
      boxes.push_back(candidates[static_cast<size_t>(candidate_index)].box);
      confidences.push_back(
        candidates[static_cast<size_t>(candidate_index)].confidence);
    }
    std::vector<int> kept;
    cv::dnn::NMSBoxes(
      boxes, confidences, confidence_threshold, nms_threshold, kept);
    for (const int local_index : kept) {
      selected.push_back(candidates[
          static_cast<size_t>(class_indices[static_cast<size_t>(local_index)])]);
    }
  }
  std::sort(
    selected.begin(), selected.end(),
    [](const Detection & left, const Detection & right) {
      return left.confidence > right.confidence;
    });
  return selected;
}

inline int BlobPixelScore(const cv::Mat & mask)
{
  cv::Mat labels;
  cv::Mat statistics;
  cv::Mat centroids;
  const int component_count = cv::connectedComponentsWithStats(
    mask, labels, statistics, centroids, 8, CV_32S);
  const int image_area = std::max(1, mask.rows * mask.cols);
  const int minimum_area = std::max(
    2, static_cast<int>(std::round(0.0005 * image_area)));
  const int maximum_area = std::max(
    minimum_area, static_cast<int>(std::round(0.35 * image_area)));
  int score = 0;
  for (int component = 1; component < component_count; ++component) {
    const int area = statistics.at<int>(component, cv::CC_STAT_AREA);
    if (area >= minimum_area && area <= maximum_area) {
      score += area;
    }
  }
  return score;
}

inline cv::Rect ExpandTrafficLightColourBox(
  const cv::Rect & detection_box,
  const cv::Size & image_size)
{
  if (
    detection_box.width <= 0 || detection_box.height <= 0 ||
    image_size.width <= 0 || image_size.height <= 0)
  {
    return {};
  }

  // Small COCO traffic-light boxes follow the currently dominant bulb and
  // can trim the lower lamp at way 25.  Keep the original top anchor, add a
  // small horizontal margin, and extend down far enough to include the green
  // bulb.  This box is used only for colour classification/debug drawing; the
  // original detection remains responsible for route/ROI candidate selection.
  const int horizontal_padding = std::max(
    1, static_cast<int>(std::round(0.12 * detection_box.width)));
  const int bottom_padding = std::max(
    4, static_cast<int>(std::round(0.50 * detection_box.height)));
  const int left = std::max(0, detection_box.x - horizontal_padding);
  const int top = std::max(0, detection_box.y);
  const int right = std::min(
    image_size.width,
    detection_box.x + detection_box.width + horizontal_padding);
  const int bottom = std::min(
    image_size.height,
    detection_box.y + detection_box.height + bottom_padding);
  return cv::Rect(
    left, top, std::max(0, right - left), std::max(0, bottom - top));
}

inline int LowerGreenLampScore(const cv::Mat & hsv)
{
  if (hsv.empty() || hsv.rows < 9 || hsv.cols < 3) {
    return 0;
  }

  // Unity's detector sometimes trims the bottom of the very small way-52
  // traffic-light box.  In that case the lit green bulb straddles the middle
  // and bottom thirds, so a pure three-band brightness classifier calls it
  // yellow.  A saturated green blob is much more specific than brightness:
  // the warm metal housing is bright but has an orange/red hue.
  cv::Mat green_mask;
  cv::inRange(
    hsv, cv::Scalar(32, 70, 180), cv::Scalar(100, 255, 255), green_mask);

  cv::Mat labels;
  cv::Mat statistics;
  cv::Mat centroids;
  const int component_count = cv::connectedComponentsWithStats(
    green_mask, labels, statistics, centroids, 8, CV_32S);
  const int image_area = std::max(1, hsv.rows * hsv.cols);
  const int minimum_area = std::max(
    3, static_cast<int>(std::round(0.01 * image_area)));
  const int maximum_area = std::max(
    minimum_area, static_cast<int>(std::round(0.35 * image_area)));
  const double minimum_center_y = 0.38 * static_cast<double>(hsv.rows);

  int score = 0;
  for (int component = 1; component < component_count; ++component) {
    const int area = statistics.at<int>(component, cv::CC_STAT_AREA);
    const int left = statistics.at<int>(component, cv::CC_STAT_LEFT);
    const int top = statistics.at<int>(component, cv::CC_STAT_TOP);
    const int width = statistics.at<int>(component, cv::CC_STAT_WIDTH);
    const int height = statistics.at<int>(component, cv::CC_STAT_HEIGHT);
    const double center_y = centroids.at<double>(component, 1);
    const double aspect_ratio =
      static_cast<double>(width) / static_cast<double>(std::max(1, height));
    const bool touches_boundary =
      left <= 0 || left + width >= hsv.cols ||
      top + height >= hsv.rows;
    const bool compact_lamp_shape =
      width <= std::max(2, static_cast<int>(std::round(0.80 * hsv.cols))) &&
      height <= std::max(2, static_cast<int>(std::round(0.45 * hsv.rows))) &&
      aspect_ratio >= 0.45 && aspect_ratio <= 2.20;
    if (
      area >= minimum_area && area <= maximum_area &&
      center_y >= minimum_center_y &&
      !touches_boundary && compact_lamp_shape)
    {
      score += area;
    }
  }
  return score;
}

inline int SupplementalGreenLampScore(
  const cv::Mat & expanded_hsv,
  const int original_box_area)
{
  if (original_box_area <= 0) {
    return 0;
  }
  const int raw_score = LowerGreenLampScore(expanded_hsv);
  const int minimum_lamp_area = std::max(
    4, static_cast<int>(std::round(0.03 * original_box_area)));
  if (raw_score < minimum_lamp_area) {
    return 0;
  }

  // Existing strict-green thresholds were calibrated with pixel counts from
  // complete boxes.  Express a compact lamp found just outside a trimmed box
  // as per-mille occupancy of the original detection, so an equivalent lamp
  // has a comparable score regardless of a few pixels of box jitter.
  return std::max(
    raw_score,
    static_cast<int>(std::round(
      1000.0 * static_cast<double>(raw_score) /
      static_cast<double>(original_box_area))));
}

inline int PredictedRoiGreenLampScore(const cv::Mat & bgr_crop)
{
  if (bgr_crop.empty() || bgr_crop.channels() != 3) {
    return 0;
  }

  cv::Mat hsv;
  cv::cvtColor(bgr_crop, hsv, cv::COLOR_BGR2HSV);
  const int raw_score = LowerGreenLampScore(hsv);
  const int maximum_lamp_area = std::max(
    12, static_cast<int>(std::round(
      0.04 * static_cast<double>(bgr_crop.rows * bgr_crop.cols))));
  if (raw_score <= 0 || raw_score > maximum_lamp_area) {
    return 0;
  }

  // The route-predicted crop is deliberately larger than a YOLO box so it
  // still contains the signal when the tiny green-phase detection disappears.
  // Normalize the compact bulb back to the approximate area of the detector's
  // original (unexpanded) box so downstream strict-green thresholds retain
  // the same meaning.
  const int nominal_detection_area = std::max(
    1, static_cast<int>(std::round(
      static_cast<double>(bgr_crop.rows * bgr_crop.cols) / 6.0)));
  return std::max(
    raw_score,
    static_cast<int>(std::round(
      1000.0 * static_cast<double>(raw_score) /
      static_cast<double>(nominal_detection_area))));
}

inline bool ClassifyVerticalTrafficLightLayout(
  const cv::Mat & hsv,
  TrafficLightScores & result)
{
  if (
    hsv.empty() || hsv.rows < 9 || hsv.cols < 3 ||
    hsv.rows < 1.5 * static_cast<double>(hsv.cols))
  {
    return false;
  }

  const int lower_green_score = LowerGreenLampScore(hsv);
  if (lower_green_score > 0) {
    result.red = 0;
    result.yellow = 0;
    result.green = lower_green_score;
    result.state = kGreen;
    return true;
  }

  const int horizontal_inset = std::min(
    std::max(1, static_cast<int>(std::round(0.18 * hsv.cols))),
    (hsv.cols - 1) / 2);
  const int inner_width = hsv.cols - 2 * horizontal_inset;
  if (inner_width < 1) {
    return false;
  }

  const std::array<int, 4> row_boundaries{
    0, hsv.rows / 3, 2 * hsv.rows / 3, hsv.rows};
  std::array<double, 3> lamp_values{};
  for (size_t lamp = 0; lamp < lamp_values.size(); ++lamp) {
    const int first_row = row_boundaries[lamp];
    const int last_row = row_boundaries[lamp + 1];
    if (last_row <= first_row) {
      return false;
    }
    const cv::Rect region(
      horizontal_inset, first_row, inner_width, last_row - first_row);
    lamp_values[lamp] = cv::mean(hsv(region))[2];
  }

  const auto winner = std::max_element(lamp_values.begin(), lamp_values.end());
  const size_t winner_index =
    static_cast<size_t>(std::distance(lamp_values.begin(), winner));
  std::array<double, 3> sorted_values = lamp_values;
  std::sort(sorted_values.begin(), sorted_values.end(), std::greater<double>());
  const double baseline =
    *std::min_element(lamp_values.begin(), lamp_values.end());
  const double winner_margin = sorted_values[0] - sorted_values[1];
  const double activation = sorted_values[0] - baseline;
  if (winner_margin < 12.0 || activation < 18.0) {
    return false;
  }

  result.red = static_cast<int>(std::round(
      std::max(0.0, lamp_values[0] - baseline)));
  result.yellow = static_cast<int>(std::round(
      std::max(0.0, lamp_values[1] - baseline)));
  result.green = static_cast<int>(std::round(
      std::max(0.0, lamp_values[2] - baseline)));
  result.state = std::array<std::uint8_t, 3>{kRed, kYellow, kGreen}[
    winner_index];
  return true;
}

inline TrafficLightScores ClassifyTrafficLight(const cv::Mat & bgr_crop)
{
  TrafficLightScores result;
  if (bgr_crop.empty() || bgr_crop.channels() != 3) {
    return result;
  }

  cv::Mat hsv;
  cv::cvtColor(bgr_crop, hsv, cv::COLOR_BGR2HSV);
  if (ClassifyVerticalTrafficLightLayout(hsv, result)) {
    return result;
  }

  cv::Mat red_low;
  cv::Mat red_high;
  cv::Mat red_mask;
  cv::Mat yellow_mask;
  cv::Mat green_mask;
  cv::inRange(hsv, cv::Scalar(0, 60, 80), cv::Scalar(15, 255, 255), red_low);
  cv::inRange(hsv, cv::Scalar(160, 60, 80), cv::Scalar(179, 255, 255), red_high);
  cv::bitwise_or(red_low, red_high, red_mask);
  cv::inRange(
    hsv, cv::Scalar(21, 125, 185), cv::Scalar(29, 255, 255), yellow_mask);
  cv::inRange(
    hsv, cv::Scalar(28, 35, 60), cv::Scalar(100, 255, 255), green_mask);
  green_mask.setTo(0, yellow_mask);

  result.red = BlobPixelScore(red_mask);
  result.yellow = BlobPixelScore(yellow_mask);
  result.green = BlobPixelScore(green_mask);

  const int image_area = std::max(1, bgr_crop.rows * bgr_crop.cols);
  const bool red_valid =
    result.red >= std::max(3, static_cast<int>(0.001 * image_area));
  const bool yellow_valid =
    result.yellow >= std::max(4, static_cast<int>(0.002 * image_area));
  const bool green_valid =
    result.green >= std::max(3, static_cast<int>(0.001 * image_area));
  const int best_non_yellow = std::max(result.red, result.green);
  const bool yellow_dominates =
    yellow_valid &&
    static_cast<double>(result.yellow) >=
    1.8 * static_cast<double>(std::max(1, best_non_yellow));
  const bool green_over_red =
    green_valid &&
    static_cast<double>(result.green) >=
    0.8 * static_cast<double>(std::max(1, result.red));
  const bool yellow_without_red =
    !red_valid && yellow_valid &&
    (!green_valid ||
    static_cast<double>(result.yellow) >=
    1.25 * static_cast<double>(std::max(1, result.green)));

  if (red_valid && !green_over_red && !yellow_dominates) {
    result.state = kRed;
  } else if (yellow_without_red) {
    result.state = kYellow;
  } else if (green_valid && !yellow_dominates) {
    result.state = kGreen;
  } else if (yellow_dominates) {
    result.state = kYellow;
  } else if (red_valid) {
    result.state = kRed;
  } else if (green_valid) {
    result.state = kGreen;
  }
  return result;
}

}  // namespace perception::yolo
