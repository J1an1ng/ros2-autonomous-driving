// Copyright 2026 Team 20

#pragma once

#include <algorithm>
#include <cstdint>

namespace perception
{

class TrafficLightStabilityFilter
{
public:
  static constexpr std::uint8_t kUnknown = 0;
  static constexpr std::uint8_t kGreen = 3;

  explicit TrafficLightStabilityFilter(
    int consecutive_green_frames,
    int confident_green_frames = 2,
    int confident_green_max_red_score = 4,
    int confident_green_min_score = 20)
  : consecutive_green_frames_(std::max(1, consecutive_green_frames)),
    confident_green_frames_(std::max(1, confident_green_frames)),
    confident_green_max_red_score_(std::max(0, confident_green_max_red_score)),
    confident_green_min_score_(std::max(0, confident_green_min_score))
  {
  }

  std::uint8_t update(
    const std::uint8_t raw_state,
    const int red_score,
    const int green_score)
  {
    if (raw_state != kGreen) {
      candidate_state_ = raw_state;
      candidate_count_ = 1;
      confident_candidate_count_ = 0;
      stable_state_ = raw_state;
      return stable_state_;
    }

    if (raw_state == candidate_state_) {
      ++candidate_count_;
    } else {
      candidate_state_ = raw_state;
      candidate_count_ = 1;
    }

    const bool confident_green =
      red_score <= confident_green_max_red_score_ &&
      green_score >= confident_green_min_score_;
    confident_candidate_count_ =
      confident_green ? confident_candidate_count_ + 1 : 0;

    if (
      candidate_count_ >= consecutive_green_frames_ ||
      confident_candidate_count_ >= confident_green_frames_)
    {
      stable_state_ = candidate_state_;
    }
    return stable_state_;
  }

private:
  int consecutive_green_frames_{4};
  int confident_green_frames_{2};
  int confident_green_max_red_score_{4};
  int confident_green_min_score_{20};
  std::uint8_t candidate_state_{kUnknown};
  int candidate_count_{0};
  int confident_candidate_count_{0};
  std::uint8_t stable_state_{kUnknown};
};

}  // namespace perception
