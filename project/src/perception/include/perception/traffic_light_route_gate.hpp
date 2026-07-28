#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <utility>

namespace perception
{

// Associates a traffic light with its controlled waypoint and keeps that
// association alive for a small number of subsequent route transitions.
// Route managers commonly advance to the next waypoint before a vehicle has
// fully stopped at the line, while the same signal still needs to be observed.
class TrafficLightRouteGate
{
public:
  TrafficLightRouteGate(
    std::set<int> controlled_waypoint_ids,
    const int post_waypoint_hold_count)
  : controlled_waypoint_ids_(std::move(controlled_waypoint_ids)),
    post_waypoint_hold_count_(std::max(0, post_waypoint_hold_count))
  {
  }

  void updateWaypoint(const int waypoint_id)
  {
    if (current_waypoint_id_ == waypoint_id) {
      return;
    }
    current_waypoint_id_ = waypoint_id;

    if (controlled_waypoint_ids_.count(waypoint_id) > 0) {
      active_waypoint_id_ = waypoint_id;
      remaining_post_waypoints_ = post_waypoint_hold_count_;
      return;
    }

    if (!active_waypoint_id_) {
      // Recover the association when this node starts or restarts after the
      // route manager has already advanced past the controlled waypoint.
      for (const int controlled_waypoint_id : controlled_waypoint_ids_) {
        const int offset = waypoint_id - controlled_waypoint_id;
        if (offset > 0 && offset <= post_waypoint_hold_count_) {
          active_waypoint_id_ = controlled_waypoint_id;
          remaining_post_waypoints_ = post_waypoint_hold_count_ - offset;
          return;
        }
      }
      return;
    }
    if (remaining_post_waypoints_ > 0) {
      --remaining_post_waypoints_;
      return;
    }
    active_waypoint_id_.reset();
  }

  bool active(
    const double current_waypoint_distance_m,
    const double activation_distance_m) const
  {
    if (!active_waypoint_id_) {
      return false;
    }
    if (current_waypoint_id_ != *active_waypoint_id_) {
      return true;
    }
    return
      std::isfinite(current_waypoint_distance_m) &&
      current_waypoint_distance_m >= 0.0 &&
      current_waypoint_distance_m <= activation_distance_m;
  }

  int activeWaypointId() const
  {
    return active_waypoint_id_.value_or(-1);
  }

  bool isPostWaypointHold() const
  {
    return
      active_waypoint_id_.has_value() &&
      current_waypoint_id_ != *active_waypoint_id_;
  }

private:
  std::set<int> controlled_waypoint_ids_;
  int post_waypoint_hold_count_{1};
  int current_waypoint_id_{-1};
  std::optional<int> active_waypoint_id_;
  int remaining_post_waypoints_{0};
};

}  // namespace perception
