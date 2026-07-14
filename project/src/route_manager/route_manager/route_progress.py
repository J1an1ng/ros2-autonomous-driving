"""Track route initialization and waypoint advancement."""

import math


def select_local_waypoints(waypoints, current_index, local_path_size):
    """Return a bounded local-path window including the previous waypoint."""
    start_index = max(int(current_index) - 1, 0)
    end_index = min(
        start_index + max(int(local_path_size), 0),
        len(waypoints),
    )
    return waypoints[start_index:end_index]


class RouteProgress:
    """Select and advance the current route waypoint."""

    def __init__(self, waypoints, reach_distance):
        """Create uninitialized progress for a non-empty waypoint route."""
        if not waypoints:
            raise ValueError('RouteProgress requires at least one waypoint.')

        self.waypoints = waypoints
        self.reach_distance = float(reach_distance)
        self.current_index = None

    def update(self, ego_x, ego_y, ego_yaw):
        """Initialize once, then advance at most one reached waypoint."""
        if self.current_index is None:
            self.current_index = self._select_initial_index(
                ego_x,
                ego_y,
                ego_yaw,
            )
            return

        if self.current_index >= len(self.waypoints) - 1:
            self.current_index = len(self.waypoints) - 1
            return

        current_waypoint = self.waypoints[self.current_index]
        distance = math.hypot(
            float(current_waypoint['x']) - ego_x,
            float(current_waypoint['y']) - ego_y,
        )
        if distance < self.reach_distance:
            self.current_index += 1

    def _select_initial_index(self, ego_x, ego_y, ego_yaw):
        candidates = []
        for index, waypoint in enumerate(self.waypoints):
            distance = math.hypot(
                float(waypoint['x']) - ego_x,
                float(waypoint['y']) - ego_y,
            )
            waypoint_yaw = math.radians(float(waypoint.get('yaw_deg', 0.0)))
            yaw_difference = abs(
                math.atan2(
                    math.sin(waypoint_yaw - ego_yaw),
                    math.cos(waypoint_yaw - ego_yaw),
                )
            )
            candidates.append((distance, yaw_difference, index))

        nearest_distance = min(candidate[0] for candidate in candidates)
        ambiguity_distance = max(0.0, self.reach_distance)
        nearby_candidates = [
            candidate
            for candidate in candidates
            if candidate[0] <= nearest_distance + ambiguity_distance
        ]
        return min(
            nearby_candidates,
            key=lambda candidate: (candidate[1], candidate[0], candidate[2]),
        )[2]
