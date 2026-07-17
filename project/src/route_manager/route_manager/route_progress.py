import math


def distance_2d(x1, y1, x2, y2):
    """Return planar Euclidean distance between two points."""
    return math.hypot(x1 - x2, y1 - y2)


def waypoint_reached_or_passed(
    waypoints,
    index,
    ego_x,
    ego_y,
    reach_distance,
):
    """Return how forward route progress satisfies the current waypoint."""
    waypoint = waypoints[index]
    distance = distance_2d(
        ego_x,
        ego_y,
        float(waypoint["x"]),
        float(waypoint["y"]),
    )
    if distance <= reach_distance:
        return "reached"
    if index == 0:
        return ""

    previous = waypoints[index - 1]
    segment_x = float(waypoint["x"]) - float(previous["x"])
    segment_y = float(waypoint["y"]) - float(previous["y"])
    if segment_x * segment_x + segment_y * segment_y <= 1e-6:
        return ""

    beyond_x = ego_x - float(waypoint["x"])
    beyond_y = ego_y - float(waypoint["y"])
    if beyond_x * segment_x + beyond_y * segment_y >= 0.0:
        return "passed"
    return ""
