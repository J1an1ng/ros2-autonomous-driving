"""ROS-independent geometry for path-relative obstacle filtering."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple


@dataclass(frozen=True)
class PathProjection:
    """Closest projection of a point onto a path polyline."""

    along: float
    lateral: float
    distance: float


def in_vehicle_safety_corridor(
    x: float,
    y: float,
    min_x: float,
    max_x: float,
    half_width: float,
) -> bool:
    """Return whether a point is inside the short ego-frame collision corridor."""
    values = (x, y, min_x, max_x, half_width)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    return min_x <= x <= max_x and abs(y) <= max(0.0, half_width)


def robust_nearest_distance(
    distances: Iterable[float],
    minimum_points: int,
) -> float:
    """Return the nearest distance supported by the requested number of points.

    Using the Nth-nearest point prevents a single depth outlier from commanding
    an emergency stop while retaining a conservative distance for solid objects.
    """
    required = max(1, int(minimum_points))
    finite_distances = sorted(
        float(distance)
        for distance in distances
        if math.isfinite(float(distance)) and float(distance) >= 0.0
    )
    if len(finite_distances) < required:
        return math.inf
    return finite_distances[required - 1]


def matches_fixed_structure_signature(
    waypoint_id: Optional[int],
    suppression_waypoint_ids: Iterable[int],
    path_front_distance: float,
    safety_front_distance: float,
    merge_distance: float,
    max_path_distance: float,
    max_safety_distance: float,
    max_merge_distance: float,
) -> bool:
    """Identify a mapped structure simultaneously spanning all three corridors."""
    if waypoint_id is None or int(waypoint_id) not in {
        int(value) for value in suppression_waypoint_ids
    }:
        return False
    distances_and_limits = (
        (path_front_distance, max_path_distance),
        (safety_front_distance, max_safety_distance),
        (merge_distance, max_merge_distance),
    )
    return all(
        math.isfinite(float(distance))
        and math.isfinite(float(limit))
        and 0.0 <= float(distance) <= max(0.0, float(limit))
        for distance, limit in distances_and_limits
    )


def prepare_path(
    points: Sequence[Tuple[float, float]],
    merge_distance: float = 0.05,
) -> list[Tuple[float, float]]:
    """Prepend the ego origin and remove non-finite/duplicate path points."""
    result = [(0.0, 0.0)]
    for x, y in points:
        x = float(x)
        y = float(y)
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if math.hypot(x - result[-1][0], y - result[-1][1]) < merge_distance:
            continue
        result.append((x, y))
    return result


def project_to_path(
    x: float,
    y: float,
    path: Sequence[Tuple[float, float]],
) -> Optional[PathProjection]:
    """Project a vehicle-frame point onto a polyline.

    ``along`` is arc length from the ego origin. Positive ``lateral`` is left
    of the local path tangent and negative is right.
    """
    if len(path) < 2 or not math.isfinite(x) or not math.isfinite(y):
        return None

    best = None
    cumulative = 0.0
    for start, end in zip(path, path[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-9:
            continue
        length = math.sqrt(length_squared)
        ratio = ((x - start[0]) * dx + (y - start[1]) * dy) / length_squared
        ratio = min(1.0, max(0.0, ratio))
        projected_x = start[0] + ratio * dx
        projected_y = start[1] + ratio * dy
        offset_x = x - projected_x
        offset_y = y - projected_y
        distance = math.hypot(offset_x, offset_y)
        lateral = (dx * offset_y - dy * offset_x) / length
        candidate = PathProjection(
            along=cumulative + ratio * length,
            lateral=lateral,
            distance=distance,
        )
        if best is None or (candidate.distance, candidate.along) < (
            best.distance,
            best.along,
        ):
            best = candidate
        cumulative += length
    return best
