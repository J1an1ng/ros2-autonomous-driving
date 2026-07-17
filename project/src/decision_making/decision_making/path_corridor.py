"""ROS-independent geometry for path-relative obstacle filtering."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class PathProjection:
    """Closest projection of a point onto a path polyline."""

    along: float
    lateral: float
    distance: float


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

