"""ROS-independent semantic/point-cloud vehicle distance fusion."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VehicleObstacleFusion:
    """Control-facing distances and evidence used to produce them."""

    front_distance: float
    merge_distance: float
    confirmed: bool
    front_matched: bool
    merge_matched: bool


def distances_match(
    pointcloud_distance: float,
    semantic_distance: float,
    absolute_tolerance: float,
    relative_tolerance: float,
) -> bool:
    """Return whether two finite positive ranges plausibly describe one object."""

    if (
        not math.isfinite(pointcloud_distance)
        or not math.isfinite(semantic_distance)
        or pointcloud_distance <= 0.0
        or semantic_distance <= 0.0
    ):
        return False
    tolerance = max(
        max(0.0, float(absolute_tolerance)),
        max(0.0, float(relative_tolerance)) * semantic_distance,
    )
    return abs(pointcloud_distance - semantic_distance) <= tolerance


def semantic_distance_is_plausible(
    previous_distance: float,
    candidate_distance: float,
    elapsed: float,
    max_abs_range_rate: float,
    jump_tolerance: float,
) -> bool:
    """Reject a one-frame semantic range jump that is physically implausible."""

    if not math.isfinite(candidate_distance) or candidate_distance <= 0.0:
        return False
    if not math.isfinite(previous_distance) or previous_distance <= 0.0:
        return True
    if not math.isfinite(elapsed) or elapsed <= 0.0:
        return False
    maximum_change = (
        max(0.0, float(jump_tolerance))
        + max(0.0, float(max_abs_range_rate)) * elapsed
    )
    return abs(candidate_distance - previous_distance) <= maximum_change


def fuse_vehicle_obstacle_distances(
    pointcloud_front_distance: float,
    pointcloud_merge_distance: float,
    semantic_vehicle_detected: bool,
    semantic_vehicle_distance: float,
    absolute_tolerance: float,
    relative_tolerance: float,
    unconfirmed_emergency_distance: float,
    semantic_direct_control: bool = False,
    pointcloud_match_front_distance: float | None = None,
) -> VehicleObstacleFusion:
    """Fuse semantic classification with path-relative point-cloud evidence.

    Across the route, a semantic vehicle controls the car only when its range
    agrees with the point-cloud front or merge corridor. Selected, validated
    waypoints may enable direct semantic control because Event2 runtime data
    showed that its point cloud measures unrelated road geometry near the NPC.

    The tiny unconfirmed point-cloud front range remains as a last-resort
    generic collision guard when no semantic vehicle is present.
    """

    semantic_valid = (
        semantic_vehicle_detected
        and math.isfinite(semantic_vehicle_distance)
        and semantic_vehicle_distance > 0.0
    )
    match_front_distance = (
        pointcloud_front_distance
        if pointcloud_match_front_distance is None
        else pointcloud_match_front_distance
    )
    front_matched = semantic_valid and distances_match(
        match_front_distance,
        semantic_vehicle_distance,
        absolute_tolerance,
        relative_tolerance,
    )
    merge_matched = semantic_valid and distances_match(
        pointcloud_merge_distance,
        semantic_vehicle_distance,
        absolute_tolerance,
        relative_tolerance,
    )
    direct_control = bool(semantic_direct_control) and semantic_valid
    front_confirmed = semantic_valid and (direct_control or front_matched)
    merge_confirmed = semantic_valid and (direct_control or merge_matched)

    if front_confirmed:
        front_distance = float(semantic_vehicle_distance)
    elif (
        math.isfinite(pointcloud_front_distance)
        and 0.0 < pointcloud_front_distance
        <= max(0.0, float(unconfirmed_emergency_distance))
    ):
        front_distance = float(pointcloud_front_distance)
    else:
        front_distance = math.inf

    merge_distance = (
        float(semantic_vehicle_distance) if merge_confirmed else math.inf
    )
    return VehicleObstacleFusion(
        front_distance=front_distance,
        merge_distance=merge_distance,
        confirmed=front_confirmed or merge_confirmed,
        front_matched=front_matched,
        merge_matched=merge_matched,
    )
