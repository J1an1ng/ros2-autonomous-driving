"""ROS-independent vision/point-cloud vehicle distance fusion."""

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
    vision_distance: float,
    absolute_tolerance: float,
    relative_tolerance: float,
) -> bool:
    """Return whether two finite positive ranges plausibly describe one object."""
    if (
        not math.isfinite(pointcloud_distance)
        or not math.isfinite(vision_distance)
        or pointcloud_distance <= 0.0
        or vision_distance <= 0.0
    ):
        return False
    tolerance = max(
        max(0.0, float(absolute_tolerance)),
        max(0.0, float(relative_tolerance)) * vision_distance,
    )
    return abs(pointcloud_distance - vision_distance) <= tolerance


def vision_distance_is_plausible(
    previous_distance: float,
    candidate_distance: float,
    elapsed: float,
    max_abs_range_rate: float,
    jump_tolerance: float,
) -> bool:
    """Reject a one-frame vision range jump that is physically implausible."""
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


def vision_detection_is_held(
    detected: bool,
    last_positive_age: float,
    hold_time: float,
) -> bool:
    """Bridge a short detector dropout after a confirmed positive frame."""
    if detected:
        return True
    return (
        math.isfinite(last_positive_age)
        and 0.0 <= last_positive_age <= max(0.0, float(hold_time))
    )


def fuse_vehicle_obstacle_distances(
    pointcloud_front_distance: float,
    pointcloud_merge_distance: float,
    vision_vehicle_detected: bool,
    vision_vehicle_distance: float,
    absolute_tolerance: float,
    relative_tolerance: float,
    unconfirmed_emergency_distance: float,
    vision_direct_control: bool = False,
    pointcloud_match_front_distance: float | None = None,
) -> VehicleObstacleFusion:
    """
    Fuse YOLO/RGB-D classification with path-relative point-cloud evidence.

    Across the route, a vision vehicle controls the car only when its range
    agrees with the point-cloud front or merge corridor. Selected, validated
    waypoints may enable direct vision control because runtime data showed that
    their point cloud measures unrelated road geometry near the NPC.

    The tiny unconfirmed point-cloud front range remains as a last-resort
    generic collision guard when no vision vehicle is present.
    """
    vision_valid = (
        vision_vehicle_detected
        and math.isfinite(vision_vehicle_distance)
        and vision_vehicle_distance > 0.0
    )
    match_front_distance = (
        pointcloud_front_distance
        if pointcloud_match_front_distance is None
        else pointcloud_match_front_distance
    )
    front_matched = vision_valid and distances_match(
        match_front_distance,
        vision_vehicle_distance,
        absolute_tolerance,
        relative_tolerance,
    )
    merge_matched = vision_valid and distances_match(
        pointcloud_merge_distance,
        vision_vehicle_distance,
        absolute_tolerance,
        relative_tolerance,
    )
    direct_control = bool(vision_direct_control) and vision_valid
    front_confirmed = vision_valid and (direct_control or front_matched)
    merge_confirmed = vision_valid and (direct_control or merge_matched)

    if front_matched:
        # Classification comes from YOLO/RGB-D, while the path-relative cloud
        # confirms that the same object occupies the driving corridor. Average
        # the two already-tolerance-checked ranges so a one-sided depth error
        # cannot create an abrupt following-distance step.
        front_distance = 0.5 * (
            float(vision_vehicle_distance) + float(match_front_distance)
        )
    elif front_confirmed:
        # Selected event windows retain RGB-D fallback when the sparse path
        # cloud temporarily misses a real lead vehicle.
        front_distance = float(vision_vehicle_distance)
    elif (
        math.isfinite(pointcloud_front_distance)
        and 0.0 < pointcloud_front_distance
        <= max(0.0, float(unconfirmed_emergency_distance))
    ):
        front_distance = float(pointcloud_front_distance)
    else:
        front_distance = math.inf

    merge_distance = (
        float(vision_vehicle_distance) if merge_confirmed else math.inf
    )
    return VehicleObstacleFusion(
        front_distance=front_distance,
        merge_distance=merge_distance,
        confirmed=front_confirmed or merge_confirmed,
        front_matched=front_matched,
        merge_matched=merge_matched,
    )
