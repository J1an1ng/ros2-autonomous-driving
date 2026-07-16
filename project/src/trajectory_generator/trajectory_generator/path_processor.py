from dataclasses import dataclass
import math

from trajectory_generator.geometry_utils import distance_2d, Pose2D


POINT_MERGE_TOLERANCE = 1e-3
PROJECTION_DISTANCE_TOLERANCE = 0.5
ROUTE_POINT_MERGE_DISTANCE = 1.0


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    z: float
    yaw: float
    s: float = 0.0
    curvature: float = 0.0


@dataclass(frozen=True)
class PathProcessingResult:
    path_points: list
    rejection_reason: str = ""


def interpolate_path(ego_pose, target_pose, path_resolution, min_target_distance):
    poses = (ego_pose, target_pose)
    if any(
        not all(math.isfinite(value) for value in (pose.x, pose.y, pose.z))
        for pose in poses
    ):
        return []

    distance = distance_2d(ego_pose.x, ego_pose.y, target_pose.x, target_pose.y)

    if distance <= POINT_MERGE_TOLERANCE:
        return []
    path_yaw = math.atan2(target_pose.y - ego_pose.y, target_pose.x - ego_pose.x)

    if distance < min_target_distance:
        return _compute_geometry(
            [
                Pose2D(ego_pose.x, ego_pose.y, ego_pose.z, path_yaw),
                Pose2D(
                    target_pose.x,
                    target_pose.y,
                    target_pose.z,
                    target_pose.yaw,
                ),
            ]
        )

    resolution = max(float(path_resolution), 1e-3)
    steps = max(1, int(math.ceil(distance / resolution)))
    points = []

    for index in range(steps + 1):
        ratio = index / steps
        x = ego_pose.x + ratio * (target_pose.x - ego_pose.x)
        y = ego_pose.y + ratio * (target_pose.y - ego_pose.y)
        z = ego_pose.z + ratio * (target_pose.z - ego_pose.z)
        yaw = target_pose.yaw if index == steps else path_yaw
        points.append(Pose2D(x, y, z, yaw))

    return _compute_geometry(points)


def _deduplicate_points(points):
    if not points:
        return []

    result = [points[0]]
    for point in points[1:]:
        previous = result[-1]
        if distance_2d(previous.x, previous.y, point.x, point.y) > (
            POINT_MERGE_TOLERANCE
        ):
            result.append(point)
    return result


def _deduplicate_route_points(points):
    """Collapse sub-metre survey clusters while retaining their latest tangent."""
    if not points:
        return []
    result = [points[0]]
    for point in points[1:]:
        previous = result[-1]
        spacing = distance_2d(previous.x, previous.y, point.x, point.y)
        if spacing < ROUTE_POINT_MERGE_DISTANCE:
            if spacing <= POINT_MERGE_TOLERANCE:
                result[-1] = Pose2D(
                    previous.x,
                    previous.y,
                    previous.z,
                    point.yaw,
                )
            else:
                result[-1] = point
        else:
            result.append(point)
    return result


def _project_ego_onto_polyline(points, ego_pose):
    candidates = []
    for index, (start, end) in enumerate(zip(points, points[1:])):
        dx = end.x - start.x
        dy = end.y - start.y
        length_squared = dx * dx + dy * dy
        if length_squared <= POINT_MERGE_TOLERANCE ** 2:
            continue

        ratio = (
            (ego_pose.x - start.x) * dx
            + (ego_pose.y - start.y) * dy
        ) / length_squared
        ratio = min(max(ratio, 0.0), 1.0)
        x = start.x + ratio * dx
        y = start.y + ratio * dy
        z = start.z + ratio * (end.z - start.z)
        distance = distance_2d(ego_pose.x, ego_pose.y, x, y)
        segment_yaw = math.atan2(dy, dx)
        endpoint_ratio = min(PROJECTION_DISTANCE_TOLERANCE / math.sqrt(length_squared), 0.5)
        if ratio <= endpoint_ratio:
            projected_yaw = start.yaw
        elif ratio >= 1.0 - endpoint_ratio:
            projected_yaw = end.yaw
        else:
            projected_yaw = segment_yaw
        heading_error = abs(
            math.atan2(
                math.sin(segment_yaw - ego_pose.yaw),
                math.cos(segment_yaw - ego_pose.yaw),
            )
        )
        candidates.append(
            (
                distance,
                heading_error,
                index,
                Pose2D(x=x, y=y, z=z, yaw=projected_yaw),
            )
        )

    if not candidates:
        return None
    minimum_distance = min(candidate[0] for candidate in candidates)
    nearby_candidates = [
        candidate
        for candidate in candidates
        if candidate[0] <= minimum_distance + PROJECTION_DISTANCE_TOLERANCE
    ]
    _, _, segment_index, projected_point = min(
        nearby_candidates,
        key=lambda candidate: (candidate[1], candidate[0]),
    )
    return segment_index, projected_point


def _crop_from_projection(points, ego_pose):
    projection = _project_ego_onto_polyline(points, ego_pose)
    if projection is None:
        return []

    segment_index, projected_point = projection
    cropped = [projected_point]
    cropped.extend(points[segment_index + 1:])
    return _deduplicate_points(cropped)


def _smooth_polyline(points, path_resolution, smoothing_strength):
    if len(points) < 3 or smoothing_strength <= 0.0:
        return points

    strength = min(float(smoothing_strength), 0.45)
    sample_step = min(max(float(path_resolution) * 0.25, 0.05), 0.25)
    smoothed = [points[0]]

    for previous, corner, following in zip(points, points[1:], points[2:]):
        incoming_x = corner.x - previous.x
        incoming_y = corner.y - previous.y
        outgoing_x = following.x - corner.x
        outgoing_y = following.y - corner.y
        incoming_length = math.hypot(incoming_x, incoming_y)
        outgoing_length = math.hypot(outgoing_x, outgoing_y)
        direction_dot = (
            incoming_x * outgoing_x + incoming_y * outgoing_y
        ) / (incoming_length * outgoing_length)
        if direction_dot < -0.95:
            smoothed.append(corner)
            continue

        trim_distance = strength * min(incoming_length, outgoing_length)
        if trim_distance <= POINT_MERGE_TOLERANCE:
            smoothed.append(corner)
            continue

        entry_ratio = trim_distance / incoming_length
        exit_ratio = trim_distance / outgoing_length
        entry = Pose2D(
            x=corner.x - entry_ratio * incoming_x,
            y=corner.y - entry_ratio * incoming_y,
            z=corner.z - entry_ratio * (corner.z - previous.z),
        )
        exit_point = Pose2D(
            x=corner.x + exit_ratio * outgoing_x,
            y=corner.y + exit_ratio * outgoing_y,
            z=corner.z + exit_ratio * (following.z - corner.z),
        )
        smoothed.append(entry)

        curve_samples = max(2, int(math.ceil(2.0 * trim_distance / sample_step)))
        for sample_index in range(1, curve_samples + 1):
            ratio = sample_index / curve_samples
            inverse = 1.0 - ratio
            smoothed.append(
                Pose2D(
                    x=(
                        inverse * inverse * entry.x
                        + 2.0 * inverse * ratio * corner.x
                        + ratio * ratio * exit_point.x
                    ),
                    y=(
                        inverse * inverse * entry.y
                        + 2.0 * inverse * ratio * corner.y
                        + ratio * ratio * exit_point.y
                    ),
                    z=(
                        inverse * inverse * entry.z
                        + 2.0 * inverse * ratio * corner.z
                        + ratio * ratio * exit_point.z
                    ),
                )
            )

    smoothed.append(points[-1])
    return _deduplicate_points(smoothed)


def _interpolate_waypoint_headings(
    points,
    path_resolution,
    smoothing_strength,
):
    """Interpolate sparse route points while respecting their tangent yaw."""
    if len(points) < 2 or smoothing_strength <= 0.0:
        return points

    tangent_scale = min(max(float(smoothing_strength) * 4.8, 0.0), 1.5)
    sample_step = min(max(float(path_resolution) * 0.25, 0.05), 0.25)
    result = [points[0]]
    maximum_heading_error = math.radians(70.0)

    for start, end in zip(points, points[1:]):
        dx = end.x - start.x
        dy = end.y - start.y
        length = math.hypot(dx, dy)
        if length <= POINT_MERGE_TOLERANCE:
            continue
        chord_yaw = math.atan2(dy, dx)
        heading_errors = [
            abs(math.atan2(math.sin(yaw - chord_yaw), math.cos(yaw - chord_yaw)))
            for yaw in (start.yaw, end.yaw)
        ]
        if length < 2.0 or max(heading_errors) > maximum_heading_error:
            result.append(end)
            continue

        tangent_length = tangent_scale * length
        start_tangent = (
            tangent_length * math.cos(start.yaw),
            tangent_length * math.sin(start.yaw),
        )
        end_tangent = (
            tangent_length * math.cos(end.yaw),
            tangent_length * math.sin(end.yaw),
        )
        samples = max(2, int(math.ceil(length / sample_step)))
        for sample_index in range(1, samples + 1):
            ratio = sample_index / samples
            ratio_squared = ratio * ratio
            ratio_cubed = ratio_squared * ratio
            h00 = 2.0 * ratio_cubed - 3.0 * ratio_squared + 1.0
            h10 = ratio_cubed - 2.0 * ratio_squared + ratio
            h01 = -2.0 * ratio_cubed + 3.0 * ratio_squared
            h11 = ratio_cubed - ratio_squared
            result.append(
                Pose2D(
                    x=(
                        h00 * start.x
                        + h10 * start_tangent[0]
                        + h01 * end.x
                        + h11 * end_tangent[0]
                    ),
                    y=(
                        h00 * start.y
                        + h10 * start_tangent[1]
                        + h01 * end.y
                        + h11 * end_tangent[1]
                    ),
                    z=start.z + ratio * (end.z - start.z),
                )
            )
    return _deduplicate_points(result)


def _resample_polyline(points, path_resolution):
    if len(points) < 2:
        return []

    segment_lengths = []
    cumulative_lengths = [0.0]
    for start, end in zip(points, points[1:]):
        length = distance_2d(start.x, start.y, end.x, end.y)
        segment_lengths.append(length)
        cumulative_lengths.append(cumulative_lengths[-1] + length)

    resolution = max(float(path_resolution), 1e-3)
    total_length = cumulative_lengths[-1]
    sample_distances = []
    distance = 0.0
    while distance < total_length:
        sample_distances.append(distance)
        distance += resolution
    if total_length - sample_distances[-1] > POINT_MERGE_TOLERANCE:
        sample_distances.append(total_length)
    else:
        sample_distances[-1] = total_length

    sampled_points = []
    segment_index = 0
    for sample_distance in sample_distances:
        while (
            segment_index < len(segment_lengths) - 1
            and sample_distance > cumulative_lengths[segment_index + 1]
        ):
            segment_index += 1

        start = points[segment_index]
        end = points[segment_index + 1]
        segment_start = cumulative_lengths[segment_index]
        ratio = (
            sample_distance - segment_start
        ) / segment_lengths[segment_index]
        sampled_points.append(
            Pose2D(
                x=start.x + ratio * (end.x - start.x),
                y=start.y + ratio * (end.y - start.y),
                z=start.z + ratio * (end.z - start.z),
            )
        )
    return sampled_points


def _compute_geometry(sampled_points):
    if len(sampled_points) < 2:
        return []

    arc_lengths = [0.0]
    for start, end in zip(sampled_points, sampled_points[1:]):
        arc_lengths.append(
            arc_lengths[-1] + distance_2d(start.x, start.y, end.x, end.y)
        )

    yaw_values = []
    for index, point in enumerate(sampled_points):
        if index == 0:
            neighbor_a = point
            neighbor_b = sampled_points[1]
        elif index == len(sampled_points) - 1:
            neighbor_a = sampled_points[index - 1]
            neighbor_b = point
        else:
            neighbor_a = sampled_points[index - 1]
            neighbor_b = sampled_points[index + 1]
        yaw = math.atan2(
            neighbor_b.y - neighbor_a.y,
            neighbor_b.x - neighbor_a.x,
        )
        yaw_values.append(yaw)

    curvature_values = [0.0] * len(sampled_points)
    for index in range(1, len(sampled_points) - 1):
        previous = sampled_points[index - 1]
        current = sampled_points[index]
        following = sampled_points[index + 1]
        side_a = distance_2d(previous.x, previous.y, current.x, current.y)
        side_b = distance_2d(current.x, current.y, following.x, following.y)
        side_c = distance_2d(previous.x, previous.y, following.x, following.y)
        denominator = side_a * side_b * side_c
        if denominator <= POINT_MERGE_TOLERANCE ** 3:
            continue
        cross_product = (
            (current.x - previous.x) * (following.y - previous.y)
            - (current.y - previous.y) * (following.x - previous.x)
        )
        curvature_values[index] = 2.0 * cross_product / denominator

    if len(curvature_values) > 2:
        curvature_values[0] = curvature_values[1]
        curvature_values[-1] = curvature_values[-2]

    result = []
    for index, point in enumerate(sampled_points):
        result.append(
            PathPoint(
                point.x,
                point.y,
                point.z,
                yaw_values[index],
                s=arc_lengths[index],
                curvature=curvature_values[index],
            )
        )
    return result


def _distance_to_polyline(point, polyline):
    minimum_distance = math.inf
    for start, end in zip(polyline, polyline[1:]):
        dx = end.x - start.x
        dy = end.y - start.y
        length_squared = dx * dx + dy * dy
        if length_squared <= POINT_MERGE_TOLERANCE ** 2:
            continue
        ratio = (
            (point.x - start.x) * dx + (point.y - start.y) * dy
        ) / length_squared
        ratio = min(max(ratio, 0.0), 1.0)
        projected_x = start.x + ratio * dx
        projected_y = start.y + ratio * dy
        minimum_distance = min(
            minimum_distance,
            distance_2d(point.x, point.y, projected_x, projected_y),
        )
    return minimum_distance


def get_path_rejection_reason(
    path_points,
    path_resolution,
    max_curvature,
    max_yaw_step,
    max_start_distance,
    max_spline_deviation,
    ego_pose=None,
    source_points=None,
):
    """Return an empty string for a valid path, otherwise a rejection reason."""
    if len(path_points) < 2:
        return "path has fewer than two points"

    for point in path_points:
        values = (
            point.x,
            point.y,
            point.z,
            point.yaw,
            point.s,
            point.curvature,
        )
        if not all(math.isfinite(value) for value in values):
            return "path contains a non-finite value"
        if abs(point.curvature) > max_curvature:
            return "path exceeds the curvature limit"

    resolution = max(float(path_resolution), 1e-3)
    for start, end in zip(path_points, path_points[1:]):
        delta_s = end.s - start.s
        if delta_s <= POINT_MERGE_TOLERANCE:
            return "path arc length is not strictly increasing"
        if delta_s > resolution * 1.5 + POINT_MERGE_TOLERANCE:
            return "path sample spacing exceeds the limit"
        yaw_step = abs(
            math.atan2(
                math.sin(end.yaw - start.yaw),
                math.cos(end.yaw - start.yaw),
            )
        )
        if yaw_step > max_yaw_step:
            return "path exceeds the yaw-step limit"

    if abs(path_points[0].s) > POINT_MERGE_TOLERANCE:
        return "path arc length does not start at zero"
    if ego_pose is not None:
        start_distance = distance_2d(
            ego_pose.x,
            ego_pose.y,
            path_points[0].x,
            path_points[0].y,
        )
        if start_distance > max_start_distance:
            return "path starts too far from the ego pose"
    if source_points is not None:
        for point in path_points:
            if _distance_to_polyline(point, source_points) > max_spline_deviation:
                return "path deviates too far from its source polyline"
    return ""


def validate_path(
    path_points,
    path_resolution,
    max_curvature,
    max_yaw_step,
    max_start_distance,
    max_spline_deviation,
    ego_pose=None,
    source_points=None,
):
    """Return whether a path satisfies all configured geometric limits."""
    return not get_path_rejection_reason(
        path_points,
        path_resolution,
        max_curvature,
        max_yaw_step,
        max_start_distance,
        max_spline_deviation,
        ego_pose=ego_pose,
        source_points=source_points,
    )


def _interpolate_path_point(path_points, query_s):
    segment_index = 0
    while (
        segment_index < len(path_points) - 2
        and query_s > path_points[segment_index + 1].s
    ):
        segment_index += 1

    start = path_points[segment_index]
    end = path_points[segment_index + 1]
    segment_length = end.s - start.s
    if segment_length <= POINT_MERGE_TOLERANCE:
        return Pose2D(start.x, start.y, start.z, start.yaw)
    ratio = min(max((query_s - start.s) / segment_length, 0.0), 1.0)
    return Pose2D(
        x=start.x + ratio * (end.x - start.x),
        y=start.y + ratio * (end.y - start.y),
        z=start.z + ratio * (end.z - start.z),
    )


def blend_with_previous_path(
    previous_path,
    candidate_path,
    ego_pose,
    path_resolution,
    blend_distance,
    candidate_weight,
    max_curvature,
    max_yaw_step,
    max_start_distance,
    max_spline_deviation,
    max_continuity_deviation,
):
    """Blend compatible consecutive paths and preserve a valid new candidate."""
    if len(previous_path) < 2 or len(candidate_path) < 2 or blend_distance <= 0.0:
        return candidate_path

    previous_points = [
        Pose2D(point.x, point.y, point.z, point.yaw)
        for point in previous_path
    ]
    previous_points = _crop_from_projection(previous_points, ego_pose)
    previous_points = _resample_polyline(previous_points, path_resolution)
    previous_path_from_ego = _compute_geometry(previous_points)
    if len(previous_path_from_ego) < 2:
        return candidate_path
    comparison_end = min(
        blend_distance,
        previous_path_from_ego[-1].s,
        candidate_path[-1].s,
    )
    for candidate in candidate_path:
        if candidate.s > comparison_end:
            break
        previous = _interpolate_path_point(previous_path_from_ego, candidate.s)
        deviation = distance_2d(
            previous.x,
            previous.y,
            candidate.x,
            candidate.y,
        )
        if deviation > max_continuity_deviation:
            return candidate_path

    base_weight = min(max(float(candidate_weight), 0.0), 1.0)
    previous_length = previous_path_from_ego[-1].s
    blended_points = []
    for candidate in candidate_path:
        if candidate.s >= blend_distance or candidate.s > previous_length:
            blended_points.append(
                Pose2D(candidate.x, candidate.y, candidate.z, candidate.yaw)
            )
            continue

        previous = _interpolate_path_point(previous_path_from_ego, candidate.s)
        progress = candidate.s / blend_distance
        weight = base_weight + (1.0 - base_weight) * progress
        blended_points.append(
            Pose2D(
                x=(1.0 - weight) * previous.x + weight * candidate.x,
                y=(1.0 - weight) * previous.y + weight * candidate.y,
                z=(1.0 - weight) * previous.z + weight * candidate.z,
            )
        )

    blended_path = _compute_geometry(_deduplicate_points(blended_points))
    if not validate_path(
        blended_path,
        path_resolution,
        max_curvature,
        max_yaw_step,
        max_start_distance,
        max_spline_deviation,
        ego_pose=ego_pose,
        source_points=candidate_path,
    ):
        return candidate_path
    return blended_path


def process_local_path(
    route_points,
    ego_pose,
    path_resolution,
    smoothing_strength=0.25,
    max_curvature=2.0,
    max_yaw_step=1.1,
    max_start_distance=5.0,
    max_spline_deviation=2.0,
    use_waypoint_yaw=False,
):
    """Create a validated reference path from route points and the ego pose."""
    return process_local_path_result(
        route_points,
        ego_pose,
        path_resolution,
        smoothing_strength,
        max_curvature,
        max_yaw_step,
        max_start_distance,
        max_spline_deviation,
        use_waypoint_yaw,
    ).path_points


def process_local_path_result(
    route_points,
    ego_pose,
    path_resolution,
    smoothing_strength=0.25,
    max_curvature=2.0,
    max_yaw_step=1.1,
    max_start_distance=5.0,
    max_spline_deviation=2.0,
    use_waypoint_yaw=False,
):
    """Create a path and retain a diagnostic reason when it is rejected."""
    if len(route_points) < 2:
        return PathProcessingResult([], "local path has fewer than two points")
    if any(
        not all(math.isfinite(value) for value in (point.x, point.y, point.z))
        for point in route_points
    ):
        return PathProcessingResult([], "local path contains a non-finite value")

    usable_points = _deduplicate_route_points(route_points)
    cropped_points = _crop_from_projection(usable_points, ego_pose)
    interpolation_points = cropped_points
    if use_waypoint_yaw:
        interpolation_points = _interpolate_waypoint_headings(
            cropped_points,
            path_resolution,
            smoothing_strength,
        )
    smoothed_points = _smooth_polyline(
        interpolation_points,
        path_resolution,
        smoothing_strength,
    )
    sampled_points = _resample_polyline(
        smoothed_points,
        path_resolution,
    )
    path_points = _compute_geometry(sampled_points)
    rejection_reason = get_path_rejection_reason(
        path_points,
        path_resolution,
        max_curvature,
        max_yaw_step,
        max_start_distance,
        max_spline_deviation,
        ego_pose=ego_pose,
        source_points=cropped_points,
    )
    if rejection_reason:
        return PathProcessingResult([], rejection_reason)
    return PathProcessingResult(path_points)
