import math
from dataclasses import dataclass

from trajectory_generator.geometry_utils import Pose2D, distance_2d


POINT_MERGE_TOLERANCE = 1e-3


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    z: float
    yaw: float


def interpolate_path(ego_pose, target_pose, path_resolution, min_target_distance):
    distance = distance_2d(ego_pose.x, ego_pose.y, target_pose.x, target_pose.y)

    if distance > 1e-6:
        path_yaw = math.atan2(target_pose.y - ego_pose.y, target_pose.x - ego_pose.x)
    else:
        path_yaw = target_pose.yaw

    if distance < min_target_distance:
        return [
            PathPoint(ego_pose.x, ego_pose.y, ego_pose.z, path_yaw),
            PathPoint(target_pose.x, target_pose.y, target_pose.z, target_pose.yaw),
        ]

    resolution = max(float(path_resolution), 1e-3)
    steps = max(1, int(math.ceil(distance / resolution)))
    points = []

    for index in range(steps + 1):
        ratio = index / steps
        x = ego_pose.x + ratio * (target_pose.x - ego_pose.x)
        y = ego_pose.y + ratio * (target_pose.y - ego_pose.y)
        z = ego_pose.z + ratio * (target_pose.z - ego_pose.z)
        yaw = target_pose.yaw if index == steps else path_yaw
        points.append(PathPoint(x, y, z, yaw))

    return points


def process_local_path(route_points, ego_pose, path_resolution):
    if len(route_points) < 2:
        return []
    if any(
        not all(math.isfinite(value) for value in (point.x, point.y, point.z))
        for point in route_points
    ):
        return []

    segment_lengths = []
    cumulative_lengths = [0.0]
    usable_points = [route_points[0]]
    for point in route_points[1:]:
        previous = usable_points[-1]
        length = distance_2d(previous.x, previous.y, point.x, point.y)
        if length <= POINT_MERGE_TOLERANCE:
            continue
        usable_points.append(point)
        segment_lengths.append(length)
        cumulative_lengths.append(cumulative_lengths[-1] + length)

    if len(usable_points) < 2:
        return []

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

        start = usable_points[segment_index]
        end = usable_points[segment_index + 1]
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

    result = []
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
        result.append(PathPoint(point.x, point.y, point.z, yaw))

    heading_x = math.cos(ego_pose.yaw)
    heading_y = math.sin(ego_pose.yaw)

    def is_forward(point):
        longitudinal_distance = (
            (point.x - ego_pose.x) * heading_x
            + (point.y - ego_pose.y) * heading_y
        )
        return longitudinal_distance >= -POINT_MERGE_TOLERANCE

    forward_indices = [
        index
        for index, point in enumerate(result)
        if is_forward(point)
    ]
    if not forward_indices:
        return []

    start_index = min(
        forward_indices,
        key=lambda index: distance_2d(
            ego_pose.x,
            ego_pose.y,
            result[index].x,
            result[index].y,
        ),
    )
    forward_path = result[start_index:]
    for index, point in enumerate(forward_path):
        if not is_forward(point):
            return forward_path[:index]
    return forward_path
