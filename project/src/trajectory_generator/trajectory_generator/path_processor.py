import math
from dataclasses import dataclass

from trajectory_generator.geometry_utils import Pose2D, distance_2d


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
