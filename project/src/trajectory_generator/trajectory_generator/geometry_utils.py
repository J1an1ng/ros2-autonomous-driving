from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


def distance_2d(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
