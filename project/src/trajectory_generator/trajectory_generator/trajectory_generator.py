from dataclasses import dataclass

from trajectory_generator.path_processor import (
    interpolate_path,
    process_local_path,
)
from trajectory_generator.velocity_profile import select_target_speed


@dataclass(frozen=True)
class TrajectoryGeneratorConfig:
    path_resolution: float
    min_target_distance: float
    default_speed: float
    max_speed: float
    slow_speed: float
    stop_speed: float


@dataclass(frozen=True)
class GeneratedTrajectory:
    path_points: list
    target_speed: float


class TrajectoryGenerator:
    def __init__(self, config):
        self.config = config

    def generate(
        self,
        ego_pose,
        target_pose,
        speed_limit,
        route_state,
        local_path=None,
    ):
        path_points = process_local_path(
            route_points=local_path or [],
            ego_pose=ego_pose,
            path_resolution=self.config.path_resolution,
        )
        if not path_points and target_pose is not None:
            path_points = interpolate_path(
                ego_pose=ego_pose,
                target_pose=target_pose,
                path_resolution=self.config.path_resolution,
                min_target_distance=self.config.min_target_distance,
            )
        target_speed = select_target_speed(
            route_state=route_state,
            speed_limit=speed_limit,
            default_speed=self.config.default_speed,
            max_speed=self.config.max_speed,
            slow_speed=self.config.slow_speed,
            stop_speed=self.config.stop_speed,
        )
        return GeneratedTrajectory(path_points=path_points, target_speed=target_speed)
