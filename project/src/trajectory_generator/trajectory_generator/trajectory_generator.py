from dataclasses import dataclass
import math

from trajectory_generator.path_processor import (
    blend_with_previous_path,
    get_path_rejection_reason,
    interpolate_path,
    process_local_path_result,
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
    smoothing_strength: float = 0.25
    max_curvature: float = 2.0
    max_yaw_step: float = 1.1
    max_start_distance: float = 5.0
    max_spline_deviation: float = 2.0
    continuity_blend_distance: float = 5.0
    continuity_candidate_weight: float = 0.5
    max_continuity_deviation: float = 2.0

    def __post_init__(self):
        values = {
            "path_resolution": self.path_resolution,
            "min_target_distance": self.min_target_distance,
            "default_speed": self.default_speed,
            "max_speed": self.max_speed,
            "slow_speed": self.slow_speed,
            "stop_speed": self.stop_speed,
            "smoothing_strength": self.smoothing_strength,
            "max_curvature": self.max_curvature,
            "max_yaw_step": self.max_yaw_step,
            "max_start_distance": self.max_start_distance,
            "max_spline_deviation": self.max_spline_deviation,
            "continuity_blend_distance": self.continuity_blend_distance,
            "continuity_candidate_weight": self.continuity_candidate_weight,
            "max_continuity_deviation": self.max_continuity_deviation,
        }
        invalid_value = next(
            (name for name, value in values.items() if not math.isfinite(value)),
            None,
        )
        if invalid_value is not None:
            raise ValueError(f"{invalid_value} must be finite")
        if self.path_resolution <= 0.0:
            raise ValueError("path_resolution must be greater than zero")
        nonnegative_values = {
            name: value for name, value in values.items() if name != "path_resolution"
        }
        invalid_value = next(
            (name for name, value in nonnegative_values.items() if value < 0.0),
            None,
        )
        if invalid_value is not None:
            raise ValueError(f"{invalid_value} must not be negative")


@dataclass(frozen=True)
class GeneratedTrajectory:
    path_points: list
    target_speed: float
    path_status: str = ""


class TrajectoryGenerator:
    def __init__(self, config):
        self.config = config
        self.previous_path_points = []
        self.previous_path_source = None

    def generate(
        self,
        ego_pose,
        target_pose,
        speed_limit,
        route_state,
        local_path=None,
    ):
        local_path_supplied = bool(local_path)
        path_source = "local"
        processing_result = process_local_path_result(
            route_points=local_path or [],
            ego_pose=ego_pose,
            path_resolution=self.config.path_resolution,
            smoothing_strength=self.config.smoothing_strength,
            max_curvature=self.config.max_curvature,
            max_yaw_step=self.config.max_yaw_step,
            max_start_distance=self.config.max_start_distance,
            max_spline_deviation=self.config.max_spline_deviation,
        )
        path_points = processing_result.path_points
        path_status = ""
        if not path_points and target_pose is not None:
            path_source = "fallback"
            path_points = interpolate_path(
                ego_pose=ego_pose,
                target_pose=target_pose,
                path_resolution=self.config.path_resolution,
                min_target_distance=self.config.min_target_distance,
            )
            if path_points and local_path_supplied:
                path_status = (
                    f"local path rejected ({processing_result.rejection_reason}); "
                    "using target-pose fallback"
                )
        rejection_reason = get_path_rejection_reason(
            path_points,
            self.config.path_resolution,
            self.config.max_curvature,
            self.config.max_yaw_step,
            self.config.max_start_distance,
            self.config.max_spline_deviation,
            ego_pose=ego_pose,
        )
        if rejection_reason:
            path_points = []
            if target_pose is not None:
                path_status = f"fallback path rejected: {rejection_reason}"
            elif local_path_supplied:
                path_status = (
                    f"local path rejected: {processing_result.rejection_reason}"
                )
            else:
                path_status = "no local path or target pose is available"
        if path_points and path_source == self.previous_path_source:
            path_points = blend_with_previous_path(
                previous_path=self.previous_path_points,
                candidate_path=path_points,
                ego_pose=ego_pose,
                path_resolution=self.config.path_resolution,
                blend_distance=self.config.continuity_blend_distance,
                candidate_weight=self.config.continuity_candidate_weight,
                max_curvature=self.config.max_curvature,
                max_yaw_step=self.config.max_yaw_step,
                max_start_distance=self.config.max_start_distance,
                max_spline_deviation=self.config.max_spline_deviation,
                max_continuity_deviation=self.config.max_continuity_deviation,
            )
        self.previous_path_points = path_points
        self.previous_path_source = path_source if path_points else None
        target_speed = select_target_speed(
            route_state=route_state,
            speed_limit=speed_limit,
            default_speed=self.config.default_speed,
            max_speed=self.config.max_speed,
            slow_speed=self.config.slow_speed,
            stop_speed=self.config.stop_speed,
        )
        return GeneratedTrajectory(
            path_points=path_points,
            target_speed=target_speed,
            path_status=path_status,
        )
