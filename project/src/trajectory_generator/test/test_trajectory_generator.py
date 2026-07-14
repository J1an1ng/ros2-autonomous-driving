import pytest

from trajectory_generator.geometry_utils import Pose2D
from trajectory_generator.trajectory_generator import (
    TrajectoryGenerator,
    TrajectoryGeneratorConfig,
)


def make_generator():
    return TrajectoryGenerator(
        TrajectoryGeneratorConfig(
            path_resolution=1.0,
            min_target_distance=0.5,
            default_speed=2.0,
            max_speed=8.0,
            slow_speed=2.0,
            stop_speed=0.0,
        )
    )


def test_prefers_valid_local_path_over_target_pose():
    generator = make_generator()

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=10.0, y=0.0),
        local_path=[Pose2D(x=0.0, y=0.0), Pose2D(x=2.0, y=0.0)],
        speed_limit=5.0,
        route_state="normal",
    )

    assert [point.x for point in trajectory.path_points] == pytest.approx(
        [0.0, 1.0, 2.0]
    )
    assert trajectory.target_speed == pytest.approx(5.0)


def test_falls_back_to_target_pose_when_local_path_is_invalid():
    generator = make_generator()
    repeated_point = Pose2D(x=0.0, y=0.0)

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=2.0, y=0.0),
        local_path=[repeated_point, repeated_point],
        speed_limit=5.0,
        route_state="normal",
    )

    assert [point.x for point in trajectory.path_points] == pytest.approx(
        [0.0, 1.0, 2.0]
    )


def test_returns_empty_path_and_nominal_speed_without_any_path_source():
    generator = make_generator()

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=None,
        local_path=[],
        speed_limit=4.0,
        route_state="normal",
    )

    assert trajectory.path_points == []
    assert trajectory.target_speed == pytest.approx(4.0)


def test_falls_back_when_local_path_contains_non_finite_coordinates():
    generator = make_generator()

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=2.0, y=0.0),
        local_path=[Pose2D(x=0.0, y=0.0), Pose2D(x=float("nan"), y=0.0)],
        speed_limit=5.0,
        route_state="normal",
    )

    assert [point.x for point in trajectory.path_points] == pytest.approx(
        [0.0, 1.0, 2.0]
    )
