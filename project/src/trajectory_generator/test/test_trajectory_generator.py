import math

import pytest

from trajectory_generator.geometry_utils import Pose2D
from trajectory_generator.trajectory_generator import (
    TrajectoryGenerator,
    TrajectoryGeneratorConfig,
)
from trajectory_generator.velocity_profile import select_target_speed


def make_generator():
    return TrajectoryGenerator(
        TrajectoryGeneratorConfig(
            path_resolution=1.0,
            min_target_distance=0.5,
            default_speed=2.0,
            max_speed=8.0,
            stop_speed=0.0,
        )
    )


def test_curvature_limit_is_bounded_by_controller_steering_kinematics():
    config = TrajectoryGeneratorConfig(
        path_resolution=1.0,
        min_target_distance=0.5,
        default_speed=2.0,
        max_speed=8.0,
        stop_speed=0.0,
        max_curvature=2.0,
        wheelbase=2.63,
        maximum_steering_angle=0.52,
    )

    assert config.effective_max_curvature == pytest.approx(
        math.tan(0.52) / 2.63
    )


def test_stop_zone_preserves_nonzero_route_speed_for_decision_layer():
    assert select_target_speed(
        route_state='stop_zone',
        speed_limit=3.0,
        default_speed=2.0,
        max_speed=8.0,
        stop_speed=0.0,
    ) == pytest.approx(3.0)


@pytest.mark.parametrize(
    'route_state',
    ['intersection_approach', 'intersection', 'intersection_exit'],
)
def test_intersection_state_does_not_impose_generic_slowdown(route_state):
    assert select_target_speed(
        route_state=route_state,
        speed_limit=6.0,
        default_speed=2.0,
        max_speed=8.0,
        stop_speed=0.0,
    ) == pytest.approx(6.0)


def test_route_complete_remains_stopped():
    assert select_target_speed(
        route_state='route_complete',
        speed_limit=3.0,
        default_speed=2.0,
        max_speed=8.0,
        stop_speed=0.0,
    ) == pytest.approx(0.0)


def test_prefers_valid_local_path_over_target_pose():
    generator = make_generator()

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=10.0, y=0.0),
        local_path=[Pose2D(x=0.0, y=0.0), Pose2D(x=2.0, y=0.0)],
        speed_limit=5.0,
        route_state='normal',
        current_speed=5.0,
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
        route_state='normal',
    )

    assert [point.x for point in trajectory.path_points] == pytest.approx(
        [0.0, 1.0, 2.0]
    )
    assert 'local path rejected' in trajectory.path_status
    assert 'fallback' in trajectory.path_status


def test_returns_empty_path_and_nominal_speed_without_any_path_source():
    generator = make_generator()

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=None,
        local_path=[],
        speed_limit=4.0,
        route_state='normal',
    )

    assert trajectory.path_points == []
    assert trajectory.target_speed == pytest.approx(4.0)


def test_outputs_time_parameterized_speed_for_existing_control_interface():
    generator = TrajectoryGenerator(
        TrajectoryGeneratorConfig(
            path_resolution=1.0,
            min_target_distance=0.5,
            default_speed=2.0,
            max_speed=8.0,
            stop_speed=0.0,
            max_lateral_acceleration=2.0,
            max_longitudinal_acceleration=1.0,
            max_longitudinal_deceleration=1.0,
            target_speed_lookahead_distance=1.0,
        )
    )

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=3.0, y=0.0),
        local_path=[],
        speed_limit=8.0,
        route_state='normal',
        current_speed=0.0,
    )

    assert len(trajectory.trajectory_samples) == len(trajectory.path_points)
    assert trajectory.target_speed == pytest.approx(2.0 ** 0.5)
    assert trajectory.trajectory_samples[0].time_from_start == pytest.approx(0.0)
    assert all(
        later.time_from_start > earlier.time_from_start
        for earlier, later in zip(
            trajectory.trajectory_samples,
            trajectory.trajectory_samples[1:],
        )
    )


def test_commands_safe_stop_when_dynamic_profile_is_unreachable():
    generator = TrajectoryGenerator(
        TrajectoryGeneratorConfig(
            path_resolution=1.0,
            min_target_distance=0.5,
            default_speed=2.0,
            max_speed=8.0,
            stop_speed=0.0,
            max_lateral_acceleration=2.0,
            max_longitudinal_acceleration=1.0,
            max_longitudinal_deceleration=1.0,
        )
    )

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=1.0, y=0.0),
        local_path=[],
        speed_limit=2.0,
        route_state='normal',
        current_speed=8.0,
    )

    assert trajectory.path_points
    assert trajectory.trajectory_samples == []
    assert trajectory.target_speed == pytest.approx(0.0)
    assert 'insufficient braking distance' in trajectory.path_status


def test_commands_safe_stop_without_fresh_measured_speed():
    generator = make_generator()

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=3.0, y=0.0),
        local_path=[],
        speed_limit=5.0,
        route_state='normal',
        current_speed=None,
    )

    assert trajectory.path_points
    assert trajectory.trajectory_samples == []
    assert trajectory.target_speed == pytest.approx(0.0)
    assert 'measured speed unavailable' in trajectory.path_status


def test_falls_back_when_local_path_contains_non_finite_coordinates():
    generator = make_generator()

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=2.0, y=0.0),
        local_path=[Pose2D(x=0.0, y=0.0), Pose2D(x=float('nan'), y=0.0)],
        speed_limit=5.0,
        route_state='normal',
    )

    assert [point.x for point in trajectory.path_points] == pytest.approx(
        [0.0, 1.0, 2.0]
    )


def test_fallback_path_contains_arc_length_and_geometry():
    generator = make_generator()

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=2.0, y=0.0),
        local_path=[],
        speed_limit=5.0,
        route_state='normal',
    )

    assert [point.s for point in trajectory.path_points] == pytest.approx(
        [0.0, 1.0, 2.0]
    )
    assert [point.yaw for point in trajectory.path_points] == pytest.approx(
        [0.0, 0.0, 0.0]
    )
    assert [point.curvature for point in trajectory.path_points] == pytest.approx(
        [0.0, 0.0, 0.0]
    )


def test_blends_changed_local_path_with_previous_cycle_near_ego():
    generator = make_generator()
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    first = generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[Pose2D(x=0.0, y=0.0), Pose2D(x=8.0, y=0.0)],
        speed_limit=5.0,
        route_state='normal',
    )
    second = generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[Pose2D(x=0.0, y=1.0), Pose2D(x=8.0, y=1.0)],
        speed_limit=5.0,
        route_state='normal',
    )

    assert first.path_points[0].y == pytest.approx(0.0)
    assert 0.0 < second.path_points[0].y < 1.0
    assert second.path_points[-1].y == pytest.approx(1.0)
    assert all(
        end.s > start.s
        for start, end in zip(second.path_points, second.path_points[1:])
    )


def test_empty_path_clears_continuity_state():
    generator = make_generator()
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[Pose2D(x=0.0, y=0.0), Pose2D(x=8.0, y=0.0)],
        speed_limit=5.0,
        route_state='normal',
    )
    empty = generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[],
        speed_limit=5.0,
        route_state='normal',
    )
    resumed = generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[Pose2D(x=0.0, y=1.0), Pose2D(x=8.0, y=1.0)],
        speed_limit=5.0,
        route_state='normal',
    )

    assert empty.path_points == []
    assert resumed.path_points[0].y == pytest.approx(1.0)


def test_non_finite_fallback_target_produces_empty_path():
    generator = make_generator()

    trajectory = generator.generate(
        ego_pose=Pose2D(x=0.0, y=0.0, yaw=0.0),
        target_pose=Pose2D(x=float('nan'), y=0.0),
        local_path=[],
        speed_limit=5.0,
        route_state='normal',
    )

    assert trajectory.path_points == []


def test_does_not_blend_when_path_source_changes():
    generator = make_generator()
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    generator.generate(
        ego_pose=ego_pose,
        target_pose=Pose2D(x=8.0, y=0.0),
        local_path=[],
        speed_limit=5.0,
        route_state='normal',
    )
    local_trajectory = generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[Pose2D(x=0.0, y=1.0), Pose2D(x=8.0, y=1.0)],
        speed_limit=5.0,
        route_state='normal',
    )

    assert local_trajectory.path_points[0].y == pytest.approx(1.0)


def test_does_not_blend_paths_beyond_continuity_deviation_limit():
    generator = make_generator()
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[Pose2D(x=0.0, y=0.0), Pose2D(x=8.0, y=0.0)],
        speed_limit=5.0,
        route_state='normal',
    )
    shifted = generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[Pose2D(x=0.0, y=3.0), Pose2D(x=8.0, y=3.0)],
        speed_limit=5.0,
        route_state='normal',
    )

    assert shifted.path_points[0].y == pytest.approx(3.0)


def test_does_not_blend_same_origin_reroute():
    generator = make_generator()
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)

    generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[Pose2D(x=0.0, y=0.0), Pose2D(x=8.0, y=0.0)],
        speed_limit=5.0,
        route_state='normal',
    )
    rerouted = generator.generate(
        ego_pose=ego_pose,
        target_pose=None,
        local_path=[Pose2D(x=0.0, y=0.0), Pose2D(x=0.0, y=8.0)],
        speed_limit=5.0,
        route_state='normal',
    )

    assert [point.x for point in rerouted.path_points] == pytest.approx([0.0] * 9)
    assert [point.y for point in rerouted.path_points] == pytest.approx(
        list(range(9))
    )


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('path_resolution', 0.0),
        ('smoothing_strength', float('nan')),
        ('max_curvature', -1.0),
        ('continuity_blend_distance', -1.0),
        ('wheelbase', 0.0),
        ('maximum_steering_angle', 0.0),
    ],
)
def test_rejects_invalid_configuration(field, value):
    values = {
        'path_resolution': 1.0,
        'min_target_distance': 0.5,
        'default_speed': 2.0,
        'max_speed': 8.0,
        'stop_speed': 0.0,
    }
    values[field] = value

    with pytest.raises(ValueError, match=field):
        TrajectoryGeneratorConfig(**values)
