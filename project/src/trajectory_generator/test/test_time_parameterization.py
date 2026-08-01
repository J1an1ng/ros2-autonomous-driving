import pytest

from trajectory_generator.path_processor import PathPoint
from trajectory_generator.velocity_profile import (
    DynamicLimits,
    time_parameterize,
)


def path_point(s, curvature=0.0):
    return PathPoint(
        x=s,
        y=0.0,
        z=0.0,
        yaw=0.0,
        s=s,
        curvature=curvature,
    )


def test_curvature_caps_speed_by_lateral_acceleration():
    samples = time_parameterize(
        path_points=[path_point(0.0), path_point(1.0, curvature=0.5)],
        initial_speed=8.0,
        speed_limit=8.0,
        limits=DynamicLimits(
            max_speed=8.0,
            max_lateral_acceleration=2.0,
            max_longitudinal_acceleration=100.0,
            max_longitudinal_deceleration=100.0,
        ),
    )

    assert samples[1].target_speed == pytest.approx(2.0)


def test_forward_pass_limits_longitudinal_acceleration():
    samples = time_parameterize(
        path_points=[path_point(0.0), path_point(1.0), path_point(2.0)],
        initial_speed=0.0,
        speed_limit=8.0,
        limits=DynamicLimits(
            max_speed=8.0,
            max_lateral_acceleration=100.0,
            max_longitudinal_acceleration=1.0,
            max_longitudinal_deceleration=100.0,
        ),
    )

    assert [sample.target_speed for sample in samples] == pytest.approx(
        [0.0, 2.0 ** 0.5, 2.0]
    )


def test_backward_pass_brakes_before_a_curvature_limit():
    samples = time_parameterize(
        path_points=[
            path_point(0.0),
            path_point(1.0),
            path_point(2.0),
            path_point(3.0, curvature=0.5),
        ],
        initial_speed=3.0,
        speed_limit=8.0,
        limits=DynamicLimits(
            max_speed=8.0,
            max_lateral_acceleration=2.0,
            max_longitudinal_acceleration=100.0,
            max_longitudinal_deceleration=1.0,
        ),
    )

    assert [sample.target_speed for sample in samples] == pytest.approx(
        [3.0, 8.0 ** 0.5, 6.0 ** 0.5, 2.0]
    )


def test_rejects_profile_when_braking_distance_is_insufficient():
    with pytest.raises(ValueError, match='insufficient braking distance'):
        time_parameterize(
            path_points=[path_point(0.0), path_point(1.0, curvature=0.5)],
            initial_speed=8.0,
            speed_limit=8.0,
            limits=DynamicLimits(
                max_speed=8.0,
                max_lateral_acceleration=2.0,
                max_longitudinal_acceleration=1.0,
                max_longitudinal_deceleration=1.0,
            ),
        )


def test_terminal_stop_has_monotonic_time_parameterization():
    samples = time_parameterize(
        path_points=[path_point(0.0), path_point(1.0), path_point(2.0)],
        initial_speed=1.0,
        speed_limit=3.0,
        limits=DynamicLimits(
            max_speed=3.0,
            max_lateral_acceleration=2.0,
            max_longitudinal_acceleration=1.0,
            max_longitudinal_deceleration=1.0,
        ),
        terminal_speed=0.0,
    )

    assert samples[-1].target_speed == pytest.approx(0.0)
    assert samples[0].time_from_start == pytest.approx(0.0)
    assert all(
        later.time_from_start > earlier.time_from_start
        for earlier, later in zip(samples, samples[1:])
    )
