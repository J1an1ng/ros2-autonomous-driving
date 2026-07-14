import math

import pytest

from trajectory_generator.geometry_utils import Pose2D
from trajectory_generator.path_processor import (
    PathPoint,
    process_local_path,
    validate_path,
)


def test_projects_ego_onto_polyline_before_resampling():
    ego_pose = Pose2D(x=3.25, y=2.0, yaw=0.0)
    route_points = [Pose2D(x=0.0, y=0.0), Pose2D(x=10.0, y=0.0)]

    result = process_local_path(route_points, ego_pose, path_resolution=2.0)

    assert [(point.x, point.y) for point in result] == pytest.approx(
        [(3.25, 0.0), (5.25, 0.0), (7.25, 0.0), (9.25, 0.0), (10.0, 0.0)]
    )
    assert [point.s for point in result] == pytest.approx(
        [0.0, 2.0, 4.0, 6.0, 6.75]
    )


def test_resamples_local_path_at_configured_resolution():
    ego_pose = Pose2D(x=-0.1, y=0.0, yaw=0.0)
    route_points = [
        Pose2D(x=0.0, y=0.0, z=0.0),
        Pose2D(x=2.5, y=0.0, z=1.0),
    ]

    result = process_local_path(route_points, ego_pose, path_resolution=1.0)

    assert [(point.x, point.y) for point in result] == pytest.approx(
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.5, 0.0)]
    )
    assert [point.z for point in result] == pytest.approx([0.0, 0.4, 0.8, 1.0])
    assert [point.yaw for point in result] == pytest.approx([0.0] * 4)


def test_removes_duplicate_and_nearly_identical_consecutive_points():
    ego_pose = Pose2D(x=-0.1, y=0.0, yaw=0.0)
    route_points = [
        Pose2D(x=0.0, y=0.0),
        Pose2D(x=0.0, y=0.0),
        Pose2D(x=0.0005, y=0.0005),
        Pose2D(x=2.0, y=0.0),
    ]

    result = process_local_path(
        route_points,
        ego_pose,
        path_resolution=1.0,
    )

    assert [(point.x, point.y) for point in result] == pytest.approx(
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    )


def test_crops_path_from_exact_ego_projection():
    ego_pose = Pose2D(x=0.2, y=0.0, yaw=0.0)
    route_points = [Pose2D(x=-2.0, y=0.0), Pose2D(x=3.0, y=0.0)]

    result = process_local_path(route_points, ego_pose, path_resolution=1.0)

    assert [point.x for point in result] == pytest.approx([0.2, 1.2, 2.2, 3.0])


def test_arc_length_crop_preserves_ordered_route_through_turnaround():
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    route_points = [
        Pose2D(x=-1.0, y=0.0),
        Pose2D(x=2.0, y=0.0),
        Pose2D(x=-1.0, y=0.0),
    ]

    result = process_local_path(
        route_points,
        ego_pose,
        path_resolution=1.0,
        max_yaw_step=math.pi + 1e-3,
    )

    assert [point.x for point in result] == pytest.approx(
        [0.0, 1.0, 2.0, 1.0, 0.0, -1.0]
    )
    assert [point.s for point in result] == pytest.approx(
        [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    )


def test_spline_smooths_polyline_corner_without_overshoot():
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    route_points = [
        Pose2D(x=0.0, y=0.0),
        Pose2D(x=5.0, y=0.0),
        Pose2D(x=5.0, y=5.0),
    ]

    result = process_local_path(route_points, ego_pose, path_resolution=0.5)

    corner_points = [
        point for point in result if 3.75 < point.x < 5.0 and 0.0 < point.y < 1.25
    ]
    assert corner_points
    assert all(-1e-9 <= point.x <= 5.0 + 1e-9 for point in result)
    assert all(-1e-9 <= point.y <= 5.0 + 1e-9 for point in result)


def test_computes_yaw_arc_length_and_signed_curvature():
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    route_points = [
        Pose2D(x=0.0, y=0.0),
        Pose2D(x=5.0, y=0.0),
        Pose2D(x=5.0, y=5.0),
    ]

    result = process_local_path(route_points, ego_pose, path_resolution=0.25)

    assert all(math.isfinite(point.curvature) for point in result)
    assert max(point.curvature for point in result) > 0.1
    assert abs(result[1].curvature) < 1e-6
    assert all(end.s > start.s for start, end in zip(result, result[1:]))
    yaw_steps = [
        abs(math.atan2(math.sin(end.yaw - start.yaw), math.cos(end.yaw - start.yaw)))
        for start, end in zip(result, result[1:])
    ]
    assert max(yaw_steps) < 0.35


def test_quality_validation_rejects_excessive_curvature():
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    route_points = [
        Pose2D(x=0.0, y=0.0),
        Pose2D(x=5.0, y=0.0),
        Pose2D(x=5.0, y=5.0),
    ]

    accepted = process_local_path(
        route_points,
        ego_pose,
        path_resolution=0.25,
        max_curvature=2.0,
    )
    rejected = process_local_path(
        route_points,
        ego_pose,
        path_resolution=0.25,
        max_curvature=0.05,
    )

    assert accepted
    assert rejected == []


def test_ego_heading_disambiguates_overlapping_route_segments():
    ego_pose = Pose2D(x=5.0, y=0.5, yaw=math.pi)
    route_points = [
        Pose2D(x=0.0, y=0.0),
        Pose2D(x=10.0, y=0.0),
        Pose2D(x=10.0, y=1.0),
        Pose2D(x=0.0, y=1.0),
    ]

    result = process_local_path(route_points, ego_pose, path_resolution=1.0)

    assert (result[0].x, result[0].y) == pytest.approx((5.0, 1.0))
    assert (result[-1].x, result[-1].y) == pytest.approx((0.0, 1.0))
    assert result[0].yaw == pytest.approx(math.pi)


def test_projection_prefers_near_route_over_distant_heading_match():
    ego_pose = Pose2D(x=5.0, y=0.0, yaw=math.pi)
    route_points = [
        Pose2D(x=0.0, y=0.0),
        Pose2D(x=10.0, y=0.0),
        Pose2D(x=10.0, y=2.0),
        Pose2D(x=0.0, y=2.0),
    ]

    result = process_local_path(
        route_points,
        ego_pose,
        path_resolution=1.0,
        max_curvature=10.0,
        max_yaw_step=math.pi,
    )

    assert (result[0].x, result[0].y) == pytest.approx((5.0, 0.0))


def test_quality_validation_rejects_each_geometric_limit():
    valid_path = [
        PathPoint(x=0.0, y=0.0, z=0.0, yaw=0.0, s=0.0),
        PathPoint(x=1.0, y=0.0, z=0.0, yaw=0.0, s=1.0),
    ]
    limits = {
        "path_resolution": 1.0,
        "max_curvature": 2.0,
        "max_yaw_step": 1.1,
        "max_start_distance": 5.0,
        "max_spline_deviation": 2.0,
    }

    assert validate_path(valid_path, **limits)
    assert not validate_path(
        [valid_path[0], PathPoint(1.0, 0.0, 0.0, 1.2, s=1.0)],
        **limits,
    )
    assert not validate_path(
        [valid_path[0], PathPoint(2.0, 0.0, 0.0, 0.0, s=2.0)],
        **limits,
    )
    assert not validate_path(
        valid_path,
        ego_pose=Pose2D(x=10.0, y=0.0),
        **limits,
    )
    assert not validate_path(
        valid_path,
        source_points=[Pose2D(x=0.0, y=3.0), Pose2D(x=1.0, y=3.0)],
        **limits,
    )


def test_default_quality_limits_accept_tight_route_turn():
    ego_pose = Pose2D(x=181.421, y=4.351, yaw=math.radians(179.644))
    route_points = [
        ego_pose,
        Pose2D(x=138.387, y=4.231),
        Pose2D(x=131.004, y=3.661),
        Pose2D(x=123.747, y=8.293),
        Pose2D(x=124.346, y=8.864),
    ]

    result = process_local_path(route_points, ego_pose, path_resolution=1.0)

    assert result
    assert max(abs(point.curvature) for point in result) < 2.0
