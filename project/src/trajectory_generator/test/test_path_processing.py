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
        Pose2D(x=138.387, y=4.231, yaw=math.radians(177.671)),
        Pose2D(x=131.004, y=3.661, yaw=math.radians(165.745)),
        Pose2D(x=127.000, y=3.700, yaw=math.radians(150.0)),
        Pose2D(x=123.747, y=8.864, yaw=math.radians(94.822)),
    ]

    result = process_local_path(
        route_points,
        ego_pose,
        path_resolution=1.0,
        use_waypoint_yaw=True,
    )

    assert result
    assert max(abs(point.curvature) for point in result) < 2.0
    # The T-junction reference must turn between the two observed failure modes:
    # it stays below y=4 m around x=128--129 m to avoid the inner kerb, but has
    # already moved north by x=125--126 m to avoid the former late-turn overrun.
    preview_points = [point for point in result if 128.0 <= point.x <= 129.0]
    assert preview_points
    assert max(point.y for point in preview_points) < 4.0
    turn_points = [point for point in result if 125.0 <= point.x <= 126.0]
    assert turn_points
    assert max(point.y for point in turn_points) > 4.4


def test_waypoint_yaw_interpolation_starts_sparse_turn_before_corner():
    route_points = [
        Pose2D(x=0.0, y=0.0, yaw=math.radians(80.0)),
        Pose2D(x=2.0, y=10.0, yaw=math.radians(50.0)),
        Pose2D(x=10.0, y=16.0, yaw=math.radians(10.0)),
    ]

    result = process_local_path(
        route_points,
        ego_pose=route_points[0],
        path_resolution=0.5,
        use_waypoint_yaw=True,
    )

    first_half = [point for point in result if point.s < 8.0]
    assert result
    assert any(point.curvature < -0.01 for point in first_half)


def test_event_intersection_left_turn_stays_clear_of_opposite_kerb():
    route_points = [
        Pose2D(x=124.120, y=40.206, yaw=math.radians(89.324)),
        Pose2D(x=123.700, y=45.500, yaw=math.radians(105.0)),
        Pose2D(x=122.000, y=48.500, yaw=math.radians(125.0)),
        Pose2D(x=119.000, y=50.800, yaw=math.radians(155.0)),
        Pose2D(x=114.500, y=52.000, yaw=math.radians(178.0)),
    ]

    result = process_local_path(
        route_points,
        ego_pose=route_points[0],
        path_resolution=1.0,
        use_waypoint_yaw=True,
    )

    assert result
    assert max(point.y for point in result) < 52.1
    corner_points = [point for point in result if 47.0 <= point.y <= 49.0]
    assert corner_points
    assert min(point.x for point in corner_points) < 122.8

    # Route progress uses a 3 m reach radius. Keep every turn-defining point
    # outside the previous point's radius so the bend cannot be skipped in one
    # update cycle.
    for first, second in zip(route_points, route_points[1:]):
        assert math.hypot(second.x - first.x, second.y - first.y) > 3.0


def test_waypoint48_turn_does_not_bulge_toward_road_end_barrier():
    route_points = [
        Pose2D(x=81.514, y=52.500, yaw=math.radians(178.441)),
        Pose2D(x=65.590, y=52.400, yaw=math.radians(180.0)),
        Pose2D(x=49.690, y=51.719, yaw=math.radians(-170.0)),
        Pose2D(x=48.266, y=50.114, yaw=math.radians(-131.668)),
        Pose2D(x=46.719, y=46.420, yaw=math.radians(-108.342)),
        Pose2D(x=45.588, y=35.993, yaw=math.radians(-89.76)),
    ]

    result = process_local_path(
        route_points,
        ego_pose=route_points[0],
        path_resolution=1.0,
        use_waypoint_yaw=True,
    )

    assert result
    barrier_approach = [point for point in result if 55.0 <= point.x <= 65.0]
    assert barrier_approach
    assert max(point.y for point in barrier_approach) < 52.5
    assert max(abs(point.curvature) for point in result) < 0.6


def test_submetre_route_cluster_keeps_latest_point_and_tangent():
    route_points = [
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        Pose2D(x=5.0, y=0.0, yaw=0.0),
        Pose2D(x=5.5, y=0.1, yaw=math.radians(30.0)),
        Pose2D(x=10.0, y=3.0, yaw=math.radians(30.0)),
    ]

    result = process_local_path(
        route_points,
        ego_pose=route_points[0],
        path_resolution=0.5,
        use_waypoint_yaw=True,
    )

    assert result
    assert max(abs(point.curvature) for point in result) < 2.0


def test_projection_at_waypoint_preserves_outgoing_tangent_yaw():
    corner = Pose2D(x=5.0, y=0.0, yaw=math.radians(45.0))
    route_points = [
        Pose2D(x=0.0, y=0.0, yaw=0.0),
        corner,
        Pose2D(x=10.0, y=5.0, yaw=math.radians(45.0)),
    ]

    result = process_local_path(
        route_points,
        ego_pose=corner,
        path_resolution=0.25,
        use_waypoint_yaw=True,
    )

    assert result
    assert result[0].yaw == pytest.approx(math.radians(45.0), abs=0.05)
