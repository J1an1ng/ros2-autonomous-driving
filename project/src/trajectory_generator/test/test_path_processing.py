import pytest

from trajectory_generator.geometry_utils import Pose2D
from trajectory_generator.path_processor import process_local_path


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

    result = process_local_path(route_points, ego_pose, path_resolution=1.0)

    assert [(point.x, point.y) for point in result] == pytest.approx(
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    )


def test_crops_path_to_nearest_point_forward_of_ego():
    ego_pose = Pose2D(x=0.2, y=0.0, yaw=0.0)
    route_points = [Pose2D(x=-2.0, y=0.0), Pose2D(x=3.0, y=0.0)]

    result = process_local_path(route_points, ego_pose, path_resolution=1.0)

    assert [point.x for point in result] == pytest.approx([1.0, 2.0, 3.0])


def test_truncates_path_when_it_bends_back_behind_ego():
    ego_pose = Pose2D(x=0.0, y=0.0, yaw=0.0)
    route_points = [
        Pose2D(x=-1.0, y=0.0),
        Pose2D(x=2.0, y=0.0),
        Pose2D(x=-1.0, y=0.0),
    ]

    result = process_local_path(route_points, ego_pose, path_resolution=1.0)

    assert [point.x for point in result] == pytest.approx(
        [0.0, 1.0, 2.0, 1.0, 0.0]
    )
