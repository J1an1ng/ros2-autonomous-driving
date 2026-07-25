import math

import pytest

from decision_making.path_corridor import (
    in_vehicle_safety_corridor,
    matches_fixed_structure_signature,
    prepare_path,
    project_to_path,
    robust_nearest_distance,
)


def test_straight_path_projection_reports_arc_length_and_signed_offset():
    path = prepare_path([(5.0, 0.0), (10.0, 0.0)])

    left = project_to_path(4.0, 1.0, path)
    right = project_to_path(7.0, -1.5, path)

    assert left.along == pytest.approx(4.0)
    assert left.lateral == pytest.approx(1.0)
    assert right.along == pytest.approx(7.0)
    assert right.lateral == pytest.approx(-1.5)


def test_turning_path_rejects_structure_ahead_but_off_future_path():
    path = prepare_path([(2.0, 0.0), (3.5, 1.0), (4.0, 3.0)])

    road_end = project_to_path(4.5, -0.5, path)
    on_turn = project_to_path(3.5, 1.0, path)

    assert road_end.distance > 1.2
    assert on_turn.distance == pytest.approx(0.0)
    assert on_turn.along > 2.0


def test_prepare_path_removes_invalid_and_duplicate_points():
    path = prepare_path(
        [(0.01, 0.01), (math.nan, 1.0), (1.0, 0.0), (1.01, 0.0)]
    )

    assert path == [(0.0, 0.0), (1.0, 0.0)]


def test_projection_requires_a_usable_segment():
    assert project_to_path(1.0, 0.0, []) is None
    assert project_to_path(1.0, 0.0, [(0.0, 0.0)]) is None


def test_vehicle_safety_corridor_covers_only_the_short_ego_lane_region():
    assert in_vehicle_safety_corridor(2.0, 0.9, 0.5, 3.5, 1.1)
    assert not in_vehicle_safety_corridor(4.0, 0.0, 0.5, 3.5, 1.1)
    assert not in_vehicle_safety_corridor(2.0, 1.2, 0.5, 3.5, 1.1)
    assert not in_vehicle_safety_corridor(-1.0, 0.0, 0.5, 3.5, 1.1)


def test_robust_nearest_distance_requires_multiple_supporting_points():
    assert math.isinf(robust_nearest_distance([1.0, 1.1, 1.2], 5))
    assert robust_nearest_distance(
        [0.2, 2.0, 2.1, 2.2, 2.3, 2.4], 5
    ) == pytest.approx(2.3)


def test_robust_nearest_distance_ignores_invalid_values():
    distance = robust_nearest_distance(
        [math.nan, math.inf, -1.0, 1.0, 1.1], 2
    )

    assert distance == pytest.approx(1.1)


def test_fixed_structure_signature_requires_all_corridors_at_mapped_waypoint():
    assert matches_fixed_structure_signature(
        48, [48], 9.45, 3.44, 7.56, 9.5, 4.0, 9.5
    )
    assert not matches_fixed_structure_signature(
        48, [48], 11.81, 2.49, 10.79, 9.5, 4.0, 9.5
    )
    assert not matches_fixed_structure_signature(
        47, [48], 3.08, 3.26, 3.08, 9.5, 4.0, 9.5
    )
