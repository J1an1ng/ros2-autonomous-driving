import math

import pytest

from decision_making.path_corridor import prepare_path, project_to_path


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
