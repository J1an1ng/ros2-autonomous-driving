import pytest

from route_manager.route_progress import distance_2d, waypoint_reached_or_passed


WAYPOINTS = [
    {"id": 0, "x": 0.0, "y": 0.0},
    {"id": 1, "x": 10.0, "y": 0.0},
    {"id": 2, "x": 20.0, "y": 0.0},
]


def test_waypoint_is_reached_inside_configured_radius():
    assert waypoint_reached_or_passed(WAYPOINTS, 1, 8.0, 1.0, 3.0) == "reached"


def test_waypoint_can_override_global_reach_radius():
    waypoints = [
        WAYPOINTS[0],
        {**WAYPOINTS[1], "reach_distance": 1.5},
        WAYPOINTS[2],
    ]

    assert waypoint_reached_or_passed(waypoints, 1, 8.0, 0.0, 3.0) == ""
    assert waypoint_reached_or_passed(waypoints, 1, 8.5, 0.0, 3.0) == "reached"


def test_waypoint_is_not_reached_before_its_progress_plane():
    assert waypoint_reached_or_passed(WAYPOINTS, 1, 6.0, 4.0, 3.0) == ""


def test_waypoint_is_passed_after_crossing_its_progress_plane():
    assert waypoint_reached_or_passed(WAYPOINTS, 1, 10.5, 4.0, 3.0) == "passed"


def test_first_waypoint_cannot_be_skipped_without_reaching_it():
    assert waypoint_reached_or_passed(WAYPOINTS, 0, 1.0, 4.0, 3.0) == ""


def test_zero_length_segment_requires_reach_radius():
    waypoints = [WAYPOINTS[0], {"id": 1, "x": 0.0, "y": 0.0}]
    assert waypoint_reached_or_passed(waypoints, 1, 4.0, 0.0, 3.0) == ""


def test_waypoint_distance_uses_planar_metric():
    assert distance_2d(1.0, 2.0, 4.0, 6.0) == pytest.approx(5.0)
