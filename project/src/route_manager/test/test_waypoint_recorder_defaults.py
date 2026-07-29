"""Tests for semantic waypoint recording defaults."""

from route_manager.interactive_waypoint_recorder import (
    DEFAULT_WAYPOINT_SPEED,
    InteractiveWaypointRecorder,
    TYPE_CONFIG,
)


def test_intersection_types_use_the_normal_waypoint_speed():
    """Intersection semantics must not imply a generic slowdown."""
    assert TYPE_CONFIG['n'][1] == DEFAULT_WAYPOINT_SPEED
    assert TYPE_CONFIG['ia'][1] == DEFAULT_WAYPOINT_SPEED
    assert TYPE_CONFIG['i'][1] == DEFAULT_WAYPOINT_SPEED
    assert TYPE_CONFIG['ix'][1] == DEFAULT_WAYPOINT_SPEED


def test_intersection_approach_zone_preserves_recorded_speed():
    """Zone metadata must not reintroduce a hard-coded speed penalty."""
    recorder = object.__new__(InteractiveWaypointRecorder)
    recorder.waypoints = [{
        'id': 7,
        'type': 'intersection_approach',
        'speed_limit': 6.0,
    }]

    zones = recorder.build_zones()

    assert zones[0]['speed_limit'] == 6.0
