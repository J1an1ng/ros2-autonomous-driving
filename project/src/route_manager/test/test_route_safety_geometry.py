"""Protect the production route geometry validated by closed-loop driving."""

from pathlib import Path

import pytest
import yaml


ROUTE_FILE = Path(__file__).parents[1] / 'config' / 'route_map.yaml'


def load_waypoints():
    """Load production waypoints keyed by their public route ID."""
    route = yaml.safe_load(ROUTE_FILE.read_text(encoding='utf-8'))
    return {int(waypoint['id']): waypoint for waypoint in route['waypoints']}


def assert_waypoint(waypoints, waypoint_id, x, y, yaw_deg, speed_limit):
    """Assert the safety-relevant fields of one route waypoint."""
    waypoint = waypoints[waypoint_id]
    assert (waypoint['x'], waypoint['y']) == pytest.approx((x, y))
    assert waypoint['yaw_deg'] == pytest.approx(yaw_deg)
    assert waypoint['speed_limit'] == pytest.approx(speed_limit)


def test_production_route_retains_validated_waypoint32_corner_geometry():
    """Prevent release edits from restoring the waypoint32 kerb cut."""
    waypoints = load_waypoints()

    assert_waypoint(waypoints, 32, 138.387, 4.231, 177.671, 8)
    assert_waypoint(waypoints, 33, 131.004, 3.661, 165.745, 3)
    assert_waypoint(waypoints, 34, 127.000, 3.700, 150.000, 3)
    assert_waypoint(waypoints, 35, 123.747, 8.864, 94.822, 3)


def test_event_intersection_retains_v1_5_geometry_and_speed_profile():
    """Keep the faster v1.5 event-intersection turn at waypoints 39--42."""
    waypoints = load_waypoints()

    assert_waypoint(waypoints, 39, 124.192, 47.259, 89.559, 3)
    assert_waypoint(waypoints, 40, 118.965, 52.874, 137.028, 8)
    assert_waypoint(waypoints, 41, 118.238, 52.652, 163.920, 8)
    assert_waypoint(waypoints, 42, 114.305, 52.684, 179.954, 14)


def test_straight_intersections_preserve_road_speed():
    """Do not restore semantic slowdowns on straight-through intersections."""
    waypoints = load_waypoints()

    for waypoint_id in (8, 9, 10, 11, 12, 15, 16, 17, 18, 19, 20):
        assert waypoints[waypoint_id]['speed_limit'] == pytest.approx(14)


def test_route_still_contains_all_final_waypoints():
    """Keep the required final route extension through waypoint 63."""
    waypoints = load_waypoints()

    assert sorted(waypoints) == list(range(64))
    assert waypoints[63]['type'] == 'curve_exit'


def test_smooth_signal_waypoints_use_decision_controlled_approach_speed():
    """Do not restore far-field 3 m/s steps before the smooth signals."""
    waypoints = load_waypoints()

    assert_waypoint(waypoints, 25, 226.782, 15.711, -91.195, 8)
    assert waypoints[25]['reach_distance'] == pytest.approx(1.5)
    assert_waypoint(waypoints, 32, 138.387, 4.231, 177.671, 8)
    assert_waypoint(waypoints, 52, 46.719, 15.284, -90.926, 8)
    assert_waypoint(waypoints, 59, -49.42, 2.623, -177.646, 8)


def test_intersection_approach_zone_metadata_preserves_waypoint_speed():
    """Semantic zone metadata must not reintroduce a generic slowdown."""
    route = yaml.safe_load(ROUTE_FILE.read_text(encoding='utf-8'))
    waypoint_speeds = {
        int(waypoint['id']): float(waypoint['speed_limit'])
        for waypoint in route['waypoints']
    }
    approach_zones = [
        zone for zone in route['zones']
        if zone['type'] == 'intersection_approach'
    ]

    assert approach_zones
    for zone in approach_zones:
        assert zone['speed_limit'] == pytest.approx(
            waypoint_speeds[int(zone['related_waypoint'])]
        )
