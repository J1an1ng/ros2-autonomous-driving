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


def test_route_still_contains_all_final_waypoints():
    """Keep the required final route extension through waypoint 63."""
    waypoints = load_waypoints()

    assert sorted(waypoints) == list(range(64))
    assert waypoints[63]['type'] == 'curve_exit'


def test_waypoint59_uses_decision_controlled_approach_speed():
    """Do not restore the far-field 3 m/s step before the final signal."""
    waypoints = load_waypoints()

    assert_waypoint(waypoints, 59, -49.42, 2.623, -177.646, 8)
