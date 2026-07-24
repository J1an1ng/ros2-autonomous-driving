"""Protect the production route geometry validated by closed-loop driving."""

import math
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


def test_production_route_retains_validated_pre_event2_corner_geometry():
    """Prevent release edits from restoring either pre-Event2 kerb cut."""
    waypoints = load_waypoints()

    # The waypoint32 right turn and the event-intersection left turn were
    # validated in Stage 5. Reverting either set made the vehicle cut a kerb.
    assert_waypoint(waypoints, 33, 131.004, 3.661, 165.745, 3)
    assert_waypoint(waypoints, 34, 127.000, 3.700, 150.000, 3)
    assert_waypoint(waypoints, 35, 123.747, 8.864, 94.822, 3)
    assert_waypoint(waypoints, 39, 123.700, 45.500, 105.000, 2)
    assert_waypoint(waypoints, 40, 122.000, 48.500, 125.000, 2)
    assert_waypoint(waypoints, 41, 119.000, 50.800, 155.000, 2)
    assert_waypoint(waypoints, 42, 114.500, 52.000, 178.000, 2)

    for first_id, second_id in zip(range(39, 42), range(40, 43)):
        first = waypoints[first_id]
        second = waypoints[second_id]
        spacing = math.hypot(
            second['x'] - first['x'],
            second['y'] - first['y'],
        )
        assert spacing > 3.0


def test_route_still_contains_all_final_waypoints():
    """Keep the required final route extension through waypoint 63."""
    waypoints = load_waypoints()

    assert sorted(waypoints) == list(range(64))
    assert waypoints[63]['type'] == 'curve_exit'
