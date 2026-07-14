# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Behavior tests for route progress initialization and advancement."""

import math
from pathlib import Path

from route_manager.route_progress import RouteProgress

import yaml


def load_route_waypoints():
    """Load the route used by the runtime node."""
    route_file = Path(__file__).parents[1] / 'config' / 'route_map.yaml'
    with route_file.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)['waypoints']


def test_initializes_near_waypoint_19():
    """Initialization selects waypoint 19 near the simulated start pose."""
    waypoints = load_route_waypoints()
    progress = RouteProgress(waypoints, reach_distance=3.0)
    waypoint = waypoints[19]

    progress.update(
        ego_x=float(waypoint['x']) + 0.2,
        ego_y=float(waypoint['y']) + 0.1,
        ego_yaw=math.radians(float(waypoint['yaw_deg'])),
    )

    assert progress.current_index == 19


def test_heading_disambiguates_overlapping_route_directions():
    """Heading selects the matching direction at overlapping positions."""
    waypoints = [
        {'x': 10.0, 'y': 20.0, 'yaw_deg': 180.0},
        {'x': 10.0, 'y': 20.0, 'yaw_deg': 0.0},
    ]
    progress = RouteProgress(waypoints, reach_distance=3.0)

    progress.update(ego_x=10.0, ego_y=20.0, ego_yaw=0.0)

    assert progress.current_index == 1


def test_initialization_callback_does_not_immediately_advance():
    """The initialization update cannot also mark its selection reached."""
    waypoints = load_route_waypoints()
    progress = RouteProgress(waypoints, reach_distance=3.0)
    waypoint = waypoints[19]
    ego_pose = {
        'ego_x': float(waypoint['x']),
        'ego_y': float(waypoint['y']),
        'ego_yaw': math.radians(float(waypoint['yaw_deg'])),
    }

    progress.update(**ego_pose)

    assert progress.current_index == 19

    progress.update(**ego_pose)

    assert progress.current_index == 20
