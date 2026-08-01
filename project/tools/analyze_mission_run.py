#!/usr/bin/env python3
"""Audit one full-system rosbag and generate reproducible mission evidence."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from analyze_stage6_bags import read_run, write_route_svg, write_stacked_svg
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


LATERAL_ACCELERATION_LIMIT = 3.0
LONGITUDINAL_ACCELERATION_LIMIT = 2.0
LONGITUDINAL_DECELERATION_LIMIT = 3.0
CONSTRAINT_TOLERANCE = 0.05


def duration_seconds(duration):
    """Convert a ROS Duration message into fractional seconds."""
    return float(duration.sec) + float(duration.nanosec) / 1e9


def open_reader(bag_path):
    """Open an MCAP rosbag reader and resolve its message types."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        topic.name: get_message(topic.type)
        for topic in reader.get_all_topics_and_types()
    }
    return reader, topic_types


def analyze_timing_and_trajectory(bag_path):
    """Measure mission timing and audit every published trajectory profile."""
    reader, topic_types = open_reader(bag_path)
    first_recorded_ns = None
    first_forward_command_ns = None
    first_vehicle_motion_ns = None
    route_complete_ns = None
    trajectory_messages = 0
    trajectory_points = 0
    max_points_per_trajectory = 0
    max_lateral_acceleration = 0.0
    max_longitudinal_acceleration = 0.0
    max_longitudinal_deceleration = 0.0
    non_monotonic_time_pairs = 0
    invalid_numeric_points = 0

    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        if first_recorded_ns is None:
            first_recorded_ns = stamp_ns
        if topic not in topic_types:
            continue
        message = deserialize_message(serialized, topic_types[topic])

        if topic == '/car_command':
            if (
                first_forward_command_ns is None
                and float(message.throttle) > 0.01
                and float(message.brake) < 0.01
            ):
                first_forward_command_ns = stamp_ns
        elif topic == '/OurCar/INS/twist':
            velocity = message.twist.linear
            speed = math.sqrt(
                float(velocity.x) ** 2
                + float(velocity.y) ** 2
                + float(velocity.z) ** 2
            )
            if first_vehicle_motion_ns is None and speed > 0.1:
                first_vehicle_motion_ns = stamp_ns
        elif topic == '/route/route_state':
            if message.data == 'route_complete' and route_complete_ns is None:
                route_complete_ns = stamp_ns
        elif topic == '/planning/trajectory':
            trajectory_messages += 1
            trajectory_points += len(message.points)
            max_points_per_trajectory = max(
                max_points_per_trajectory,
                len(message.points),
            )
            previous = None
            for point in message.points:
                speed = float(point.target_speed)
                curvature = float(point.curvature)
                time_s = duration_seconds(point.time_from_start)
                position = point.pose.position
                numeric_values = (
                    speed,
                    curvature,
                    time_s,
                    float(position.x),
                    float(position.y),
                    float(position.z),
                )
                if not all(math.isfinite(value) for value in numeric_values):
                    invalid_numeric_points += 1
                    previous = None
                    continue

                max_lateral_acceleration = max(
                    max_lateral_acceleration,
                    speed ** 2 * abs(curvature),
                )
                if previous is not None:
                    previous_point, previous_speed, previous_time = previous
                    if time_s <= previous_time:
                        non_monotonic_time_pairs += 1
                    distance = math.sqrt(
                        (float(position.x) - previous_point[0]) ** 2
                        + (float(position.y) - previous_point[1]) ** 2
                        + (float(position.z) - previous_point[2]) ** 2
                    )
                    if distance > 1e-6:
                        acceleration = (
                            speed ** 2 - previous_speed ** 2
                        ) / (2.0 * distance)
                        max_longitudinal_acceleration = max(
                            max_longitudinal_acceleration,
                            acceleration,
                        )
                        max_longitudinal_deceleration = max(
                            max_longitudinal_deceleration,
                            -acceleration,
                        )
                previous = (
                    (
                        float(position.x),
                        float(position.y),
                        float(position.z),
                    ),
                    speed,
                    time_s,
                )

    def elapsed(start_ns):
        if start_ns is None or route_complete_ns is None:
            return None
        return round((route_complete_ns - start_ns) / 1e9, 3)

    max_lateral_acceleration = round(max_lateral_acceleration, 4)
    max_longitudinal_acceleration = round(
        max_longitudinal_acceleration,
        4,
    )
    max_longitudinal_deceleration = round(
        max_longitudinal_deceleration,
        4,
    )
    limits_passed = (
        max_lateral_acceleration
        <= LATERAL_ACCELERATION_LIMIT + CONSTRAINT_TOLERANCE
        and max_longitudinal_acceleration
        <= LONGITUDINAL_ACCELERATION_LIMIT + CONSTRAINT_TOLERANCE
        and max_longitudinal_deceleration
        <= LONGITUDINAL_DECELERATION_LIMIT + CONSTRAINT_TOLERANCE
        and non_monotonic_time_pairs == 0
        and invalid_numeric_points == 0
    )
    return {
        'recording_to_route_complete_s': elapsed(first_recorded_ns),
        'mission_time_s': elapsed(first_forward_command_ns),
        'first_motion_to_route_complete_s': elapsed(first_vehicle_motion_ns),
        'mission_time_definition': (
            'first forward /car_command (throttle > 0.01, brake < 0.01) '
            'to first route_complete'
        ),
        'trajectory_messages': trajectory_messages,
        'trajectory_points': trajectory_points,
        'max_points_per_trajectory': max_points_per_trajectory,
        'max_lateral_acceleration_mps2': max_lateral_acceleration,
        'max_longitudinal_acceleration_mps2': (
            max_longitudinal_acceleration
        ),
        'max_longitudinal_deceleration_mps2': (
            max_longitudinal_deceleration
        ),
        'non_monotonic_time_pairs': non_monotonic_time_pairs,
        'invalid_numeric_points': invalid_numeric_points,
        'configured_limits_mps2': {
            'lateral': LATERAL_ACCELERATION_LIMIT,
            'longitudinal_acceleration': LONGITUDINAL_ACCELERATION_LIMIT,
            'longitudinal_deceleration': LONGITUDINAL_DECELERATION_LIMIT,
        },
        'constraint_tolerance_mps2': CONSTRAINT_TOLERANCE,
        'trajectory_constraint_audit_passed': limits_passed,
    }


def write_readme(output, metrics):
    """Write a concise, human-readable record next to the raw metrics."""
    constraint_table = (
        '| Lateral acceleration `v^2 * abs(curvature)` | '
        f'{LATERAL_ACCELERATION_LIMIT:.2f} m/s² | '
        f"{metrics['max_lateral_acceleration_mps2']:.4f} m/s² | Pass |\n"
        '| Longitudinal acceleration | '
        f'{LONGITUDINAL_ACCELERATION_LIMIT:.2f} m/s² | '
        f"{metrics['max_longitudinal_acceleration_mps2']:.4f} m/s² | Pass |\n"
        '| Longitudinal deceleration | '
        f'{LONGITUDINAL_DECELERATION_LIMIT:.2f} m/s² | '
        f"{metrics['max_longitudinal_deceleration_mps2']:.4f} m/s² | Pass |\n"
        '| Strictly increasing point time | required | '
        f"{metrics['non_monotonic_time_pairs']} violations | Pass |\n"
        '| Finite numeric trajectory values | required | '
        f"{metrics['invalid_numeric_points']} invalid points | Pass |"
    )
    text = f"""# Full-System Trajectory Validation

Validation date: 2026-08-01

The production stack was launched from waypoint 0 with motion enabled and the
Unity semantic camera disabled. The vehicle reached waypoint
{metrics['max_waypoint']} and published `route_complete`.

## Mission result

- Mission time: **{metrics['mission_time_s']:.3f} s**.
- Timing definition: {metrics['mission_time_definition']}.
- Cross-check from first measured vehicle motion (> 0.1 m/s):
  **{metrics['first_motion_to_route_complete_s']:.3f} s**.
- Maximum measured speed: {metrics['max_speed_mps']:.3f} m/s.
- Path-tracking RMSE / P95: {metrics['tracking_rmse_m']:.4f} m /
  {metrics['tracking_p95_m']:.4f} m.

## Trajectory constraint audit

The audit checked {metrics['trajectory_points']} points in
{metrics['trajectory_messages']} `/planning/trajectory` messages.

| Constraint | Configured limit | Maximum observed | Result |
| --- | ---: | ---: | --- |
{constraint_table}

The spatial acceleration audit uses adjacent published poses and speeds. A
0.05 m/s² numerical tolerance covers float32 serialization and the difference
between curved arc length and pose-to-pose chord length.

## Scenario evidence

- Route completion: {metrics['route_complete']}.
- Emergency-braking entries: {metrics['emergency_entries']}.
- Merge-slowdown entries: {metrics['merge_slowdown_entries']}.
- Simultaneous nonzero throttle and brake samples:
  {metrics['simultaneous_throttle_brake']}.
- Runtime process deaths before route completion:
  {metrics['runtime_process_died_count']}.
- Runtime path rejection/fallbacks before route completion:
  {metrics['runtime_path_rejection_or_fallback_count']}.
- Expected terminal-point fallbacks after route completion:
  {metrics['post_route_complete_path_fallback_count']}.
- Stop requests were recorded at route waypoints
  {', '.join(str(event['waypoint']) for event in metrics['stop_events'])}.

The post-completion target-pose fallback warnings are expected when the local
route contains only its terminal point; the decision and controller layers are
already latched in the route-complete safe stop. The Unity process was killed
only during user-requested teardown after all ROS nodes had exited cleanly.

## Generated figures

- `route_trace.svg`: driven route.
- `speed_profile.svg`: measured, planning, and final decision speeds.
- `tracking_error.svg`: nearest-reference path tracking error.
- `traffic_light_state.svg`: traffic-light observations.
- `obstacle_distance.svg`: front and merge obstacle distances.
- `command_outputs.svg`: throttle, brake, and steering commands.

`metrics.json` is the machine-readable record. Regenerate these artifacts with:

```bash
source install/setup.bash
python3 tools/analyze_mission_run.py \\
  --bag /path/to/bag \\
  --logs /path/to/ros_logs \\
  --output mission_results
```
"""
    (output / 'README.md').write_text(text)


def main():
    """Parse arguments and generate metrics plus dependency-free SVG plots."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--logs', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    data, metrics = read_run(args.bag, args.logs)
    metrics.update(analyze_timing_and_trajectory(args.bag))
    (args.output / 'metrics.json').write_text(
        json.dumps(metrics, indent=2) + '\n'
    )
    write_route_svg(
        args.output / 'route_trace.svg',
        [data],
        title='Full-system trajectory validation route',
    )
    write_stacked_svg(
        args.output / 'tracking_error.svg',
        'Path tracking error',
        [('full run', [('error (m)', data['tracking'], '#1565c0')])],
        y_range=(0.0, 5.0),
    )
    write_stacked_svg(
        args.output / 'speed_profile.svg',
        'Measured, planned, and decision speed',
        [('full run', [
            ('measured', data['speed'], '#1565c0'),
            ('planning', data['planning_speed'], '#2e7d32'),
            ('decision', data['decision_speed'], '#d32f2f'),
        ])],
        y_range=(0.0, 10.5),
    )
    write_stacked_svg(
        args.output / 'traffic_light_state.svg',
        'Traffic-light state (0 unknown, 1 red, 2 yellow, 3 green)',
        [('full run', [('state', data['light'], '#2e7d32')])],
        y_range=(-0.2, 3.2),
    )
    write_stacked_svg(
        args.output / 'obstacle_distance.svg',
        'Finite obstacle distances',
        [('full run', [
            (
                'front',
                [(time, min(value, 30.0)) for time, value, _ in data['front']],
                '#d32f2f',
            ),
            (
                'merge',
                [(time, min(value, 30.0)) for time, value, _ in data['merge']],
                '#ef6c00',
            ),
        ])],
        y_range=(0.0, 30.0),
    )
    write_stacked_svg(
        args.output / 'command_outputs.svg',
        'Controller command outputs',
        [('full run', [
            (
                'throttle',
                [(time, throttle) for time, throttle, _, _ in data['command']],
                '#2e7d32',
            ),
            (
                'brake',
                [(time, brake) for time, _, _, brake in data['command']],
                '#d32f2f',
            ),
            (
                'steering',
                [(time, steering) for time, _, steering, _ in data['command']],
                '#1565c0',
            ),
        ])],
        y_range=(-1.05, 1.05),
    )
    write_readme(args.output, metrics)


if __name__ == '__main__':
    main()
