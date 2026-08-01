#!/usr/bin/env python3
"""Audit one full-system rosbag and generate reproducible mission evidence."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import date
import json
import math
from pathlib import Path

from mission_analysis import read_run, write_route_svg, write_stacked_svg

from rclpy.serialization import deserialize_message

import rosbag2_py

from rosidl_runtime_py.utilities import get_message

import yaml


CONSTRAINT_TOLERANCE = 0.05
CURVATURE_TOLERANCE = 1e-4
MAX_RUNTIME_EMPTY_TRAJECTORY_FRACTION = 0.05
MAX_CONTINUOUS_EMPTY_TRAJECTORY_S = 10.0
TRAJECTORY_MESSAGE_GAP_TOLERANCE_S = 0.05


@dataclass(frozen=True)
class AuditLimits:
    """Effective trajectory limits loaded from the production YAML."""

    max_curvature: float
    max_lateral_acceleration: float
    max_longitudinal_acceleration: float
    max_longitudinal_deceleration: float
    wheelbase: float
    maximum_steering_angle: float
    trajectory_timeout: float
    source: str
    controller_source: str


def load_audit_limits(config_path, controller_config_path):
    """Load the exact planner limits and derive the steering curvature cap."""
    with config_path.open() as stream:
        document = yaml.safe_load(stream)
    parameters = document['trajectory_generator_node']['ros__parameters']
    wheelbase = float(parameters['wheelbase'])
    maximum_steering_angle = float(parameters['maximum_steering_angle'])
    with controller_config_path.open() as stream:
        controller_document = yaml.safe_load(stream)
    controller_parameters = controller_document[
        'controller_node'
    ]['ros__parameters']
    steering_curvature = math.tan(maximum_steering_angle) / wheelbase
    return AuditLimits(
        max_curvature=min(
            float(parameters['max_curvature']),
            steering_curvature,
        ),
        max_lateral_acceleration=float(
            parameters['max_lateral_acceleration']
        ),
        max_longitudinal_acceleration=float(
            parameters['max_longitudinal_acceleration']
        ),
        max_longitudinal_deceleration=float(
            parameters['max_longitudinal_deceleration']
        ),
        wheelbase=wheelbase,
        maximum_steering_angle=maximum_steering_angle,
        trajectory_timeout=float(
            controller_parameters['trajectory_timeout']
        ),
        source=str(config_path),
        controller_source=str(controller_config_path),
    )


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


def analyze_trajectory_availability(
        records, route_complete_ns, recording_end_ns=None,
        maximum_message_gap_s=1.05, runtime_start_ns=None):
    """Audit nonempty trajectory availability before route completion."""
    runtime_records = [
        record for record in records
        if route_complete_ns is None or record[0] <= route_complete_ns
    ]
    empty_records = [record for record in runtime_records if record[1] == 0]
    nonempty_records = [record for record in runtime_records if record[1] > 0]
    maximum_empty_duration_s = 0.0
    empty_start_ns = None
    for stamp_ns, point_count in runtime_records:
        if point_count == 0:
            if empty_start_ns is None:
                empty_start_ns = stamp_ns
        elif empty_start_ns is not None:
            maximum_empty_duration_s = max(
                maximum_empty_duration_s,
                (stamp_ns - empty_start_ns) / 1e9,
            )
            empty_start_ns = None
    if empty_start_ns is not None:
        outage_end_ns = (
            route_complete_ns
            if route_complete_ns is not None
            else recording_end_ns
        )
        if outage_end_ns is None:
            outage_end_ns = runtime_records[-1][0]
        maximum_empty_duration_s = max(
            maximum_empty_duration_s,
            max(outage_end_ns - empty_start_ns, 0) / 1e9,
        )

    gap_records = [
        record for record in runtime_records
        if runtime_start_ns is None or record[0] >= runtime_start_ns
    ]
    message_gaps_s = []
    if runtime_start_ns is not None:
        first_gap_end_ns = (
            gap_records[0][0]
            if gap_records
            else route_complete_ns or recording_end_ns
        )
        if first_gap_end_ns is not None:
            message_gaps_s.append(
                max(first_gap_end_ns - runtime_start_ns, 0) / 1e9
            )
    message_gaps_s.extend([
        (gap_records[index][0] - gap_records[index - 1][0]) / 1e9
        for index in range(1, len(gap_records))
    ])
    if gap_records and route_complete_ns is not None:
        message_gaps_s.append(
            max(route_complete_ns - gap_records[-1][0], 0) / 1e9
        )
    maximum_message_gap = max(message_gaps_s, default=0.0)

    message_count = len(runtime_records)
    empty_fraction = (
        len(empty_records) / message_count if message_count else 1.0
    )
    point_count = sum(record[1] for record in nonempty_records)
    results = {
        'nonempty_trajectory_evidence': (
            len(nonempty_records) > 0 and point_count > 0
        ),
        'runtime_empty_trajectory_fraction': (
            empty_fraction <= MAX_RUNTIME_EMPTY_TRAJECTORY_FRACTION
        ),
        'continuous_runtime_availability': (
            maximum_empty_duration_s <= MAX_CONTINUOUS_EMPTY_TRAJECTORY_S
        ),
        'runtime_trajectory_message_gap': (
            maximum_message_gap <= maximum_message_gap_s
        ),
    }
    return {
        'runtime_trajectory_messages': message_count,
        'runtime_nonempty_trajectory_messages': len(nonempty_records),
        'runtime_empty_trajectory_messages': len(empty_records),
        'runtime_trajectory_points': point_count,
        'runtime_empty_trajectory_fraction': round(empty_fraction, 6),
        'maximum_continuous_empty_trajectory_s': round(
            maximum_empty_duration_s, 6
        ),
        'maximum_runtime_trajectory_message_gap_s': round(
            maximum_message_gap, 6
        ),
        'maximum_runtime_empty_trajectory_fraction': (
            MAX_RUNTIME_EMPTY_TRAJECTORY_FRACTION
        ),
        'maximum_allowed_continuous_empty_trajectory_s': (
            MAX_CONTINUOUS_EMPTY_TRAJECTORY_S
        ),
        'maximum_allowed_runtime_trajectory_message_gap_s': (
            maximum_message_gap_s
        ),
        'trajectory_availability_results': results,
    }


def require_route_and_trajectory_success(metrics):
    """Exit nonzero unless both the route and trajectory audits pass."""
    if not metrics['route_and_trajectory_audit_passed']:
        raise SystemExit('Route or trajectory audit failed')


def analyze_timing_and_trajectory(bag_path, limits):
    """Measure mission timing and audit every published trajectory profile."""
    reader, topic_types = open_reader(bag_path)
    first_recorded_ns = None
    first_forward_command_ns = None
    first_vehicle_motion_ns = None
    route_complete_ns = None
    trajectory_messages = 0
    trajectory_points = 0
    max_points_per_trajectory = 0
    max_abs_curvature = 0.0
    max_lateral_acceleration = 0.0
    max_longitudinal_acceleration = 0.0
    max_longitudinal_deceleration = 0.0
    non_monotonic_time_pairs = 0
    invalid_numeric_points = 0
    trajectory_records = []
    recording_end_ns = None
    wanted_topics = {
        '/OurCar/INS/twist',
        '/car_command',
        '/planning/trajectory',
        '/route/route_state',
    }

    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        recording_end_ns = stamp_ns
        if first_recorded_ns is None:
            first_recorded_ns = stamp_ns
        if topic not in wanted_topics or topic not in topic_types:
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
            trajectory_records.append((stamp_ns, len(message.points)))
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
                max_abs_curvature = max(max_abs_curvature, abs(curvature))
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

    availability = analyze_trajectory_availability(
        trajectory_records,
        route_complete_ns,
        recording_end_ns,
        limits.trajectory_timeout + TRAJECTORY_MESSAGE_GAP_TOLERANCE_S,
        first_forward_command_ns,
    )
    constraint_results = {
        'kinematic_curvature': (
            max_abs_curvature
            <= limits.max_curvature + CURVATURE_TOLERANCE
        ),
        'lateral_acceleration': (
            max_lateral_acceleration
            <= limits.max_lateral_acceleration + CONSTRAINT_TOLERANCE
        ),
        'longitudinal_acceleration': (
            max_longitudinal_acceleration
            <= limits.max_longitudinal_acceleration + CONSTRAINT_TOLERANCE
        ),
        'longitudinal_deceleration': (
            max_longitudinal_deceleration
            <= limits.max_longitudinal_deceleration + CONSTRAINT_TOLERANCE
        ),
        'monotonic_time': non_monotonic_time_pairs == 0,
        'finite_values': invalid_numeric_points == 0,
        **availability['trajectory_availability_results'],
    }
    max_abs_curvature = round(max_abs_curvature, 6)
    max_lateral_acceleration = round(max_lateral_acceleration, 4)
    max_longitudinal_acceleration = round(
        max_longitudinal_acceleration,
        4,
    )
    max_longitudinal_deceleration = round(
        max_longitudinal_deceleration,
        4,
    )
    limits_passed = all(constraint_results.values())
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
        'max_abs_curvature_per_m': max_abs_curvature,
        'max_lateral_acceleration_mps2': max_lateral_acceleration,
        'max_longitudinal_acceleration_mps2': (
            max_longitudinal_acceleration
        ),
        'max_longitudinal_deceleration_mps2': (
            max_longitudinal_deceleration
        ),
        'non_monotonic_time_pairs': non_monotonic_time_pairs,
        'invalid_numeric_points': invalid_numeric_points,
        **availability,
        'configured_limits_mps2': {
            'lateral': limits.max_lateral_acceleration,
            'longitudinal_acceleration': (
                limits.max_longitudinal_acceleration
            ),
            'longitudinal_deceleration': (
                limits.max_longitudinal_deceleration
            ),
        },
        'configured_max_curvature_per_m': limits.max_curvature,
        'configured_wheelbase_m': limits.wheelbase,
        'configured_maximum_steering_angle_rad': (
            limits.maximum_steering_angle
        ),
        'configured_controller_trajectory_timeout_s': (
            limits.trajectory_timeout
        ),
        'configuration_source': limits.source,
        'controller_configuration_source': limits.controller_source,
        'trajectory_message_gap_tolerance_s': (
            TRAJECTORY_MESSAGE_GAP_TOLERANCE_S
        ),
        'constraint_tolerance_mps2': CONSTRAINT_TOLERANCE,
        'curvature_tolerance_per_m': CURVATURE_TOLERANCE,
        'trajectory_constraint_results': constraint_results,
        'trajectory_constraint_audit_passed': limits_passed,
    }


def write_readme(output, metrics, validation_date):
    """Write a concise, human-readable record next to the raw metrics."""
    configured = metrics['configured_limits_mps2']
    results = metrics['trajectory_constraint_results']
    stop_waypoints = sorted({
        event['waypoint'] for event in metrics['stop_events']
    })
    stop_waypoint_text = (
        ', '.join(str(waypoint) for waypoint in stop_waypoints)
        if stop_waypoints
        else 'none'
    )
    completion_text = (
        'The vehicle reached waypoint '
        f"{metrics['max_waypoint']} and published "
        '`route_complete`.'
        if metrics['route_complete']
        else (
            f"The highest observed waypoint was {metrics['max_waypoint']}; "
            '`route_complete` was not published.'
        )
    )
    shutdown_text = (
        'The standard\nrunner initiated teardown only after route completion. '
        'Its log audit classified\n'
        f"{metrics['teardown_process_died_count']} process-death entries as "
        'teardown-only and '
        f"{metrics['runtime_process_died_count']} as occurring before "
        'completion.'
        if metrics['route_complete']
        else (
            'This run did not complete; process-death and shutdown counts '
            'must '
            'be interpreted as failed/aborted-run evidence.'
        )
    )
    post_completion_text = (
        'No target-pose fallback was recorded after route completion.'
        if metrics['post_route_complete_path_fallback_count'] == 0
        else (
            'Post-completion target-pose fallbacks occurred only after the '
            'decision and controller layers had latched the route-complete '
            'safe stop.'
        )
    )

    def format_seconds(value):
        return 'unavailable' if value is None else f'{value:.3f} s'

    def status(key):
        return 'Pass' if results[key] else '**Fail**'

    allowed_empty_duration = metrics[
        'maximum_allowed_continuous_empty_trajectory_s'
    ]
    allowed_message_gap = metrics[
        'maximum_allowed_runtime_trajectory_message_gap_s'
    ]
    constraint_table = (
        '| Bicycle-model curvature | '
        f"{metrics['configured_max_curvature_per_m']:.6f} 1/m | "
        f"{metrics['max_abs_curvature_per_m']:.6f} 1/m | "
        f"{status('kinematic_curvature')} |\n"
        '| Lateral acceleration `v^2 * abs(curvature)` | '
        f"{configured['lateral']:.2f} m/s² | "
        f"{metrics['max_lateral_acceleration_mps2']:.4f} m/s² | "
        f"{status('lateral_acceleration')} |\n"
        '| Longitudinal acceleration | '
        f"{configured['longitudinal_acceleration']:.2f} m/s² | "
        f"{metrics['max_longitudinal_acceleration_mps2']:.4f} m/s² | "
        f"{status('longitudinal_acceleration')} |\n"
        '| Longitudinal deceleration | '
        f"{configured['longitudinal_deceleration']:.2f} m/s² | "
        f"{metrics['max_longitudinal_deceleration_mps2']:.4f} m/s² | "
        f"{status('longitudinal_deceleration')} |\n"
        '| Strictly increasing point time | required | '
        f"{metrics['non_monotonic_time_pairs']} violations | "
        f"{status('monotonic_time')} |\n"
        '| Finite numeric trajectory values | required | '
        f"{metrics['invalid_numeric_points']} invalid points | "
        f"{status('finite_values')} |\n"
        '| Nonempty runtime trajectory evidence | required | '
        f"{metrics['runtime_nonempty_trajectory_messages']} messages | "
        f"{status('nonempty_trajectory_evidence')} |\n"
        '| Empty runtime trajectory fraction | '
        f"≤ {metrics['maximum_runtime_empty_trajectory_fraction']:.1%} | "
        f"{metrics['runtime_empty_trajectory_fraction']:.2%} | "
        f"{status('runtime_empty_trajectory_fraction')} |\n"
        '| Longest continuous empty-trajectory interval | '
        f'≤ {allowed_empty_duration:.1f} s | '
        f"{metrics['maximum_continuous_empty_trajectory_s']:.3f} s | "
        f"{status('continuous_runtime_availability')} |\n"
        '| Maximum trajectory-message gap | '
        f'≤ {allowed_message_gap:.2f} s | '
        f"{metrics['maximum_runtime_trajectory_message_gap_s']:.3f} s | "
        f"{status('runtime_trajectory_message_gap')} |"
    )
    text = f"""# Full-System Trajectory Validation

Validation date: {validation_date}

The production stack was launched from waypoint 0 with motion enabled and the
Unity semantic camera disabled. {completion_text}

## Mission result

- Mission time: **{format_seconds(metrics['mission_time_s'])}**.
- Timing definition: {metrics['mission_time_definition']}.
- Cross-check from first measured vehicle motion (> 0.1 m/s):
  **{format_seconds(metrics['first_motion_to_route_complete_s'])}**.
- The command and twist topics are recorded independently, so the first
  measured motion sample may precede the first recorded forward-command sample
  by one cycle.
- Maximum measured speed: {metrics['max_speed_mps']:.3f} m/s.
- Path-tracking RMSE / P95: {metrics['tracking_rmse_m']:.4f} m /
  {metrics['tracking_p95_m']:.4f} m.

## Trajectory constraint audit

The audit checked {metrics['trajectory_points']} points in
{metrics['trajectory_messages']} `/planning/trajectory` messages.
Effective limits were loaded from `{metrics['configuration_source']}`. The
kinematic cap is `min(max_curvature, tan(maximum_steering_angle) / wheelbase)`.

| Constraint | Configured limit | Maximum observed | Result |
| --- | ---: | ---: | --- |
{constraint_table}

The spatial acceleration audit uses adjacent published poses and speeds. A
0.05 m/s² acceleration tolerance and 0.0001 1/m curvature tolerance cover
float32 serialization and the difference between curved arc length and
pose-to-pose chord length. The availability gate allows at most 5% empty
runtime publications and no continuous empty interval longer than 10 s. An
empty publication makes the controller brake, so these bounds allow short
fail-safe recovery while rejecting persistent planner loss. Topic silence is
checked separately against the controller's configured trajectory timeout plus
a 0.05 s timestamp tolerance, from the first forward command through route
completion. Overall trajectory audit:
**{'Pass' if metrics['trajectory_constraint_audit_passed'] else 'Fail'}**.

## Scenario evidence

- Route completion: {metrics['route_complete']}.
- Adaptive-following entries: {metrics['following_entries']}.
- Emergency-braking entries: {metrics['emergency_entries']}.
- Merge-slowdown entries: {metrics['merge_slowdown_entries']}.
- Traffic-light messages: {metrics['traffic_light_messages']} as `std_msgs/msg/UInt8`;
  state counts: {metrics['traffic_light_state_counts']}.
- Simultaneous nonzero throttle and brake samples:
  {metrics['simultaneous_throttle_brake']}.
- Runtime process deaths before route completion:
  {metrics['runtime_process_died_count']}.
- Runtime path rejection/fallbacks before route completion:
  {metrics['runtime_path_rejection_or_fallback_count']}.
- Runtime fallback reasons:
  {metrics['runtime_path_rejection_or_fallback_reasons']}.
- Expected terminal-point fallbacks after route completion:
  {metrics['post_route_complete_path_fallback_count']}.
- Stop requests were recorded at route waypoints
  {stop_waypoint_text}.

Every runtime rejection in this run was caused by the bicycle-model curvature
gate. The planner then generated its straight target-pose fallback and applied
the same curvature, acceleration, time, and finite-value validation before
publication; rejected fallbacks produce an empty trajectory and controller safe
stop. Thus the count records conservative degradation, not unchecked output.

{post_completion_text} {shutdown_text}

## Generated figures

- `route_trace.svg`: driven route.
- `speed_profile.svg`: measured, planning, and final decision speeds.
- `tracking_error.svg`: nearest-reference path tracking error.
- `traffic_light_state.svg`: traffic-light observations.
- `obstacle_distance.svg`: front and merge obstacle distances.
- `command_outputs.svg`: throttle, brake, and steering commands.

`metrics.json` is the machine-readable result. The standard mission runner also
writes `validation_provenance.json` with the launch command, Git state,
renderer, and raw evidence location, plus `ros_logs.sha256` for the run logs.
For standard live runs, raw bags and logs stay in the Git-ignored
`validation_evidence/` directory. Every successful full-mission run replaces
this directory atomically, so all plots and metrics always come from the same
bag.
Regenerate derived artifacts from retained evidence with:

```bash
source install/setup.bash
python3 tools/analyze_mission_run.py \\
  --bag /path/to/bag \\
  --logs /path/to/ros_logs \\
  --trajectory-config \\
    src/trajectory_generator/config/trajectory_generator.yaml \\
  --controller-config src/controller/config/controller.yaml \\
  --output mission_results
```
"""
    (output / 'README.md').write_text(text)


def main():
    """Parse arguments and generate metrics plus dependency-free SVG plots."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--logs', type=Path, required=True)
    parser.add_argument(
        '--trajectory-config',
        type=Path,
        default=Path(
            'src/trajectory_generator/config/trajectory_generator.yaml'
        ),
    )
    parser.add_argument(
        '--controller-config',
        type=Path,
        default=Path('src/controller/config/controller.yaml'),
    )
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument(
        '--validation-date',
        default=date.today().isoformat(),
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    data, metrics = read_run(args.bag, args.logs)
    limits = load_audit_limits(
        args.trajectory_config, args.controller_config
    )
    metrics.update(analyze_timing_and_trajectory(args.bag, limits))
    metrics['route_and_trajectory_audit_passed'] = (
        metrics['route_complete']
        and metrics['trajectory_constraint_audit_passed']
    )
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
    write_readme(args.output, metrics, args.validation_date)
    require_route_and_trajectory_success(metrics)


if __name__ == '__main__':
    main()
