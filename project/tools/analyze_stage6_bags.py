#!/usr/bin/env python3
"""Generate Stage 6 metrics and dependency-free SVG plots from rosbag2 runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from statistics import mean

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


COLORS = ['#1565c0', '#d32f2f', '#2e7d32', '#6a1b9a', '#ef6c00']


def finite_min(values):
    finite = [value for value in values if math.isfinite(value)]
    return min(finite) if finite else None


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]


def downsample(points, limit=1800):
    if len(points) <= limit:
        return points
    step = max(1, len(points) // limit)
    return points[::step]


def read_run(bag_path: Path, logs_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()
    }

    data = {
        'pose': [], 'speed': [], 'planning_speed': [], 'decision_speed': [],
        'light': [], 'front': [], 'merge': [], 'command': [], 'tracking': [],
    }
    start_ns = None
    last_pose = None
    last_waypoint = None
    last_waypoint_distance = None
    last_stop = False
    stop_events = []
    route_complete_ns = None
    max_waypoint = -1
    decision_entries = {}
    previous_decision = None

    wanted = set(topic_types).intersection({
        '/OurCar/INS/pose', '/OurCar/INS/twist', '/route/current_waypoint_id',
        '/route/current_waypoint_distance', '/route/route_state', '/planning/path',
        '/planning/target_speed', '/perception/traffic_light_state',
        '/decision/target_speed', '/decision/stop_required', '/decision/state',
        '/decision/front_obstacle_distance', '/decision/merge_obstacle_distance',
        '/car_command',
    })

    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        if topic not in wanted:
            continue
        message = deserialize_message(serialized, topic_types[topic])
        if start_ns is None:
            start_ns = stamp_ns
        elapsed = (stamp_ns - start_ns) / 1e9

        if topic == '/OurCar/INS/pose':
            x = message.pose.position.x
            y = message.pose.position.y
            last_pose = (x, y)
            data['pose'].append((elapsed, x, y))
        elif topic == '/OurCar/INS/twist':
            linear = message.twist.linear
            speed = math.sqrt(linear.x ** 2 + linear.y ** 2 + linear.z ** 2)
            data['speed'].append((elapsed, speed))
        elif topic == '/route/current_waypoint_id':
            last_waypoint = int(message.data)
            max_waypoint = max(max_waypoint, last_waypoint)
        elif topic == '/route/current_waypoint_distance':
            last_waypoint_distance = float(message.data)
        elif topic == '/route/route_state':
            if message.data == 'route_complete' and route_complete_ns is None:
                route_complete_ns = stamp_ns
        elif topic == '/planning/path':
            if last_pose is not None and message.poses:
                error = min(
                    math.hypot(
                        pose.pose.position.x - last_pose[0],
                        pose.pose.position.y - last_pose[1],
                    )
                    for pose in message.poses
                )
                data['tracking'].append((elapsed, error))
        elif topic == '/planning/target_speed':
            data['planning_speed'].append((elapsed, float(message.data)))
        elif topic == '/decision/target_speed':
            data['decision_speed'].append((elapsed, float(message.data)))
        elif topic == '/perception/traffic_light_state':
            data['light'].append((elapsed, float(message.data)))
        elif topic == '/decision/front_obstacle_distance':
            value = float(message.data)
            data['front'].append((elapsed, value, last_waypoint))
        elif topic == '/decision/merge_obstacle_distance':
            value = float(message.data)
            data['merge'].append((elapsed, value, last_waypoint))
        elif topic == '/decision/state':
            state = message.data
            if state != previous_decision:
                decision_entries[state] = decision_entries.get(state, 0) + 1
                previous_decision = state
        elif topic == '/decision/stop_required':
            current = bool(message.data)
            if current and not last_stop:
                stop_events.append({
                    'waypoint': last_waypoint,
                    'distance': last_waypoint_distance,
                    'time_s': elapsed,
                })
            last_stop = current
        elif topic == '/car_command':
            data['command'].append((
                elapsed, float(message.throttle), float(message.steering), float(message.brake)
            ))

    duration = (
        (route_complete_ns - start_ns) / 1e9
        if route_complete_ns is not None and start_ns is not None
        else (data['pose'][-1][0] if data['pose'] else 0.0)
    )
    speed_values = [item[1] for item in data['speed']]
    tracking_values = [item[1] for item in data['tracking']]
    commands = data['command']
    front = [(value, waypoint) for _, value, waypoint in data['front']]
    merge = [(value, waypoint) for _, value, waypoint in data['merge']]

    controller_safe_stops = {}
    fallback_count = 0
    process_died_count = 0
    runtime_process_died_count = 0
    teardown_process_died_count = 0
    for log_path in logs_path.rglob('*.log'):
        text = log_path.read_text(errors='replace')
        for reason in re.findall(r'Safe stop: ([^\n\r]+)', text):
            controller_safe_stops[reason] = controller_safe_stops.get(reason, 0) + 1
        fallback_count += len(re.findall(r'local path rejected|fallback path rejected', text))
        died_lines = [line for line in text.splitlines() if 'process has died' in line]
        process_died_count += len(died_lines)
        route_complete_seconds = route_complete_ns / 1e9 if route_complete_ns is not None else math.inf
        for line in died_lines:
            match = re.match(r'([0-9]+(?:\.[0-9]+)?) ', line)
            if match and float(match.group(1)) <= route_complete_seconds:
                runtime_process_died_count += 1
            else:
                teardown_process_died_count += 1

    def max_gap(series):
        return max(
            (series[index][0] - series[index - 1][0] for index in range(1, len(series))),
            default=0.0,
        )

    metrics = {
        'route_complete': route_complete_ns is not None,
        'mission_duration_s': round(duration, 3),
        'max_waypoint': max_waypoint,
        'max_speed_mps': round(max(speed_values, default=0.0), 3),
        'mean_speed_mps': round(mean(speed_values), 3) if speed_values else 0.0,
        'tracking_rmse_m': round(
            math.sqrt(mean([value * value for value in tracking_values])), 4
        ) if tracking_values else 0.0,
        'tracking_p95_m': round(percentile(tracking_values, 0.95), 4),
        'tracking_max_m': round(max(tracking_values, default=0.0), 4),
        'max_twist_gap_s': round(max_gap(data['speed']), 4),
        'max_planning_path_gap_s': round(max_gap(data['tracking']), 4),
        'max_decision_speed_gap_s': round(max_gap(data['decision_speed']), 4),
        'event1_min_front_m': finite_min([
            value for value, waypoint in front if waypoint is not None and 17 <= waypoint <= 20
        ]),
        'event2_min_merge_m': finite_min([
            value for value, waypoint in merge if waypoint is not None and 45 <= waypoint <= 49
        ]),
        'event2_min_front_m': finite_min([
            value for value, waypoint in front if waypoint is not None and 45 <= waypoint <= 49
        ]),
        'emergency_entries': decision_entries.get('EMERGENCY_BRAKING', 0),
        'merge_slowdown_entries': decision_entries.get('SLOWING_FOR_MERGE', 0),
        'simultaneous_throttle_brake': sum(
            1 for _, throttle, _, brake in commands if throttle > 1e-6 and brake > 1e-6
        ),
        'throttle_saturation_samples': sum(1 for _, throttle, _, _ in commands if throttle >= 0.349),
        'brake_saturation_samples': sum(1 for _, _, _, brake in commands if brake >= 0.999),
        'path_rejection_or_fallback_count': fallback_count,
        'process_died_log_count': process_died_count,
        'runtime_process_died_count': runtime_process_died_count,
        'teardown_process_died_count': teardown_process_died_count,
        'safe_stop_reasons': controller_safe_stops,
        'stop_events': stop_events,
    }
    for key in ('event1_min_front_m', 'event2_min_merge_m', 'event2_min_front_m'):
        if metrics[key] is not None:
            metrics[key] = round(metrics[key], 3)
    return data, metrics


def polyline(points, x_scale, y_scale, color, width=1.4):
    sampled = downsample(points)
    coords = ' '.join(f'{x_scale(x):.1f},{y_scale(y):.1f}' for x, y in sampled)
    return (
        f'<polyline fill="none" stroke="{color}" stroke-width="{width}" '
        f'points="{coords}" />'
    )


def write_stacked_svg(path: Path, title: str, panels, y_range=None):
    width, panel_height, margin = 1100, 230, 55
    height = 55 + panel_height * len(panels)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="white" />',
        f'<text x="{width / 2}" y="28" text-anchor="middle" font-size="18">{title}</text>',
    ]
    for index, (label, series) in enumerate(panels):
        top = 45 + index * panel_height
        all_points = [point for _, points, _ in series for point in points]
        max_x = max((point[0] for point in all_points), default=1.0)
        if y_range is None:
            ys = [point[1] for point in all_points if math.isfinite(point[1])]
            low = min(ys, default=0.0)
            high = max(ys, default=1.0)
            pad = max(0.05, (high - low) * 0.08)
            low, high = low - pad, high + pad
        else:
            low, high = y_range
        if high <= low:
            high = low + 1.0
        x_scale = lambda x: margin + x / max(max_x, 1e-6) * (width - 2 * margin)
        y_scale = lambda y: top + panel_height - 35 - (y - low) / (high - low) * (panel_height - 65)
        parts.extend([
            f'<text x="12" y="{top + 18}" font-size="14">{label}</text>',
            f'<line x1="{margin}" y1="{top + panel_height - 35}" x2="{width - margin}" '
            f'y2="{top + panel_height - 35}" stroke="#444" />',
            f'<line x1="{margin}" y1="{top + 30}" x2="{margin}" '
            f'y2="{top + panel_height - 35}" stroke="#444" />',
            f'<text x="{margin}" y="{top + panel_height - 15}" font-size="11">0 s</text>',
            f'<text x="{width - margin}" y="{top + panel_height - 15}" text-anchor="end" '
            f'font-size="11">{max_x:.0f} s</text>',
            f'<text x="{margin - 6}" y="{top + 40}" text-anchor="end" font-size="11">{high:.2f}</text>',
            f'<text x="{margin - 6}" y="{top + panel_height - 35}" text-anchor="end" '
            f'font-size="11">{low:.2f}</text>',
        ])
        for series_index, (name, points, color) in enumerate(series):
            valid = [(x, y) for x, y in points if math.isfinite(y)]
            if valid:
                parts.append(polyline(valid, x_scale, y_scale, color))
            parts.append(
                f'<text x="{width - margin - series_index * 145}" y="{top + 18}" '
                f'text-anchor="end" fill="{color}" font-size="12">{name}</text>'
            )
    parts.append('</svg>')
    path.write_text('\n'.join(parts))


def write_route_svg(path: Path, runs):
    width, height, margin = 1000, 650, 55
    points = [(x, y) for data in runs for _, x, y in data['pose']]
    min_x, max_x = min(x for x, _ in points), max(x for x, _ in points)
    min_y, max_y = min(y for _, y in points), max(y for _, y in points)
    x_scale = lambda x: margin + (x - min_x) / (max_x - min_x) * (width - 2 * margin)
    y_scale = lambda y: height - margin - (y - min_y) / (max_y - min_y) * (height - 2 * margin)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="white" />',
        f'<text x="{width / 2}" y="28" text-anchor="middle" font-size="18">Stage 6 route traces</text>',
        f'<rect x="{margin}" y="{margin}" width="{width - 2 * margin}" '
        f'height="{height - 2 * margin}" fill="none" stroke="#444" />',
    ]
    for index, data in enumerate(runs):
        route = [(x, y) for _, x, y in data['pose']]
        parts.append(polyline(route, x_scale, y_scale, COLORS[index], 1.6))
        parts.append(
            f'<text x="{width - margin - index * 90}" y="45" text-anchor="end" '
            f'fill="{COLORS[index]}" font-size="13">run {index + 1}</text>'
        )
    parts.extend([
        f'<text x="{margin}" y="{height - 16}" font-size="11">x: {min_x:.1f} .. {max_x:.1f} m</text>',
        f'<text x="{width - margin}" y="{height - 16}" text-anchor="end" '
        f'font-size="11">y: {min_y:.1f} .. {max_y:.1f} m</text>',
        '</svg>',
    ])
    path.write_text('\n'.join(parts))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--evidence-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    runs = []
    metrics = []
    for index in range(1, 4):
        data, run_metrics = read_run(
            args.evidence_root / f'run{index}' / 'bag',
            args.evidence_root / f'run{index}' / 'ros_logs',
        )
        runs.append(data)
        metrics.append(run_metrics)

    (args.output / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    scalar_keys = [
        key for key, value in metrics[0].items()
        if not isinstance(value, (dict, list))
    ]
    with (args.output / 'summary.csv').open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(['run', *scalar_keys])
        for index, run_metrics in enumerate(metrics, start=1):
            writer.writerow([index, *(run_metrics[key] for key in scalar_keys)])

    write_route_svg(args.output / 'route_trace.svg', runs)
    write_stacked_svg(
        args.output / 'tracking_error.svg', 'Path tracking error',
        [(f'run {i + 1}', [('error (m)', run['tracking'], COLORS[0])]) for i, run in enumerate(runs)],
        y_range=(0.0, max(1.0, max(point[1] for run in runs for point in run['tracking']))),
    )
    write_stacked_svg(
        args.output / 'speed_profile.svg', 'Measured and commanded speed',
        [(f'run {i + 1}', [
            ('measured', run['speed'], COLORS[0]),
            ('decision target', run['decision_speed'], COLORS[1]),
        ]) for i, run in enumerate(runs)],
        y_range=(0.0, 10.5),
    )
    write_stacked_svg(
        args.output / 'traffic_light_state.svg', 'Traffic-light state (0 unknown, 1 red, 2 yellow, 3 green)',
        [(f'run {i + 1}', [('state', run['light'], COLORS[2])]) for i, run in enumerate(runs)],
        y_range=(-0.2, 3.2),
    )
    write_stacked_svg(
        args.output / 'obstacle_distance.svg', 'Finite obstacle distances',
        [(f'run {i + 1}', [
            ('front', [(t, min(value, 30.0)) for t, value, _ in run['front']], COLORS[1]),
            ('merge', [(t, min(value, 30.0)) for t, value, _ in run['merge']], COLORS[4]),
        ]) for i, run in enumerate(runs)],
        y_range=(0.0, 30.0),
    )
    write_stacked_svg(
        args.output / 'command_outputs.svg', 'Controller command outputs',
        [(f'run {i + 1}', [
            ('throttle', [(t, throttle) for t, throttle, _, _ in run['command']], COLORS[2]),
            ('brake', [(t, brake) for t, _, _, brake in run['command']], COLORS[1]),
            ('steering', [(t, steering) for t, _, steering, _ in run['command']], COLORS[0]),
        ]) for i, run in enumerate(runs)],
        y_range=(-1.05, 1.05),
    )


if __name__ == '__main__':
    main()
