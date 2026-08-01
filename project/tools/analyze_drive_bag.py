#!/usr/bin/env python3

import argparse
import collections
import math
import re

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: get_message(item.type)
        for item in reader.get_all_topics_and_types()
    }

    waypoint = None
    waypoint_distance = math.inf
    state_counts = collections.defaultdict(collections.Counter)
    score_ranges = collections.defaultdict(
        lambda: {'count': 0, 'red_min': math.inf, 'red_max': -math.inf,
                 'green_min': math.inf, 'green_max': -math.inf}
    )
    transitions = []
    traffic_debug = collections.defaultdict(collections.Counter)
    last_state = None
    start_ns = None

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if start_ns is None:
            start_ns = stamp_ns
        message = deserialize_message(data, topic_types[topic])
        if topic == '/route/current_waypoint_id':
            waypoint = message.data
        elif topic == '/route/current_waypoint_distance':
            waypoint_distance = message.data
        elif topic == '/perception/traffic_light_state':
            state = int(message.data)
            state_counts[waypoint][state] += 1
            if state != last_state:
                transitions.append(
                    ((stamp_ns - start_ns) / 1e9, waypoint,
                     waypoint_distance, last_state, state)
                )
                last_state = state
        elif topic == '/perception/traffic_light_scores':
            if len(message.data) >= 2:
                red, green = message.data[:2]
                summary = score_ranges[waypoint]
                summary['count'] += 1
                summary['red_min'] = min(summary['red_min'], red)
                summary['red_max'] = max(summary['red_max'], red)
                summary['green_min'] = min(summary['green_min'], green)
                summary['green_max'] = max(summary['green_max'], green)
        elif topic == '/perception/traffic_light_debug':
            values = dict(
                re.findall(
                    r'(candidate|fallback|raw|stable|red_score|yellow_score|'
                    r'green_score|bbox)=([^ ]+)',
                    message.data,
                )
            )
            key = (
                values.get('candidate'),
                values.get('fallback'),
                values.get('raw'),
                values.get('stable'),
                values.get('red_score'),
                values.get('yellow_score'),
                values.get('green_score'),
                values.get('bbox'),
            )
            traffic_debug[waypoint][key] += 1

    print('state counts (0=UNKNOWN 1=RED 2=YELLOW 3=GREEN)')
    for key in sorted(state_counts, key=lambda value: (-1 if value is None else value)):
        print(f'  waypoint={key}: {dict(state_counts[key])}')
    print('score ranges')
    for key in sorted(score_ranges, key=lambda value: (-1 if value is None else value)):
        print(f'  waypoint={key}: {score_ranges[key]}')
    print('state transitions')
    for elapsed, wp, distance, old, new in transitions:
        print(
            f'  t={elapsed:8.3f}s waypoint={wp} distance={distance:7.3f} '
            f'{old}->{new}'
        )
    print('most common traffic debug tuples')
    for key in sorted(
        traffic_debug, key=lambda value: (-1 if value is None else value)
    ):
        print(f'  waypoint={key}')
        for values, count in traffic_debug[key].most_common(20):
            print(f'    count={count:4d} values={values}')


if __name__ == '__main__':
    main()
