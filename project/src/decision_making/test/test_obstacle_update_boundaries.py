import math
from types import SimpleNamespace

from decision_making.obstacle_fusion import VehicleObstacleFusion
from decision_making.obstacle_monitor_node import ObstacleMonitorNode
from decision_making.traffic_light_state_machine_node import (
    TrafficLightStateMachineNode,
)
import pytest


class RecordingMachine:
    def __init__(self):
        self.front_updates = []
        self.merge_updates = []

    def update_front_obstacle_distance(self, distance, now):
        self.front_updates.append((distance, now))

    def update_merge_obstacle_distance(self, distance, now):
        self.merge_updates.append((distance, now))


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return SimpleNamespace(
            nanoseconds=self.nanoseconds - other.nanoseconds
        )


def test_front_and_merge_callbacks_refresh_only_their_own_source():
    machine = RecordingMachine()
    adapter = SimpleNamespace(machine=machine, now_seconds=lambda: 12.5)

    TrafficLightStateMachineNode.front_obstacle_callback(
        adapter, SimpleNamespace(data=7.0)
    )
    TrafficLightStateMachineNode.merge_obstacle_callback(
        adapter, SimpleNamespace(data=5.0)
    )

    assert machine.front_updates == [(7.0, 12.5)]
    assert machine.merge_updates == [(5.0, 12.5)]


@pytest.mark.parametrize(
    ('publish_merge', 'expected_merge_messages'),
    [(False, 0), (True, 1)],
)
def test_front_only_publication_cannot_synthesize_a_fresh_merge_observation(
    publish_merge,
    expected_merge_messages,
):
    monitor = SimpleNamespace(
        front_pub=RecordingPublisher(),
        merge_pub=RecordingPublisher(),
    )
    fusion = VehicleObstacleFusion(
        front_distance=8.0,
        merge_distance=float('inf'),
        confirmed=True,
        front_matched=True,
        merge_matched=False,
    )

    ObstacleMonitorNode._publish_fusion(
        monitor,
        fusion,
        publish_merge=publish_merge,
    )

    assert len(monitor.front_pub.messages) == 1
    assert len(monitor.merge_pub.messages) == expected_merge_messages


def test_leaving_event1_direct_window_clears_vision_association():
    monitor = SimpleNamespace(
        waypoint_id=20,
        vision_direct_control_waypoint_ids={
            17, 18, 19, 20, 43, 44, 45, 46, 47, 48
        },
        fixed_structure_suppressed=True,
        vision_vehicle_detected=True,
        vision_vehicle_last_positive_time=object(),
        vision_vehicle_distance=7.0,
        vision_vehicle_distance_accepted_time=object(),
    )

    ObstacleMonitorNode.waypoint_callback(
        monitor,
        SimpleNamespace(data=21),
    )

    assert monitor.waypoint_id == 21
    assert not monitor.fixed_structure_suppressed
    assert not monitor.vision_vehicle_detected
    assert monitor.vision_vehicle_last_positive_time is None
    assert math.isinf(monitor.vision_vehicle_distance)
    assert monitor.vision_vehicle_distance_accepted_time is None


def test_event1_association_is_retained_inside_direct_window():
    positive_time = object()
    accepted_time = object()
    monitor = SimpleNamespace(
        waypoint_id=19,
        vision_direct_control_waypoint_ids={
            17, 18, 19, 20, 43, 44, 45, 46, 47, 48
        },
        fixed_structure_suppressed=False,
        vision_vehicle_detected=True,
        vision_vehicle_last_positive_time=positive_time,
        vision_vehicle_distance=17.0,
        vision_vehicle_distance_accepted_time=accepted_time,
    )

    ObstacleMonitorNode.waypoint_callback(
        monitor,
        SimpleNamespace(data=20),
    )

    assert monitor.waypoint_id == 20
    assert monitor.vision_vehicle_detected
    assert monitor.vision_vehicle_last_positive_time is positive_time
    assert monitor.vision_vehicle_distance == pytest.approx(17.0)
    assert monitor.vision_vehicle_distance_accepted_time is accepted_time


def test_finite_vision_range_confirms_detection_despite_topic_reordering():
    now = FakeTime(12.0)
    publications = []
    monitor = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        vision_vehicle_detected=False,
        vision_vehicle_last_positive_time=None,
        vision_vehicle_distance=math.inf,
        vision_vehicle_distance_accepted_time=None,
        vision_vehicle_distance_receive_time=None,
        vision_vehicle_detection_hold_time=1.3,
        vision_vehicle_max_abs_range_rate=6.0,
        vision_vehicle_jump_tolerance=1.5,
        publish_distances=lambda: publications.append(True),
    )
    monitor.vision_vehicle_detection_held = (
        lambda sample_time: ObstacleMonitorNode.vision_vehicle_detection_held(
            monitor, sample_time
        )
    )

    ObstacleMonitorNode.vision_vehicle_distance_callback(
        monitor,
        SimpleNamespace(data=18.0),
    )

    assert monitor.vision_vehicle_detected
    assert monitor.vision_vehicle_last_positive_time is now
    assert monitor.vision_vehicle_distance == pytest.approx(18.0)
    assert monitor.vision_vehicle_distance_accepted_time is now
    assert publications == [True]
