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
