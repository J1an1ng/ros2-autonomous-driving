from builtin_interfaces.msg import Time
import pytest

from trajectory_generator.path_processor import PathPoint
from trajectory_generator.trajectory_generator_node import (
    TrajectoryGeneratorNode,
)
from trajectory_generator.velocity_profile import TrajectorySample


def sample(x, speed, time_from_start):
    return TrajectorySample(
        path_point=PathPoint(
            x=x,
            y=1.0,
            z=0.5,
            yaw=0.0,
            s=x,
            curvature=0.25,
        ),
        target_speed=speed,
        time_from_start=time_from_start,
    )


def message_converter():
    converter = object.__new__(TrajectoryGeneratorNode)
    converter.map_frame = 'world'
    return converter


def test_serializes_complete_trajectory_contract_and_duration_rollover():
    stamp = Time(sec=12, nanosec=34)
    message = message_converter().trajectory_samples_to_msg(
        [sample(0.0, 1.0, 0.0), sample(1.0, 2.0, 1.9999999996)],
        stamp,
    )

    assert message.header.stamp == stamp
    assert message.header.frame_id == 'world'
    assert len(message.points) == 2
    assert message.points[1].pose.position.x == pytest.approx(1.0)
    assert message.points[1].pose.position.y == pytest.approx(1.0)
    assert message.points[1].pose.position.z == pytest.approx(0.5)
    assert message.points[1].pose.orientation.w == pytest.approx(1.0)
    assert message.points[1].curvature == pytest.approx(0.25)
    assert message.points[1].target_speed == pytest.approx(2.0)
    assert message.points[0].time_from_start.sec == 0
    assert message.points[0].time_from_start.nanosec == 0
    assert message.points[1].time_from_start.sec == 2
    assert message.points[1].time_from_start.nanosec == 0


def test_serializes_empty_trajectory_with_header():
    stamp = Time(sec=12, nanosec=34)
    message = message_converter().trajectory_samples_to_msg([], stamp)

    assert message.header.stamp == stamp
    assert message.header.frame_id == 'world'
    assert message.points == []
