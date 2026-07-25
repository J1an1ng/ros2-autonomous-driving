import math

import pytest

from decision_making.obstacle_fusion import (
    distances_match,
    fuse_vehicle_obstacle_distances,
    semantic_distance_is_plausible,
)


def fuse(front, merge, detected, semantic, direct=True):
    return fuse_vehicle_obstacle_distances(
        front,
        merge,
        detected,
        semantic,
        absolute_tolerance=2.5,
        relative_tolerance=0.2,
        unconfirmed_emergency_distance=0.8,
        semantic_direct_control=direct,
    )


def test_baseline_pointcloud_returns_are_rejected_without_semantic_vehicle():
    result = fuse(2.63, 10.97, False, math.inf)

    assert math.isinf(result.front_distance)
    assert math.isinf(result.merge_distance)
    assert not result.confirmed


def test_event2_semantic_vehicle_is_not_vetoed_by_unrelated_near_return():
    result = fuse(3.02, 10.01, True, 19.34)

    assert result.front_distance == pytest.approx(19.34)
    assert result.merge_distance == pytest.approx(19.34)
    assert result.confirmed
    assert not result.front_matched
    assert not result.merge_matched


def test_event2_matching_merge_and_semantic_ranges_enable_slowdown():
    result = fuse(20.34, 10.95, True, 10.86)

    assert result.front_distance == pytest.approx(10.86)
    assert result.merge_distance == pytest.approx(10.86)
    assert result.confirmed
    assert not result.front_matched
    assert result.merge_matched


def test_matching_front_range_uses_semantic_vehicle_distance():
    result = fuse(12.83, 7.85, True, 12.28, direct=False)

    assert result.front_distance == pytest.approx(12.28)
    assert math.isinf(result.merge_distance)
    assert result.confirmed
    assert result.front_matched


def test_global_unmatched_semantic_vehicle_is_rejected():
    result = fuse(3.02, 10.01, True, 19.34, direct=False)

    assert math.isinf(result.front_distance)
    assert math.isinf(result.merge_distance)
    assert not result.confirmed
    assert not result.front_matched
    assert not result.merge_matched


def test_global_matching_merge_range_can_confirm_vehicle():
    result = fuse(20.34, 10.95, True, 10.86, direct=False)

    assert math.isinf(result.front_distance)
    assert result.merge_distance == pytest.approx(10.86)
    assert result.confirmed
    assert not result.front_matched
    assert result.merge_matched


def test_global_front_match_uses_path_corridor_not_near_safety_return():
    result = fuse_vehicle_obstacle_distances(
        pointcloud_front_distance=0.7,
        pointcloud_merge_distance=math.inf,
        semantic_vehicle_detected=True,
        semantic_vehicle_distance=12.0,
        absolute_tolerance=2.5,
        relative_tolerance=0.2,
        unconfirmed_emergency_distance=0.8,
        semantic_direct_control=False,
        pointcloud_match_front_distance=11.5,
    )

    assert result.front_distance == pytest.approx(12.0)
    assert result.front_matched
    assert result.confirmed


def test_semantic_vehicle_remains_active_when_pointcloud_loses_it_near_contact():
    result = fuse(math.inf, 9.97, True, 2.90)

    assert result.front_distance == pytest.approx(2.90)
    assert result.merge_distance == pytest.approx(2.90)
    assert result.confirmed
    assert not result.front_matched
    assert not result.merge_matched


def test_submetre_unconfirmed_front_return_remains_last_resort_guard():
    result = fuse(0.7, math.inf, False, math.inf)

    assert result.front_distance == pytest.approx(0.7)
    assert not result.confirmed


def test_distance_match_uses_relative_tolerance_at_long_range():
    assert distances_match(16.0, 19.0, 2.5, 0.2)
    assert not distances_match(12.0, 19.0, 2.5, 0.2)


def test_semantic_distance_filter_rejects_single_frame_range_jump():
    assert not semantic_distance_is_plausible(23.35, 9.97, 1.0, 6.0, 1.5)
    assert semantic_distance_is_plausible(11.75, 7.20, 1.0, 6.0, 1.5)
    assert semantic_distance_is_plausible(math.inf, 19.0, math.inf, 6.0, 1.5)
