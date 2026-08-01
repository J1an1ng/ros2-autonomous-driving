import math

from decision_making.obstacle_fusion import (
    distances_match,
    fuse_vehicle_obstacle_distances,
    vision_detection_is_held,
    vision_distance_is_plausible,
)
import pytest


def fuse(front, merge, detected, vision, direct=True):
    return fuse_vehicle_obstacle_distances(
        front,
        merge,
        detected,
        vision,
        absolute_tolerance=2.5,
        relative_tolerance=0.2,
        unconfirmed_emergency_distance=0.8,
        vision_direct_control=direct,
    )


def test_baseline_pointcloud_returns_are_rejected_without_vision_vehicle():
    result = fuse(2.63, 10.97, False, math.inf)

    assert math.isinf(result.front_distance)
    assert math.isinf(result.merge_distance)
    assert not result.confirmed


def test_event2_vision_vehicle_is_not_vetoed_by_unrelated_near_return():
    result = fuse(3.02, 10.01, True, 19.34)

    assert result.front_distance == pytest.approx(19.34)
    assert result.merge_distance == pytest.approx(19.34)
    assert result.confirmed
    assert not result.front_matched
    assert not result.merge_matched


def test_event2_matching_merge_and_vision_ranges_enable_slowdown():
    result = fuse(20.34, 10.95, True, 10.86)

    assert result.front_distance == pytest.approx(10.86)
    assert result.merge_distance == pytest.approx(10.86)
    assert result.confirmed
    assert not result.front_matched
    assert result.merge_matched


def test_matching_front_range_fuses_vision_and_path_cloud_distance():
    result = fuse(12.83, 7.85, True, 12.28, direct=False)

    assert result.front_distance == pytest.approx((12.83 + 12.28) / 2.0)
    assert math.isinf(result.merge_distance)
    assert result.confirmed
    assert result.front_matched


def test_global_unmatched_vision_vehicle_is_rejected():
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
        vision_vehicle_detected=True,
        vision_vehicle_distance=12.0,
        absolute_tolerance=2.5,
        relative_tolerance=0.2,
        unconfirmed_emergency_distance=0.8,
        vision_direct_control=False,
        pointcloud_match_front_distance=11.5,
    )

    assert result.front_distance == pytest.approx((11.5 + 12.0) / 2.0)
    assert result.front_matched
    assert result.confirmed


def test_vision_vehicle_remains_active_when_pointcloud_loses_it_near_contact():
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


def test_vision_distance_filter_rejects_single_frame_range_jump():
    assert not vision_distance_is_plausible(23.35, 9.97, 1.0, 6.0, 1.5)
    assert vision_distance_is_plausible(11.75, 7.20, 1.0, 6.0, 1.5)
    assert vision_distance_is_plausible(math.inf, 19.0, math.inf, 6.0, 1.5)


def test_vision_detection_hold_bridges_only_short_dropouts():
    assert vision_detection_is_held(True, math.inf, 0.6)
    assert vision_detection_is_held(False, 0.4, 0.6)
    assert not vision_detection_is_held(False, 0.7, 0.6)
    assert not vision_detection_is_held(False, math.inf, 0.6)
