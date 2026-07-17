import pytest

from decision_making.traffic_light_state_machine import (
    DecisionState,
    TrafficLight,
    TrafficLightStateMachine,
)


def make_machine(**overrides):
    parameters = dict(
        controlled_route_states=["intersection_approach", "stop_zone", "intersection"],
        stopped_speed_threshold=0.15,
        traffic_light_timeout=0.75,
        planning_speed_timeout=0.5,
        vehicle_speed_timeout=0.5,
        green_release_delay=0.3,
        treat_yellow_as_red=True,
        require_green_in_controlled_zone=True,
    )
    parameters.update(overrides)
    return TrafficLightStateMachine(**parameters)


def test_cruise_passes_through_fresh_planning_speed():
    machine = make_machine()
    machine.update_route("normal", 8)
    machine.update_planning_speed(4.0, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.CRUISE
    assert output.target_speed == pytest.approx(4.0)
    assert not output.stop_required


def test_red_light_latches_stop_after_route_manager_advances():
    machine = make_machine()
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(2.0, 0.0)
    machine.update_vehicle_speed(3.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)

    stopping = machine.evaluate(0.1)
    machine.update_route("curve_entry", 26)
    machine.update_planning_speed(8.0, 0.2)
    still_stopping = machine.evaluate(0.2)

    assert stopping.state == DecisionState.STOPPING_FOR_LIGHT
    assert still_stopping.state == DecisionState.STOPPING_FOR_LIGHT
    assert still_stopping.target_speed == 0.0
    assert still_stopping.stop_required


def test_only_fresh_stopped_speed_enters_waiting_state():
    machine = make_machine(vehicle_speed_timeout=0.2)
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(2.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.1)

    stale_speed = machine.evaluate(0.3)
    machine.update_vehicle_speed(0.1, 0.31)
    fresh_speed = machine.evaluate(0.31)

    assert stale_speed.state == DecisionState.STOPPING_FOR_LIGHT
    assert fresh_speed.state == DecisionState.WAITING_FOR_GREEN


def test_waiting_releases_only_after_continuous_fresh_green():
    machine = make_machine(green_release_delay=0.3)
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(2.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.05)
    assert machine.evaluate(0.06).state == DecisionState.WAITING_FOR_GREEN

    machine.update_traffic_light(TrafficLight.GREEN, 0.1)
    assert machine.evaluate(0.1).state == DecisionState.WAITING_FOR_GREEN
    machine.update_traffic_light(TrafficLight.GREEN, 0.39)
    assert machine.evaluate(0.39).state == DecisionState.WAITING_FOR_GREEN
    machine.update_traffic_light(TrafficLight.GREEN, 0.41)
    released = machine.evaluate(0.41)

    assert released.state == DecisionState.RELEASED
    assert not released.stop_required


def test_release_is_retained_until_vehicle_leaves_controlled_zone():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], green_release_delay=0.1
    )
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.01)
    machine.evaluate(0.02)
    machine.update_traffic_light(TrafficLight.GREEN, 0.1)
    machine.evaluate(0.1)
    machine.update_traffic_light(TrafficLight.GREEN, 0.21)
    assert machine.evaluate(0.21).state == DecisionState.RELEASED

    machine.update_planning_speed(3.0, 0.3)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.3)
    lost_from_view = machine.evaluate(0.3)
    machine.update_planning_speed(3.0, 0.4)
    machine.update_traffic_light(TrafficLight.RED, 0.4)
    changed_after_commit = machine.evaluate(0.4)

    assert lost_from_view.state == DecisionState.RELEASED
    assert lost_from_view.target_speed == pytest.approx(3.0)
    assert changed_after_commit.state == DecisionState.RELEASED
    assert changed_after_commit.target_speed == pytest.approx(3.0)

    machine.update_route("curve_entry", 26)
    machine.update_planning_speed(8.0, 0.5)
    left_zone = machine.evaluate(0.5)

    assert left_zone.state == DecisionState.CRUISE
    assert left_zone.target_speed == pytest.approx(8.0)


def test_unknown_or_stale_light_never_releases_waiting_stop():
    machine = make_machine(traffic_light_timeout=0.2, green_release_delay=0.1)
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(2.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.01)
    machine.evaluate(0.02)

    machine.update_traffic_light(TrafficLight.GREEN, 0.1)
    assert machine.evaluate(0.1).state == DecisionState.WAITING_FOR_GREEN
    assert machine.evaluate(0.31).state == DecisionState.WAITING_FOR_GREEN
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.32)
    assert machine.evaluate(0.5).state == DecisionState.WAITING_FOR_GREEN


def test_yellow_is_treated_as_stop_by_default():
    machine = make_machine()
    machine.update_route("intersection_approach", 9)
    machine.update_planning_speed(2.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.YELLOW, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.STOPPING_FOR_LIGHT
    assert output.target_speed == 0.0


def test_waypoint_allowlist_rejects_unlisted_stop_zone():
    machine = make_machine(traffic_light_waypoint_ids=[25, 32, 52, 59])
    machine.update_route("stop_zone", 0)
    machine.update_planning_speed(2.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.CRUISE
    assert output.target_speed == pytest.approx(2.0)


def test_unknown_light_stops_at_allowlisted_controlled_waypoint():
    machine = make_machine(traffic_light_waypoint_ids=[25, 32, 52, 59])
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.STOPPING_FOR_LIGHT
    assert output.target_speed == 0.0
    assert output.stop_required


def test_unknown_light_creeps_outside_hard_stop_boundary():
    machine = make_machine(
        traffic_light_waypoint_ids=[25],
        unknown_light_stop_distance=6.0,
        unknown_light_approach_speed=1.0,
    )
    machine.update_route("stop_zone", 25)
    machine.update_waypoint_distance(12.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.APPROACHING_INTERSECTION
    assert output.target_speed == pytest.approx(1.0)
    assert not output.stop_required
    assert output.reason == "unknown_light_controlled_approach"


def test_unknown_light_stops_at_hard_stop_boundary():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], unknown_light_stop_distance=6.0
    )
    machine.update_route("stop_zone", 25)
    machine.update_waypoint_distance(6.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(1.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.STOPPING_FOR_LIGHT
    assert output.target_speed == 0.0
    assert output.stop_required


def test_stale_waypoint_distance_cannot_authorize_unknown_creep():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], waypoint_distance_timeout=0.2
    )
    machine.update_route("stop_zone", 25)
    machine.update_waypoint_distance(12.0, 0.0)
    machine.update_planning_speed(3.0, 0.3)
    machine.update_vehicle_speed(1.0, 0.3)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.3)

    output = machine.evaluate(0.3)

    assert output.state == DecisionState.STOPPING_FOR_LIGHT
    assert output.target_speed == 0.0


def test_red_light_is_remembered_while_creeping_to_signal_stop_boundary():
    machine = make_machine(
        traffic_light_waypoint_ids=[25],
        traffic_light_stop_distance=4.0,
        unknown_light_approach_speed=1.0,
    )
    machine.update_route("stop_zone", 25)
    machine.update_waypoint_distance(20.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)

    approaching = machine.evaluate(0.1)
    machine.update_waypoint_distance(10.0, 0.2)
    machine.update_planning_speed(3.0, 0.2)
    machine.update_vehicle_speed(1.0, 0.2)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.2)
    after_light_loss = machine.evaluate(0.2)

    assert approaching.state == DecisionState.APPROACHING_INTERSECTION
    assert approaching.target_speed == pytest.approx(1.0)
    assert approaching.reason == "traffic_light_controlled_approach"
    assert after_light_loss.state == DecisionState.APPROACHING_INTERSECTION
    assert after_light_loss.target_speed == pytest.approx(1.0)


def test_far_red_is_treated_as_unassociated_until_activation_distance():
    machine = make_machine(
        traffic_light_waypoint_ids=[32],
        traffic_light_activation_distance=20.0,
        unknown_light_approach_speed=1.0,
    )
    machine.update_route("stop_zone", 32)
    machine.update_waypoint_distance(45.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)

    far_output = machine.evaluate(0.1)

    assert far_output.state == DecisionState.APPROACHING_INTERSECTION
    assert far_output.target_speed == pytest.approx(1.0)
    assert not far_output.stop_required

    machine.update_waypoint_distance(4.0, 0.2)
    machine.update_planning_speed(3.0, 0.2)
    machine.update_vehicle_speed(1.0, 0.2)
    machine.update_traffic_light(TrafficLight.RED, 0.2)
    near_output = machine.evaluate(0.2)

    assert near_output.state == DecisionState.STOPPING_FOR_LIGHT
    assert near_output.target_speed == 0.0
    assert near_output.stop_required


def test_far_green_cannot_grant_passage_authorization():
    machine = make_machine(
        traffic_light_waypoint_ids=[32],
        traffic_light_activation_distance=20.0,
        green_release_delay=0.1,
    )
    machine.update_route("stop_zone", 32)
    machine.update_waypoint_distance(45.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.GREEN, 0.0)
    machine.evaluate(0.0)
    machine.update_waypoint_distance(40.0, 0.2)
    machine.update_planning_speed(3.0, 0.2)
    machine.update_traffic_light(TrafficLight.GREEN, 0.2)

    output = machine.evaluate(0.2)

    assert output.state == DecisionState.APPROACHING_INTERSECTION
    assert output.target_speed == pytest.approx(1.0)


def test_unknown_boundary_stop_cancels_for_green_before_standstill():
    machine = make_machine(traffic_light_waypoint_ids=[25])
    machine.update_route("stop_zone", 25)
    machine.update_waypoint_distance(5.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(1.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)
    assert machine.evaluate(0.1).state == DecisionState.STOPPING_FOR_LIGHT

    machine.update_waypoint_distance(4.8, 0.2)
    machine.update_planning_speed(3.0, 0.2)
    machine.update_vehicle_speed(0.8, 0.2)
    machine.update_traffic_light(TrafficLight.GREEN, 0.2)
    released = machine.evaluate(0.2)

    assert released.state == DecisionState.RELEASED
    assert released.target_speed == pytest.approx(3.0)
    assert not released.stop_required


def test_unknown_boundary_stop_cancels_if_fresh_distance_is_outside_boundary():
    machine = make_machine(traffic_light_waypoint_ids=[25])
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(1.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)
    assert machine.evaluate(0.1).state == DecisionState.STOPPING_FOR_LIGHT

    machine.update_waypoint_distance(12.0, 0.2)
    machine.update_planning_speed(3.0, 0.2)
    resumed = machine.evaluate(0.2)

    assert resumed.state == DecisionState.APPROACHING_INTERSECTION
    assert resumed.target_speed == pytest.approx(1.0)
    assert not resumed.stop_required


def test_explicit_red_stop_cannot_cancel_for_green_before_standstill():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], traffic_light_stop_distance=4.0
    )
    machine.update_route("stop_zone", 25)
    machine.update_waypoint_distance(4.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(1.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    assert machine.evaluate(0.1).state == DecisionState.STOPPING_FOR_LIGHT

    machine.update_waypoint_distance(3.8, 0.2)
    machine.update_planning_speed(3.0, 0.2)
    machine.update_vehicle_speed(0.8, 0.2)
    machine.update_traffic_light(TrafficLight.GREEN, 0.2)
    still_stopping = machine.evaluate(0.2)

    assert still_stopping.state == DecisionState.STOPPING_FOR_LIGHT
    assert still_stopping.target_speed == 0.0
    assert still_stopping.stop_required


def test_stale_light_stops_at_allowlisted_controlled_waypoint():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], traffic_light_timeout=0.2
    )
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(3.0, 0.3)
    machine.update_vehicle_speed(2.0, 0.3)
    machine.update_traffic_light(TrafficLight.GREEN, 0.0)

    output = machine.evaluate(0.3)

    assert output.state == DecisionState.STOPPING_FOR_LIGHT
    assert output.target_speed == 0.0


def test_fresh_green_passes_at_allowlisted_controlled_waypoint():
    machine = make_machine(traffic_light_waypoint_ids=[25])
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.GREEN, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.APPROACHING_INTERSECTION
    assert output.target_speed == pytest.approx(3.0)
    assert not output.stop_required


def test_continuous_green_authorizes_approach_passage_before_light_is_lost():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], green_release_delay=0.3
    )
    machine.update_route("stop_zone", 25)
    machine.update_waypoint_distance(10.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.GREEN, 0.0)
    assert machine.evaluate(0.0).state == DecisionState.APPROACHING_INTERSECTION

    machine.update_waypoint_distance(9.0, 0.31)
    machine.update_planning_speed(3.0, 0.31)
    machine.update_vehicle_speed(2.0, 0.31)
    machine.update_traffic_light(TrafficLight.GREEN, 0.31)
    authorized = machine.evaluate(0.31)

    assert authorized.state == DecisionState.RELEASED
    assert authorized.target_speed == pytest.approx(3.0)

    machine.update_waypoint_distance(5.0, 0.4)
    machine.update_planning_speed(3.0, 0.4)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.4)
    after_loss = machine.evaluate(0.4)

    assert after_loss.state == DecisionState.RELEASED
    assert after_loss.target_speed == pytest.approx(3.0)
    assert not after_loss.stop_required


def test_green_confirmation_policy_can_be_disabled():
    machine = make_machine(
        traffic_light_waypoint_ids=[25],
        require_green_in_controlled_zone=False,
    )
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.APPROACHING_INTERSECTION
    assert output.target_speed == pytest.approx(3.0)


def test_planning_speed_timeout_fails_safe_to_zero():
    machine = make_machine(planning_speed_timeout=0.2)
    machine.update_route("normal", 8)
    machine.update_planning_speed(4.0, 0.0)

    output = machine.evaluate(0.21)

    assert output.target_speed == 0.0
    assert output.reason == "planning_speed_timeout"


def test_merge_vehicle_limits_speed_and_uses_hysteresis():
    machine = make_machine(
        merge_trigger_distance=8.0,
        merge_release_margin=2.0,
        merge_target_speed=2.5,
    )
    machine.update_route("normal", 12)
    machine.update_planning_speed(6.0, 0.0)
    machine.update_obstacle_distances(float("inf"), 7.0, 0.0)

    slowing = machine.evaluate(0.1)
    machine.update_planning_speed(6.0, 0.2)
    machine.update_obstacle_distances(float("inf"), 9.0, 0.2)
    hysteresis = machine.evaluate(0.2)
    machine.update_obstacle_distances(float("inf"), 10.1, 0.3)
    released = machine.evaluate(0.3)

    assert slowing.state == DecisionState.SLOWING_FOR_MERGE
    assert slowing.target_speed == pytest.approx(2.5)
    assert hysteresis.state == DecisionState.SLOWING_FOR_MERGE
    assert released.state == DecisionState.CRUISE
    assert released.target_speed == pytest.approx(6.0)


def test_emergency_braking_has_priority_over_traffic_and_merge():
    machine = make_machine(emergency_min_distance=4.0, merge_target_speed=2.0)
    machine.update_route("stop_zone", 25)
    machine.update_planning_speed(6.0, 0.0)
    machine.update_vehicle_speed(3.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.update_obstacle_distances(2.0, 5.0, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.EMERGENCY_BRAKING
    assert output.target_speed == 0.0
    assert output.stop_required
    assert output.reason == "front_obstacle_emergency"


def test_emergency_threshold_grows_with_vehicle_speed():
    machine = make_machine(
        emergency_min_distance=2.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=5.0,
    )
    machine.update_route("normal", 12)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(10.0, 0.0)
    machine.update_obstacle_distances(14.0, float("inf"), 0.0)

    output = machine.evaluate(0.1)

    # 10 * 0.5 + 10^2 / (2 * 5) = 15 metres.
    assert output.state == DecisionState.EMERGENCY_BRAKING


def test_emergency_releases_only_after_clear_margin():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_release_margin=2.0,
    )
    machine.update_route("normal", 12)
    machine.update_planning_speed(5.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_obstacle_distances(3.0, float("inf"), 0.0)
    assert machine.evaluate(0.1).state == DecisionState.EMERGENCY_BRAKING

    machine.update_planning_speed(5.0, 0.2)
    machine.update_vehicle_speed(0.0, 0.2)
    machine.update_obstacle_distances(5.0, float("inf"), 0.2)
    assert machine.evaluate(0.2).state == DecisionState.EMERGENCY_BRAKING

    machine.update_obstacle_distances(6.1, float("inf"), 0.3)
    released = machine.evaluate(0.3)
    assert released.state == DecisionState.CRUISE
    assert released.target_speed == pytest.approx(5.0)


def test_stale_emergency_observation_remains_fail_safe():
    machine = make_machine(obstacle_timeout=0.2, emergency_min_distance=4.0)
    machine.update_route("normal", 12)
    machine.update_planning_speed(5.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_obstacle_distances(2.0, float("inf"), 0.0)
    machine.evaluate(0.1)
    machine.update_planning_speed(5.0, 0.3)

    stale = machine.evaluate(0.3)

    assert stale.state == DecisionState.EMERGENCY_BRAKING
    assert stale.target_speed == 0.0
