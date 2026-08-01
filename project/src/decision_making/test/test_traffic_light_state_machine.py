import math

from decision_making.traffic_light_state_machine import (
    DecisionState,
    TrafficLight,
    TrafficLightStateMachine,
)
import pytest


def make_machine(**overrides):
    parameters = {
        'controlled_route_states': [
            'intersection_approach',
            'stop_zone',
            'intersection',
        ],
        'stopped_speed_threshold': 0.15,
        'traffic_light_timeout': 0.75,
        'planning_speed_timeout': 0.5,
        'vehicle_speed_timeout': 0.5,
        'green_release_delay': 0.3,
        'treat_yellow_as_red': True,
        'require_green_in_controlled_zone': True,
    }
    parameters.update(overrides)
    return TrafficLightStateMachine(**parameters)


def test_cruise_passes_through_fresh_planning_speed():
    machine = make_machine()
    machine.update_route('normal', 8)
    machine.update_planning_speed(4.0, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.CRUISE
    assert output.target_speed == pytest.approx(4.0)
    assert not output.stop_required


def test_red_light_latches_stop_after_route_manager_advances():
    machine = make_machine()
    machine.update_route('stop_zone', 25)
    machine.update_planning_speed(2.0, 0.0)
    machine.update_vehicle_speed(3.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)

    stopping = machine.evaluate(0.1)
    machine.update_route('curve_entry', 26)
    machine.update_planning_speed(8.0, 0.2)
    still_stopping = machine.evaluate(0.2)

    assert stopping.state == DecisionState.STOPPING_FOR_LIGHT
    assert still_stopping.state == DecisionState.STOPPING_FOR_LIGHT
    assert still_stopping.target_speed == 0.0
    assert still_stopping.stop_required


def test_only_fresh_stopped_speed_enters_waiting_state():
    machine = make_machine(vehicle_speed_timeout=0.2)
    machine.update_route('stop_zone', 25)
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
    machine.update_route('stop_zone', 25)
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


def test_ambiguous_green_does_not_release_strict_waypoint_red_stop():
    machine = make_machine(
        green_release_delay=0.3,
        strict_green_waypoint_ids=[25],
        strict_green_over_red_ratio=1.05,
        strict_green_max_red_score=4,
    )
    machine.update_route('stop_zone', 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.01)
    assert machine.evaluate(0.02).state == DecisionState.WAITING_FOR_GREEN

    machine.update_traffic_light_scores(44, 40, 0.1)
    machine.update_traffic_light(TrafficLight.GREEN, 0.1)
    assert machine.evaluate(0.1).state == DecisionState.WAITING_FOR_GREEN
    machine.update_traffic_light_scores(36, 40, 0.5)
    machine.update_traffic_light(TrafficLight.GREEN, 0.5)
    ambiguous = machine.evaluate(0.5)

    assert ambiguous.state == DecisionState.WAITING_FOR_GREEN
    assert ambiguous.stop_required

    # Even green that is numerically stronger must not release this waypoint
    # while substantial red evidence remains (observed as 44/60 at runtime).
    machine.update_traffic_light_scores(44, 60, 0.6)
    machine.update_traffic_light(TrafficLight.GREEN, 0.6)
    stronger_false_green = machine.evaluate(0.6)
    assert stronger_false_green.state == DecisionState.WAITING_FOR_GREEN
    assert stronger_false_green.stop_required


def test_unambiguous_green_releases_strict_waypoint():
    machine = make_machine(
        green_release_delay=0.3,
        strict_green_waypoint_ids=[25],
        strict_green_over_red_ratio=1.05,
        strict_green_max_red_score=4,
    )
    machine.update_route('stop_zone', 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.01)
    machine.evaluate(0.02)

    machine.update_traffic_light_scores(0, 40, 0.1)
    machine.update_traffic_light(TrafficLight.GREEN, 0.1)
    assert machine.evaluate(0.1).state == DecisionState.WAITING_FOR_GREEN
    machine.update_traffic_light_scores(0, 40, 0.41)
    machine.update_traffic_light(TrafficLight.GREEN, 0.41)
    released = machine.evaluate(0.41)
    assert released.state == DecisionState.RELEASED
    assert not released.stop_required


def test_measured_waypoint25_green_releases_with_small_red_residue():
    machine = make_machine(
        green_release_delay=0.3,
        strict_green_waypoint_ids=[25],
        strict_green_over_red_ratio=1.05,
        strict_green_max_red_score=15,
        strict_green_min_green_score=30,
    )
    machine.update_route('stop_zone', 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.01)
    machine.evaluate(0.02)

    # Closed-loop waypoint-25 evidence from the correctly associated signal.
    machine.update_traffic_light_scores(10, 42, 0.1)
    machine.update_traffic_light(TrafficLight.GREEN, 0.1)
    assert machine.evaluate(0.1).state == DecisionState.WAITING_FOR_GREEN
    machine.update_traffic_light_scores(10, 42, 0.41)
    machine.update_traffic_light(TrafficLight.GREEN, 0.41)
    released = machine.evaluate(0.41)

    assert released.state == DecisionState.RELEASED
    assert not released.stop_required


def test_waypoint25_reproduced_weak_false_green_does_not_release():
    machine = make_machine(
        green_release_delay=0.3,
        strict_green_waypoint_ids=[25],
        strict_green_over_red_ratio=1.05,
        strict_green_max_red_score=10,
        strict_green_min_green_score=30,
        strict_green_release_delay=0.6,
    )
    machine.update_route('stop_zone', 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.01)
    machine.evaluate(0.02)

    # Exact false-release evidence recovered from closed-loop run6.
    machine.update_traffic_light_scores(0, 22, 0.1)
    machine.update_traffic_light(TrafficLight.GREEN, 0.1)
    assert machine.evaluate(0.1).state == DecisionState.WAITING_FOR_GREEN
    assert machine.evaluate(0.5).state == DecisionState.WAITING_FOR_GREEN

    machine.update_traffic_light_scores(34, 12, 0.55)
    machine.update_traffic_light(TrafficLight.RED, 0.55)
    still_waiting = machine.evaluate(0.55)
    assert still_waiting.state == DecisionState.WAITING_FOR_GREEN
    assert still_waiting.stop_required


def test_waypoint25_accepts_strong_green_at_normal_response_delay():
    machine = make_machine(
        green_release_delay=0.3,
        strict_green_waypoint_ids=[25],
        strict_green_max_red_score=10,
        strict_green_min_green_score=30,
        strict_green_release_delay=0.3,
    )
    machine.update_route('stop_zone', 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.01)
    machine.evaluate(0.02)

    machine.update_traffic_light_scores(10, 42, 0.1)
    machine.update_traffic_light(TrafficLight.GREEN, 0.1)
    assert machine.evaluate(0.1).state == DecisionState.WAITING_FOR_GREEN
    machine.update_traffic_light_scores(10, 42, 0.39)
    machine.update_traffic_light(TrafficLight.GREEN, 0.39)
    assert machine.evaluate(0.39).state == DecisionState.WAITING_FOR_GREEN
    machine.update_traffic_light_scores(10, 42, 0.41)
    machine.update_traffic_light(TrafficLight.GREEN, 0.41)
    released = machine.evaluate(0.41)

    assert released.state == DecisionState.RELEASED
    assert not released.stop_required


def test_ambiguous_green_remains_allowed_at_non_strict_verified_waypoint():
    machine = make_machine(
        green_release_delay=0.3,
        strict_green_waypoint_ids=[25],
        strict_green_over_red_ratio=1.05,
    )
    machine.update_route('stop_zone', 52)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.evaluate(0.01)
    machine.evaluate(0.02)

    machine.update_traffic_light_scores(44, 40, 0.1)
    machine.update_traffic_light(TrafficLight.GREEN, 0.1)
    machine.evaluate(0.1)
    machine.update_traffic_light_scores(44, 40, 0.41)
    machine.update_traffic_light(TrafficLight.GREEN, 0.41)
    released = machine.evaluate(0.41)

    assert released.state == DecisionState.RELEASED


def test_release_is_retained_until_vehicle_leaves_controlled_zone():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], green_release_delay=0.1
    )
    machine.update_route('stop_zone', 25)
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

    machine.update_route('curve_entry', 26)
    machine.update_planning_speed(8.0, 0.5)
    left_zone = machine.evaluate(0.5)

    assert left_zone.state == DecisionState.CRUISE
    assert left_zone.target_speed == pytest.approx(8.0)


def test_unknown_or_stale_light_never_releases_waiting_stop():
    machine = make_machine(traffic_light_timeout=0.2, green_release_delay=0.1)
    machine.update_route('stop_zone', 25)
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
    machine.update_route('intersection_approach', 9)
    machine.update_planning_speed(2.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.YELLOW, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.STOPPING_FOR_LIGHT
    assert output.target_speed == 0.0


def test_waypoint_allowlist_rejects_unlisted_stop_zone():
    machine = make_machine(traffic_light_waypoint_ids=[25, 32, 52, 59])
    machine.update_route('stop_zone', 0)
    machine.update_planning_speed(2.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.CRUISE
    assert output.target_speed == pytest.approx(2.0)


def test_unknown_light_stops_at_allowlisted_controlled_waypoint():
    machine = make_machine(traffic_light_waypoint_ids=[25, 32, 52, 59])
    machine.update_route('stop_zone', 25)
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
    machine.update_route('stop_zone', 25)
    machine.update_waypoint_distance(12.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.APPROACHING_INTERSECTION
    assert output.target_speed == pytest.approx(1.0)
    assert not output.stop_required
    assert output.reason == 'unknown_light_controlled_approach'


@pytest.mark.parametrize(
    'waypoint_id, stop_distance',
    [(25, 2.0), (32, 4.0), (52, 3.4), (59, 4.0)],
)
def test_unknown_light_uses_distance_based_braking_envelope(
    waypoint_id, stop_distance
):
    machine = make_machine(
        traffic_light_waypoint_ids=[25, 32, 52, 59],
        unknown_light_stop_distance=4.0,
        unknown_light_approach_speed=8.0,
        traffic_light_approach_deceleration=0.7,
        stop_distance_override_waypoint_ids=[25, 52],
        stop_distance_overrides=[2.0, 3.4],
    )
    machine.update_route('stop_zone', waypoint_id)
    machine.update_waypoint_distance(44.0, 0.0)
    machine.update_planning_speed(14.0, 0.0)
    machine.update_vehicle_speed(4.4, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)

    far_output = machine.evaluate(0.1)
    assert far_output.state == DecisionState.APPROACHING_INTERSECTION
    assert far_output.target_speed == pytest.approx(
        math.sqrt(2.0 * 0.7 * (44.0 - stop_distance))
    )
    assert far_output.target_speed > 4.4
    assert not far_output.stop_required

    machine.update_waypoint_distance(20.0, 0.2)
    machine.update_planning_speed(14.0, 0.2)
    machine.update_vehicle_speed(5.0, 0.2)
    near_output = machine.evaluate(0.2)

    assert near_output.target_speed == pytest.approx(
        math.sqrt(2.0 * 0.7 * (20.0 - stop_distance))
    )
    assert near_output.target_speed < far_output.target_speed


def test_distance_based_approach_envelope_reaches_zero_at_stop_boundary():
    machine = make_machine(
        traffic_light_waypoint_ids=[59],
        unknown_light_stop_distance=4.0,
        unknown_light_approach_speed=8.0,
        traffic_light_approach_deceleration=0.7,
    )
    machine.update_route('stop_zone', 59)
    machine.update_waypoint_distance(6.0, 0.0)
    machine.update_planning_speed(14.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)

    approaching = machine.evaluate(0.1)
    assert approaching.target_speed == pytest.approx(math.sqrt(2.0 * 0.7 * 2.0))

    machine.update_waypoint_distance(4.0, 0.2)
    machine.update_planning_speed(14.0, 0.2)
    machine.update_vehicle_speed(1.0, 0.2)
    stopped = machine.evaluate(0.2)

    assert stopped.state == DecisionState.STOPPING_FOR_LIGHT
    assert stopped.target_speed == 0.0
    assert stopped.stop_required


@pytest.mark.parametrize(
    'waypoint_id, stop_distance',
    [(25, 2.0), (52, 3.4)],
)
def test_waypoint_stop_distance_override_moves_selected_intersections(
    waypoint_id, stop_distance
):
    override_parameters = {
        'traffic_light_waypoint_ids': [25, 52, 59],
        'unknown_light_stop_distance': 4.0,
        'traffic_light_stop_distance': 4.0,
        'stop_distance_override_waypoint_ids': [25, 52],
        'stop_distance_overrides': [2.0, 3.4],
        'traffic_light_approach_deceleration': 0.7,
    }
    machine = make_machine(**override_parameters)
    machine.update_route('stop_zone', waypoint_id)
    machine.update_waypoint_distance(stop_distance + 0.2, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(1.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)

    still_approaching = machine.evaluate(0.1)

    assert still_approaching.state == DecisionState.APPROACHING_INTERSECTION
    assert still_approaching.target_speed == pytest.approx(
        math.sqrt(2.0 * 0.7 * 0.2)
    )
    assert not still_approaching.stop_required

    machine.update_waypoint_distance(stop_distance, 0.2)
    machine.update_planning_speed(3.0, 0.2)
    machine.update_vehicle_speed(0.8, 0.2)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.2)
    selected_stop = machine.evaluate(0.2)

    assert selected_stop.state == DecisionState.STOPPING_FOR_LIGHT
    assert selected_stop.target_speed == 0.0
    assert selected_stop.stop_required


def test_non_overridden_waypoint_retains_default_stop_distance():
    machine = make_machine(
        traffic_light_waypoint_ids=[25, 52, 59],
        unknown_light_stop_distance=4.0,
        traffic_light_stop_distance=4.0,
        stop_distance_override_waypoint_ids=[25, 52],
        stop_distance_overrides=[2.0, 3.4],
    )
    machine.update_route('stop_zone', 59)
    machine.update_waypoint_distance(3.6, 0.0)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(1.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)

    default_stop = machine.evaluate(0.1)

    assert default_stop.state == DecisionState.STOPPING_FOR_LIGHT
    assert default_stop.target_speed == 0.0
    assert default_stop.stop_required


def test_stop_distance_override_arrays_must_have_equal_length():
    with pytest.raises(ValueError):
        make_machine(
            stop_distance_override_waypoint_ids=[25],
            stop_distance_overrides=[],
        )


def test_unknown_light_stops_at_hard_stop_boundary():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], unknown_light_stop_distance=6.0
    )
    machine.update_route('stop_zone', 25)
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
    machine.update_route('stop_zone', 25)
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
    machine.update_route('stop_zone', 25)
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
    assert approaching.reason == 'traffic_light_controlled_approach'
    assert after_light_loss.state == DecisionState.APPROACHING_INTERSECTION
    assert after_light_loss.target_speed == pytest.approx(1.0)


def test_far_red_is_treated_as_unassociated_until_activation_distance():
    machine = make_machine(
        traffic_light_waypoint_ids=[32],
        traffic_light_activation_distance=20.0,
        unknown_light_approach_speed=1.0,
    )
    machine.update_route('stop_zone', 32)
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
    machine.update_route('stop_zone', 32)
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
    machine.update_route('stop_zone', 25)
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
    machine.update_route('stop_zone', 25)
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


def test_confirmed_green_cancels_red_stop_before_standstill():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], traffic_light_stop_distance=4.0
    )
    machine.update_route('stop_zone', 25)
    machine.update_waypoint_distance(4.0, 0.0)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(1.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    assert machine.evaluate(0.1).state == DecisionState.STOPPING_FOR_LIGHT

    machine.update_waypoint_distance(3.8, 0.2)
    machine.update_planning_speed(3.0, 0.2)
    machine.update_vehicle_speed(0.8, 0.2)
    machine.update_traffic_light(TrafficLight.GREEN, 0.2)
    released = machine.evaluate(0.2)

    assert released.state == DecisionState.RELEASED
    assert released.target_speed == pytest.approx(3.0)
    assert not released.stop_required


def test_stale_light_stops_at_allowlisted_controlled_waypoint():
    machine = make_machine(
        traffic_light_waypoint_ids=[25], traffic_light_timeout=0.2
    )
    machine.update_route('stop_zone', 25)
    machine.update_planning_speed(3.0, 0.3)
    machine.update_vehicle_speed(2.0, 0.3)
    machine.update_traffic_light(TrafficLight.GREEN, 0.0)

    output = machine.evaluate(0.3)

    assert output.state == DecisionState.STOPPING_FOR_LIGHT
    assert output.target_speed == 0.0


def test_fresh_green_passes_at_allowlisted_controlled_waypoint():
    machine = make_machine(traffic_light_waypoint_ids=[25])
    machine.update_route('stop_zone', 25)
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
    machine.update_route('stop_zone', 25)
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
    machine.update_route('stop_zone', 25)
    machine.update_planning_speed(3.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_traffic_light(TrafficLight.UNKNOWN, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.APPROACHING_INTERSECTION
    assert output.target_speed == pytest.approx(3.0)


def test_planning_speed_timeout_fails_safe_to_zero():
    machine = make_machine(planning_speed_timeout=0.2)
    machine.update_route('normal', 8)
    machine.update_planning_speed(4.0, 0.0)

    output = machine.evaluate(0.21)

    assert output.target_speed == 0.0
    assert output.reason == 'planning_speed_timeout'


def test_merge_vehicle_limits_speed_and_uses_hysteresis():
    machine = make_machine(
        merge_trigger_distance=12.0,
        merge_release_margin=2.0,
        merge_target_speed=1.5,
    )
    machine.update_route('normal', 12)
    machine.update_planning_speed(6.0, 0.0)
    machine.update_obstacle_distances(float('inf'), 11.5, 0.0)

    slowing = machine.evaluate(0.1)
    machine.update_planning_speed(6.0, 0.2)
    machine.update_obstacle_distances(float('inf'), 13.0, 0.2)
    hysteresis = machine.evaluate(0.2)
    machine.update_obstacle_distances(float('inf'), 14.1, 0.3)
    released = machine.evaluate(0.3)

    assert slowing.state == DecisionState.SLOWING_FOR_MERGE
    assert slowing.target_speed == pytest.approx(1.5)
    assert hysteresis.state == DecisionState.SLOWING_FOR_MERGE
    assert released.state == DecisionState.CRUISE
    assert released.target_speed == pytest.approx(6.0)


def test_merge_latch_survives_measured_pointcloud_gap():
    machine = make_machine(
        obstacle_timeout=1.0,
        merge_stale_timeout=3.0,
        merge_trigger_distance=8.0,
        merge_target_speed=2.5,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_obstacle_distances(math.inf, 2.5, 0.0)
    assert machine.evaluate(0.1).state == DecisionState.SLOWING_FOR_MERGE

    machine.update_planning_speed(8.0, 2.3)
    output = machine.evaluate(2.3)

    assert output.state == DecisionState.SLOWING_FOR_MERGE
    assert output.target_speed == pytest.approx(2.5)
    assert not output.stop_required


def test_stale_merge_observation_stops_until_fresh_safe_clearance():
    machine = make_machine(
        obstacle_timeout=1.0,
        merge_stale_timeout=3.0,
        merge_trigger_distance=8.0,
        merge_release_margin=2.0,
        merge_target_speed=2.5,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_obstacle_distances(math.inf, 2.5, 0.0)
    assert machine.evaluate(0.1).state == DecisionState.SLOWING_FOR_MERGE

    # Front-obstacle updates may continue even when the merge source stops.
    # They must not refresh the merge observation or clear its latch.
    machine.update_front_obstacle_distance(math.inf, 1.2)
    machine.update_front_obstacle_distance(math.inf, 2.2)
    machine.update_front_obstacle_distance(math.inf, 3.1)
    machine.update_planning_speed(8.0, 3.1)
    stale = machine.evaluate(3.1)

    assert stale.state == DecisionState.SLOWING_FOR_MERGE
    assert stale.target_speed == 0.0
    assert stale.stop_required
    assert stale.reason == 'merge_obstacle_stale_stop'

    machine.update_planning_speed(8.0, 3.2)
    machine.update_obstacle_distances(math.inf, 5.0, 3.2)
    still_unsafe = machine.evaluate(3.2)

    assert still_unsafe.state == DecisionState.SLOWING_FOR_MERGE
    assert still_unsafe.target_speed == pytest.approx(2.5)
    assert not still_unsafe.stop_required

    machine.update_planning_speed(8.0, 3.3)
    machine.update_obstacle_distances(math.inf, math.inf, 3.3)
    released = machine.evaluate(3.3)

    assert released.state == DecisionState.CRUISE
    assert released.target_speed == pytest.approx(8.0)
    assert not released.stop_required


def test_emergency_braking_has_priority_over_traffic_and_merge():
    machine = make_machine(emergency_min_distance=4.0, merge_target_speed=2.0)
    machine.update_route('stop_zone', 25)
    machine.update_planning_speed(6.0, 0.0)
    machine.update_vehicle_speed(3.0, 0.0)
    machine.update_traffic_light(TrafficLight.RED, 0.0)
    machine.update_obstacle_distances(2.0, 5.0, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.EMERGENCY_BRAKING
    assert output.target_speed == 0.0
    assert output.stop_required
    assert output.reason == 'front_obstacle_emergency'


def test_emergency_threshold_grows_with_relative_closing_speed():
    machine = make_machine(
        emergency_min_distance=2.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=5.0,
        following_max_abs_range_rate=10.0,
    )
    machine.update_route('normal', 12)
    machine.update_obstacle_distances(20.0, float('inf'), 0.0)
    machine.update_planning_speed(8.0, 0.6)
    machine.update_vehicle_speed(10.0, 0.6)
    machine.update_obstacle_distances(14.0, float('inf'), 0.6)

    output = machine.evaluate(0.6)

    # Relative closing speed is 10 m/s:
    # 10 * 0.5 + 10^2 / (2 * 5) = 15 metres.
    assert output.state == DecisionState.EMERGENCY_BRAKING


def test_stable_lead_vehicle_does_not_emergency_brake_from_ego_speed_alone():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=0.7,
    )
    machine.update_route('normal', 45)
    machine.update_planning_speed(14.0, 0.0)
    machine.update_vehicle_speed(4.0, 0.0)
    machine.update_obstacle_distances(18.0, 18.0, 0.0)

    output = machine.evaluate(0.1)

    assert output.state == DecisionState.FOLLOWING_VEHICLE
    assert output.target_speed == pytest.approx(4.0)
    assert not output.stop_required


def test_emergency_releases_only_after_clear_margin():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_release_margin=2.0,
    )
    machine.update_route('normal', 12)
    machine.update_planning_speed(5.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_obstacle_distances(3.0, float('inf'), 0.0)
    assert machine.evaluate(0.1).state == DecisionState.EMERGENCY_BRAKING

    machine.update_planning_speed(5.0, 0.2)
    machine.update_vehicle_speed(0.0, 0.2)
    machine.update_obstacle_distances(5.0, float('inf'), 0.2)
    assert machine.evaluate(0.2).state == DecisionState.EMERGENCY_BRAKING

    machine.update_obstacle_distances(6.1, float('inf'), 0.3)
    released = machine.evaluate(0.3)
    assert released.state == DecisionState.FOLLOWING_VEHICLE
    assert not machine.emergency_latched
    assert released.target_speed > 0.0


def test_waypoint47_far_closing_vehicle_uses_adaptive_braking():
    machine = make_machine(
        stopped_speed_threshold=0.15,
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=0.7,
        emergency_release_margin=2.0,
        emergency_min_hold_time=2.5,
        emergency_trigger_max_distance=10.0,
        merge_trigger_distance=0.0,
    )
    machine.update_route('normal', 47)
    machine.update_obstacle_distances(19.50, 19.50, 0.0)
    machine.update_planning_speed(14.0, 1.3)
    machine.update_vehicle_speed(3.27, 1.3)
    machine.update_obstacle_distances(15.07, 15.07, 1.3)
    slowing = machine.evaluate(1.3)

    assert slowing.state == DecisionState.FOLLOWING_VEHICLE
    assert 0.0 < slowing.target_speed < 3.27
    assert not slowing.stop_required


def test_emergency_stays_latched_while_gap_is_still_closing():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=0.7,
        emergency_min_hold_time=1.0,
        emergency_trigger_max_distance=10.0,
        emergency_follow_release_min_distance=5.4,
        emergency_follow_release_min_range_rate=-0.3,
        emergency_follow_release_max_speed=2.0,
        emergency_follow_release_confirmation=0.3,
        following_range_rate_alpha=1.0,
        merge_trigger_distance=0.0,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(3.0, 0.0)
    machine.update_obstacle_distances(10.0, 10.0, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(3.0, 1.0)
    machine.update_obstacle_distances(6.0, 6.0, 1.0)
    assert machine.evaluate(1.0).state == DecisionState.EMERGENCY_BRAKING

    machine.update_planning_speed(8.0, 3.0)
    machine.update_vehicle_speed(1.5, 3.0)
    machine.update_obstacle_distances(5.0, 5.0, 3.0)
    machine.evaluate(3.0)
    still_braking = machine.evaluate(3.4)

    assert still_braking.state == DecisionState.EMERGENCY_BRAKING
    assert still_braking.target_speed == 0.0
    assert still_braking.stop_required


def test_emergency_hands_back_to_following_when_lead_vehicle_moves():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=0.7,
        emergency_min_hold_time=1.0,
        emergency_trigger_max_distance=10.0,
        emergency_follow_release_min_distance=5.4,
        emergency_follow_release_min_range_rate=-0.6,
        emergency_follow_release_max_speed=2.0,
        emergency_follow_release_confirmation=0.1,
        following_range_rate_alpha=1.0,
        merge_trigger_distance=0.0,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(3.0, 0.0)
    machine.update_obstacle_distances(10.0, 10.0, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(3.0, 1.0)
    machine.update_obstacle_distances(6.8, 6.8, 1.0)
    assert machine.evaluate(1.0).state == DecisionState.EMERGENCY_BRAKING

    # Ego is still moving, but the filtered range now shrinks by only 0.55 m/s,
    # matching the first lead-vehicle recovery evidence observed at waypoint 47.
    machine.update_planning_speed(8.0, 3.0)
    machine.update_vehicle_speed(1.8, 3.0)
    machine.update_obstacle_distances(5.7, 5.7, 3.0)
    assert machine.evaluate(3.0).state == DecisionState.EMERGENCY_BRAKING
    released = machine.evaluate(3.11)

    assert released.state == DecisionState.FOLLOWING_VEHICLE
    assert released.target_speed == pytest.approx(1.8)
    assert not released.stop_required
    assert not machine.emergency_latched


def test_emergency_handoff_recovery_does_not_continue_braking():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_deceleration=0.7,
        emergency_min_hold_time=1.0,
        emergency_trigger_max_distance=10.0,
        emergency_follow_release_min_distance=5.4,
        emergency_follow_release_min_range_rate=-0.6,
        emergency_follow_release_max_speed=2.0,
        emergency_follow_release_confirmation=0.1,
        emergency_follow_recovery_hold_time=1.0,
        following_range_rate_alpha=1.0,
        merge_trigger_distance=0.0,
    )
    machine.update_route('curve_mid', 48)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(2.5, 0.0)
    machine.update_obstacle_distances(10.0, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(2.5, 1.0)
    machine.update_obstacle_distances(6.5, math.inf, 1.0)
    assert machine.evaluate(1.0).state == DecisionState.EMERGENCY_BRAKING

    machine.update_planning_speed(8.0, 2.2)
    machine.update_vehicle_speed(1.7, 2.2)
    machine.update_obstacle_distances(6.2, math.inf, 2.2)
    assert machine.evaluate(2.2).state == DecisionState.EMERGENCY_BRAKING
    released = machine.evaluate(2.31)
    assert released.state == DecisionState.FOLLOWING_VEHICLE
    assert released.target_speed == pytest.approx(1.7)

    # Brake inertia has reduced actual speed, but recovery hold must not ask
    # for still more braking while the confirmed lead-vehicle recovery stands.
    machine.update_planning_speed(8.0, 2.7)
    machine.update_vehicle_speed(1.2, 2.7)
    machine.update_obstacle_distances(6.3, math.inf, 2.7)
    recovering = machine.evaluate(2.7)
    assert recovering.state == DecisionState.FOLLOWING_VEHICLE
    assert recovering.target_speed >= 1.2


def test_emergency_uses_fresh_raw_opening_before_filtered_rate_recovers():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_deceleration=0.7,
        emergency_min_hold_time=1.0,
        emergency_trigger_max_distance=10.0,
        emergency_follow_release_min_distance=5.4,
        emergency_follow_release_min_range_rate=-0.6,
        emergency_follow_release_min_raw_range_rate=0.5,
        emergency_follow_release_max_speed=2.0,
        emergency_follow_release_confirmation=0.1,
        following_range_rate_alpha=0.1,
        merge_trigger_distance=0.0,
    )
    machine.update_route('normal', 48)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(3.0, 0.0)
    machine.update_obstacle_distances(10.0, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(2.8, 1.0)
    machine.update_obstacle_distances(6.8, math.inf, 1.0)
    assert machine.evaluate(1.0).state == DecisionState.EMERGENCY_BRAKING

    # The lead vehicle starts pulling away, but the deliberately slow low-pass
    # estimate is still strongly negative. Fresh raw opening evidence should
    # hand control back to adaptive following before ego reaches standstill.
    machine.update_planning_speed(8.0, 2.0)
    machine.update_vehicle_speed(1.5, 2.0)
    machine.update_obstacle_distances(7.8, math.inf, 2.0)
    assert machine.front_obstacle_range_rate < -0.6
    assert machine.front_obstacle_raw_range_rate >= 0.5
    assert machine.evaluate(2.0).state == DecisionState.EMERGENCY_BRAKING

    released = machine.evaluate(2.11)

    assert released.state == DecisionState.FOLLOWING_VEHICLE
    assert 1.5 < released.target_speed <= 8.0
    assert not released.stop_required
    assert not machine.emergency_latched


def test_adaptive_following_holds_speed_at_desired_stable_gap():
    machine = make_machine(
        emergency_standstill_distance=6.0,
        following_time_headway=1.5,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(4.0, 0.0)
    machine.update_obstacle_distances(12.0, 12.0, 0.0)

    output = machine.evaluate(0.0)

    assert output.state == DecisionState.FOLLOWING_VEHICLE
    assert output.target_speed == pytest.approx(4.0)
    assert not output.stop_required


def test_adaptive_following_does_not_accelerate_into_non_opening_gap():
    machine = make_machine(
        emergency_standstill_distance=6.0,
        following_time_headway=2.2,
        following_distance_gain=0.1,
        following_range_rate_alpha=1.0,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(3.5, 0.0)
    machine.update_obstacle_distances(19.0, 19.0, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 0.5)
    machine.update_vehicle_speed(3.5, 0.5)
    machine.update_obstacle_distances(18.9, 18.9, 0.5)
    non_opening = machine.evaluate(0.5)

    assert non_opening.state == DecisionState.FOLLOWING_VEHICLE
    assert non_opening.target_speed <= 3.5

    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(3.5, 1.0)
    machine.update_obstacle_distances(19.5, 19.5, 1.0)
    opening = machine.evaluate(1.0)

    assert opening.target_speed > 3.5


def test_adaptive_following_brakes_early_on_closing_vehicle():
    machine = make_machine(
        emergency_standstill_distance=6.0,
        following_time_headway=2.2,
        following_distance_gain=0.1,
        following_range_rate_alpha=1.0,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(3.7, 0.0)
    machine.update_obstacle_distances(18.8, 18.8, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(3.7, 1.0)
    machine.update_obstacle_distances(18.3, 18.3, 1.0)
    slowing = machine.evaluate(1.0)

    assert slowing.state == DecisionState.FOLLOWING_VEHICLE
    assert 0.0 < slowing.target_speed < 3.7


def test_adaptive_following_keeps_rolling_with_safe_surplus_gap():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_deceleration=0.7,
        emergency_trigger_max_distance=10.0,
        following_time_headway=2.2,
        following_distance_gain=0.1,
        following_range_rate_alpha=1.0,
        following_min_rolling_speed=0.6,
        following_min_rolling_clearance=2.0,
        following_target_deceleration=4.0,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(1.72, 0.0)
    machine.update_obstacle_distances(13.46, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(1.72, 1.0)
    machine.update_obstacle_distances(11.29, math.inf, 1.0)
    output = machine.evaluate(1.0)

    assert output.state == DecisionState.FOLLOWING_VEHICLE
    assert output.target_speed == pytest.approx(0.6)
    assert output.target_speed <= 1.72
    assert not output.stop_required
    assert not machine.emergency_latched


def test_rolling_following_floor_never_overrides_near_field_emergency():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_deceleration=0.7,
        emergency_trigger_max_distance=10.0,
        following_min_rolling_speed=0.6,
        following_min_rolling_clearance=2.0,
        following_range_rate_alpha=1.0,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(1.0, 0.0)
    machine.update_obstacle_distances(9.5, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 0.5)
    machine.update_vehicle_speed(1.0, 0.5)
    machine.update_obstacle_distances(7.0, math.inf, 0.5)
    output = machine.evaluate(0.5)

    assert output.state == DecisionState.EMERGENCY_BRAKING
    assert output.target_speed == 0.0
    assert output.stop_required


def test_safe_rolling_gap_rejects_impossible_rgbd_closing_spike():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=0.7,
        emergency_trigger_max_distance=10.0,
        following_min_rolling_speed=0.6,
        following_min_rolling_clearance=2.0,
        following_emergency_closing_speed_margin=0.5,
        following_range_rate_alpha=1.0,
    )
    machine.update_route('curve_approach', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(1.69, 0.0)
    machine.update_obstacle_distances(13.41, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(1.69, 1.0)
    machine.update_obstacle_distances(9.84, math.inf, 1.0)
    output = machine.evaluate(1.0)

    assert machine.front_obstacle_range_rate == pytest.approx(-3.57)
    assert machine.emergency_closing_speed == pytest.approx(2.19)
    assert machine.emergency_distance < 9.84
    assert output.state == DecisionState.FOLLOWING_VEHICLE
    assert output.target_speed == pytest.approx(0.6)
    assert not machine.emergency_latched


def test_event2_uses_progressive_following_outside_seven_metre_near_field():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=0.7,
        emergency_trigger_max_distance=7.0,
        following_min_rolling_speed=0.6,
        following_min_rolling_clearance=2.0,
        following_emergency_closing_speed_margin=0.5,
        following_range_rate_alpha=1.0,
    )
    machine.update_route('curve_approach', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(2.6, 0.0)
    machine.update_obstacle_distances(12.0, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 0.5)
    machine.update_vehicle_speed(2.4, 0.5)
    machine.update_obstacle_distances(8.0, math.inf, 0.5)
    progressive = machine.evaluate(0.5)

    assert machine.emergency_distance == pytest.approx(7.0)
    assert progressive.state == DecisionState.FOLLOWING_VEHICLE
    assert progressive.target_speed > 0.0
    assert not progressive.stop_required

    machine.update_planning_speed(8.0, 0.8)
    machine.update_vehicle_speed(2.0, 0.8)
    machine.update_obstacle_distances(6.8, math.inf, 0.8)
    near_field = machine.evaluate(0.8)

    assert near_field.state == DecisionState.EMERGENCY_BRAKING
    assert near_field.target_speed == 0.0
    assert near_field.stop_required


def test_full_closing_rate_returns_inside_rolling_clearance_boundary():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=0.7,
        emergency_trigger_max_distance=10.0,
        following_min_rolling_speed=0.6,
        following_min_rolling_clearance=2.0,
        following_emergency_closing_speed_margin=0.5,
        following_range_rate_alpha=1.0,
    )
    machine.update_route('curve_approach', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(1.0, 0.0)
    machine.update_obstacle_distances(10.0, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 0.5)
    machine.update_vehicle_speed(1.0, 0.5)
    machine.update_obstacle_distances(7.9, math.inf, 0.5)
    output = machine.evaluate(0.5)

    assert machine.emergency_closing_speed == pytest.approx(4.2)
    assert output.state == DecisionState.EMERGENCY_BRAKING
    assert output.target_speed == 0.0


def test_event2_stable_nineteen_metre_gap_does_not_create_first_full_stop():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_reaction_time=0.5,
        emergency_deceleration=0.7,
        following_time_headway=1.5,
    )
    machine.update_route('normal', 44)
    samples = [
        (0.0, 3.57, 19.03),
        (0.5, 3.66, 19.02),
        (1.0, 3.75, 18.84),
        (1.5, 3.83, 18.65),
        (2.0, 3.86, 18.46),
        (2.5, 3.90, 18.35),
    ]

    outputs = []
    for now, speed, distance in samples:
        machine.update_planning_speed(14.0, now)
        machine.update_vehicle_speed(speed, now)
        machine.update_obstacle_distances(distance, distance, now)
        outputs.append(machine.evaluate(now))

    assert all(
        output.state == DecisionState.FOLLOWING_VEHICLE
        for output in outputs
    )
    assert not machine.emergency_latched


def test_adaptive_following_reduces_target_when_gap_is_closing():
    machine = make_machine(
        emergency_standstill_distance=6.0,
        emergency_deceleration=5.0,
        following_time_headway=1.5,
        following_distance_gain=0.35,
        following_target_deceleration=2.0,
    )
    machine.update_route('normal', 47)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(4.0, 0.0)
    machine.update_obstacle_distances(15.0, 15.0, 0.0)
    initial = machine.evaluate(0.0)

    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(4.0, 1.0)
    machine.update_obstacle_distances(13.0, 13.0, 1.0)
    closing = machine.evaluate(1.0)

    assert initial.target_speed == pytest.approx(4.0)
    assert closing.state == DecisionState.FOLLOWING_VEHICLE
    assert 0.0 < closing.target_speed < initial.target_speed
    assert machine.front_obstacle_range_rate == pytest.approx(-2.0)


def test_event1_following_uses_comfortable_route_specific_deceleration():
    machine = make_machine(
        emergency_standstill_distance=6.0,
        emergency_trigger_max_distance=10.0,
        following_time_headway=2.2,
        following_distance_gain=0.1,
        following_range_rate_alpha=1.0,
        following_target_deceleration=4.0,
        smooth_following_waypoint_ids=[17, 18, 19, 20],
        smooth_following_target_deceleration=2.0,
    )
    machine.update_route('normal', 17)
    machine.update_planning_speed(14.0, 0.0)
    machine.update_vehicle_speed(6.4, 0.0)
    machine.update_obstacle_distances(19.5, math.inf, 0.0)
    initial = machine.evaluate(0.0)

    machine.update_route('normal', 18)
    machine.update_planning_speed(14.0, 1.0)
    machine.update_vehicle_speed(6.4, 1.0)
    machine.update_obstacle_distances(16.4, math.inf, 1.0)
    slowing = machine.evaluate(1.0)

    assert initial.target_speed == pytest.approx(6.4)
    assert slowing.state == DecisionState.FOLLOWING_VEHICLE
    assert slowing.target_speed == pytest.approx(4.4)
    assert not slowing.stop_required
    assert not machine.emergency_latched


def test_new_far_field_following_association_has_no_instant_target_step():
    machine = make_machine(
        emergency_standstill_distance=6.0,
        emergency_trigger_max_distance=10.0,
        following_time_headway=2.2,
        following_distance_gain=0.1,
        following_target_deceleration=4.0,
        smooth_following_waypoint_ids=[17, 18, 19, 20],
        smooth_following_target_deceleration=2.0,
    )
    machine.update_route('normal', 17)
    machine.update_planning_speed(14.0, 0.0)
    machine.update_vehicle_speed(6.4, 0.0)
    machine.update_obstacle_distances(15.0, math.inf, 0.0)

    output = machine.evaluate(0.0)

    assert output.state == DecisionState.FOLLOWING_VEHICLE
    assert machine.following_raw_target_speed < 6.4
    assert output.target_speed == pytest.approx(6.4)
    assert not machine.emergency_latched


def test_event1_following_holds_across_short_detection_dropout():
    machine = make_machine(
        obstacle_timeout=1.0,
        emergency_standstill_distance=6.0,
        emergency_trigger_max_distance=10.0,
        following_time_headway=2.2,
        following_distance_gain=0.1,
        smooth_following_waypoint_ids=[17, 18, 19, 20],
        smooth_following_target_deceleration=2.0,
        smooth_following_release_margin=2.0,
        smooth_following_dropout_hold_time=1.3,
    )
    machine.update_route('normal', 18)
    machine.update_planning_speed(14.0, 0.0)
    machine.update_vehicle_speed(6.4, 0.0)
    machine.update_obstacle_distances(19.0, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(14.0, 0.5)
    machine.update_vehicle_speed(6.2, 0.5)
    machine.update_obstacle_distances(math.inf, math.inf, 0.5)
    held = machine.evaluate(0.5)

    assert held.state == DecisionState.FOLLOWING_VEHICLE
    assert held.reason == 'adaptive_vehicle_following_hold'
    assert held.target_speed < 14.0
    assert not held.stop_required

    machine.update_planning_speed(14.0, 1.4)
    machine.update_vehicle_speed(6.0, 1.4)
    machine.update_obstacle_distances(math.inf, math.inf, 1.4)
    released = machine.evaluate(1.4)

    assert released.state == DecisionState.CRUISE
    assert released.target_speed == pytest.approx(14.0)


def test_event1_dropout_retains_last_safe_command_while_ego_slows():
    """A short vision gap must not ratchet the follow command to zero."""
    machine = make_machine(
        obstacle_timeout=1.0,
        emergency_standstill_distance=6.0,
        emergency_trigger_max_distance=10.0,
        following_time_headway=2.2,
        following_distance_gain=0.1,
        following_target_acceleration=1.0,
        smooth_following_waypoint_ids=[17, 18, 19, 20],
        smooth_following_target_deceleration=2.0,
        smooth_following_release_margin=2.0,
        smooth_following_dropout_hold_time=1.3,
    )
    machine.update_route('normal', 20)
    machine.update_planning_speed(4.5, 0.0)
    machine.update_vehicle_speed(1.2, 0.0)
    machine.update_obstacle_distances(14.4, math.inf, 0.0)
    initial = machine.evaluate(0.0)

    held = initial
    for now, speed in ((0.2, 0.8), (0.4, 0.4), (0.6, 0.1)):
        machine.update_planning_speed(4.5, now)
        machine.update_vehicle_speed(speed, now)
        machine.update_obstacle_distances(math.inf, math.inf, now)
        held = machine.evaluate(now)

    assert initial.state == DecisionState.FOLLOWING_VEHICLE
    assert held.state == DecisionState.FOLLOWING_VEHICLE
    assert held.reason == 'adaptive_vehicle_following_hold'
    assert held.target_speed == pytest.approx(initial.target_speed)
    assert not held.stop_required


def test_event1_safe_surplus_gap_recovers_from_low_speed():
    """A far, stable lead must allow gradual recovery instead of zero lock."""
    machine = make_machine(
        stopped_speed_threshold=0.15,
        emergency_standstill_distance=6.0,
        emergency_trigger_max_distance=10.0,
        following_time_headway=2.2,
        following_distance_gain=0.1,
        following_range_rate_deadband=0.2,
        following_target_acceleration=1.0,
        smooth_following_waypoint_ids=[17, 18, 19, 20],
        smooth_following_target_deceleration=2.0,
    )
    machine.update_route('normal', 20)
    machine.update_planning_speed(4.5, 0.0)
    machine.update_vehicle_speed(0.1, 0.0)
    machine.update_obstacle_distances(19.5, math.inf, 0.0)
    initial = machine.evaluate(0.0)

    machine.update_planning_speed(4.5, 0.2)
    machine.update_vehicle_speed(0.1, 0.2)
    machine.update_obstacle_distances(19.5, math.inf, 0.2)
    recovering = machine.evaluate(0.2)

    assert initial.target_speed == pytest.approx(0.1)
    assert recovering.state == DecisionState.FOLLOWING_VEHICLE
    assert recovering.target_speed > initial.target_speed
    assert recovering.target_speed == pytest.approx(0.3)
    assert not recovering.stop_required


def test_event1_following_release_margin_prevents_twenty_metre_chatter():
    machine = make_machine(
        emergency_standstill_distance=6.0,
        emergency_trigger_max_distance=10.0,
        following_time_headway=2.2,
        smooth_following_waypoint_ids=[17, 18, 19, 20],
        smooth_following_target_deceleration=2.0,
        smooth_following_release_margin=2.0,
    )
    machine.update_route('normal', 20)
    machine.update_planning_speed(14.0, 0.0)
    machine.update_vehicle_speed(4.0, 0.0)
    machine.update_obstacle_distances(19.9, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(14.0, 0.5)
    machine.update_vehicle_speed(4.0, 0.5)
    machine.update_obstacle_distances(20.1, math.inf, 0.5)
    output = machine.evaluate(0.5)

    assert output.state == DecisionState.FOLLOWING_VEHICLE
    assert output.target_speed <= 4.5
    assert not output.stop_required


def test_event1_continuity_never_blocks_near_field_emergency():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_standstill_distance=6.0,
        emergency_trigger_max_distance=10.0,
        emergency_deceleration=0.7,
        smooth_following_waypoint_ids=[17, 18, 19, 20],
        smooth_following_dropout_hold_time=1.3,
    )
    machine.update_route('normal', 18)
    machine.update_planning_speed(14.0, 0.0)
    machine.update_vehicle_speed(4.0, 0.0)
    machine.update_obstacle_distances(19.0, math.inf, 0.0)
    machine.evaluate(0.0)

    machine.update_planning_speed(14.0, 0.5)
    machine.update_vehicle_speed(4.0, 0.5)
    machine.update_obstacle_distances(8.0, math.inf, 0.5)
    emergency = machine.evaluate(0.5)

    assert emergency.state == DecisionState.EMERGENCY_BRAKING
    assert emergency.target_speed == 0.0
    assert emergency.stop_required


def test_t_junction_cross_traffic_is_ignored_before_conflict_waypoints():
    machine = make_machine(
        vehicle_behavior_waypoint_ids=[43, 44, 45, 46, 47, 48],
        emergency_standstill_distance=6.0,
        emergency_deceleration=0.7,
    )
    machine.update_route('normal', 38)
    machine.update_obstacle_distances(22.0, 22.0, 0.0)
    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(4.0, 1.0)
    machine.update_obstacle_distances(17.0, 17.0, 1.0)

    output = machine.evaluate(1.0)

    assert output.state == DecisionState.CRUISE
    assert output.target_speed == pytest.approx(8.0)
    assert not machine.emergency_latched


def test_global_front_vehicle_following_applies_outside_event2_waypoints():
    machine = make_machine(
        vehicle_behavior_waypoint_ids=[],
        merge_behavior_waypoint_ids=[43, 44, 45, 46, 47, 48],
        following_activation_distance=20.0,
        emergency_standstill_distance=6.0,
    )
    machine.update_route('normal', 12)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(3.5, 0.0)
    machine.update_obstacle_distances(15.0, math.inf, 0.0)

    output = machine.evaluate(0.0)

    assert output.state == DecisionState.FOLLOWING_VEHICLE
    assert output.target_speed <= 3.5
    assert not output.stop_required


def test_event2_merge_slowdown_does_not_apply_globally():
    machine = make_machine(
        vehicle_behavior_waypoint_ids=[],
        merge_behavior_waypoint_ids=[43, 44, 45, 46, 47, 48],
        following_enabled=False,
        merge_trigger_distance=12.0,
        merge_target_speed=1.5,
    )
    machine.update_route('normal', 12)
    machine.update_planning_speed(8.0, 0.0)
    machine.update_vehicle_speed(3.0, 0.0)
    machine.update_obstacle_distances(math.inf, 8.0, 0.0)

    outside_event2 = machine.evaluate(0.0)
    machine.update_route('normal', 47)
    inside_event2 = machine.evaluate(0.1)

    assert outside_event2.state == DecisionState.CRUISE
    assert outside_event2.target_speed == pytest.approx(8.0)
    assert inside_event2.state == DecisionState.SLOWING_FOR_MERGE
    assert inside_event2.target_speed == pytest.approx(1.5)


def test_far_braking_lead_vehicle_uses_following_not_full_emergency():
    machine = make_machine(
        vehicle_behavior_waypoint_ids=[43, 44, 45, 46, 47, 48],
        following_activation_distance=20.0,
        emergency_standstill_distance=6.0,
        emergency_deceleration=0.7,
        emergency_trigger_max_distance=10.0,
    )
    machine.update_route('normal', 47)
    machine.update_obstacle_distances(24.0, 24.0, 0.0)
    machine.update_planning_speed(8.0, 1.3)
    machine.update_vehicle_speed(4.0, 1.3)
    machine.update_obstacle_distances(18.4, 18.4, 1.3)

    output = machine.evaluate(1.3)

    assert machine.front_obstacle_range_rate < -4.0
    assert output.state == DecisionState.FOLLOWING_VEHICLE
    assert output.target_speed < 4.0
    assert not output.stop_required
    assert not machine.emergency_latched


def test_near_fast_closing_vehicle_retains_full_emergency_stop():
    machine = make_machine(
        vehicle_behavior_waypoint_ids=[43, 44, 45, 46, 47, 48],
        emergency_standstill_distance=6.0,
        emergency_deceleration=0.7,
        emergency_trigger_max_distance=10.0,
    )
    machine.update_route('normal', 47)
    machine.update_obstacle_distances(14.0, 14.0, 0.0)
    machine.update_planning_speed(8.0, 1.0)
    machine.update_vehicle_speed(3.0, 1.0)
    machine.update_obstacle_distances(9.0, 9.0, 1.0)

    output = machine.evaluate(1.0)

    assert output.state == DecisionState.EMERGENCY_BRAKING
    assert output.target_speed == 0.0
    assert output.stop_required


def test_emergency_respects_minimum_hold_time_after_detection_flicker():
    machine = make_machine(
        emergency_min_distance=4.0,
        emergency_release_margin=2.0,
        emergency_min_hold_time=1.0,
    )
    machine.update_route('normal', 12)
    machine.update_planning_speed(5.0, 0.0)
    machine.update_vehicle_speed(2.0, 0.0)
    machine.update_obstacle_distances(3.0, float('inf'), 0.0)
    assert machine.evaluate(0.0).state == DecisionState.EMERGENCY_BRAKING

    machine.update_obstacle_distances(float('inf'), float('inf'), 0.1)
    held = machine.evaluate(0.1)
    assert held.state == DecisionState.EMERGENCY_BRAKING

    machine.update_planning_speed(5.0, 1.01)
    machine.update_vehicle_speed(0.0, 1.01)
    machine.update_obstacle_distances(float('inf'), float('inf'), 1.01)
    released = machine.evaluate(1.01)
    assert released.state == DecisionState.CRUISE


def test_stale_emergency_observation_remains_fail_safe():
    machine = make_machine(obstacle_timeout=0.2, emergency_min_distance=4.0)
    machine.update_route('normal', 12)
    machine.update_planning_speed(5.0, 0.0)
    machine.update_vehicle_speed(0.0, 0.0)
    machine.update_obstacle_distances(2.0, float('inf'), 0.0)
    machine.evaluate(0.1)
    machine.update_planning_speed(5.0, 0.3)

    stale = machine.evaluate(0.3)

    assert stale.state == DecisionState.EMERGENCY_BRAKING
    assert stale.target_speed == 0.0
