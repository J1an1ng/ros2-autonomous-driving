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


def test_planning_speed_timeout_fails_safe_to_zero():
    machine = make_machine(planning_speed_timeout=0.2)
    machine.update_route("normal", 8)
    machine.update_planning_speed(4.0, 0.0)

    output = machine.evaluate(0.21)

    assert output.target_speed == 0.0
    assert output.reason == "planning_speed_timeout"
