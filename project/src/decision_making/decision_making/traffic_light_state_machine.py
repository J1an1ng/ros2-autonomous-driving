"""ROS-independent traffic-light decision state machine."""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Optional, Sequence


class TrafficLight(IntEnum):
    UNKNOWN = 0
    RED = 1
    YELLOW = 2
    GREEN = 3


class DecisionState(IntEnum):
    CRUISE = 0
    APPROACHING_INTERSECTION = 1
    STOPPING_FOR_LIGHT = 2
    WAITING_FOR_GREEN = 3
    RELEASED = 4
    SLOWING_FOR_MERGE = 5
    EMERGENCY_BRAKING = 6


@dataclass(frozen=True)
class DecisionOutput:
    target_speed: float
    state: DecisionState
    stop_required: bool
    reason: str


class TrafficLightStateMachine:
    """Select a safe target speed from route, planning and traffic-light inputs.

    Time values passed to update methods and :meth:`evaluate` must use the same
    monotonic clock. The class deliberately has no ROS dependency so transition
    behavior can be tested without a running ROS graph.
    """

    def __init__(
        self,
        controlled_route_states: Sequence[str],
        traffic_light_waypoint_ids: Sequence[int] = (),
        stopped_speed_threshold: float = 0.15,
        traffic_light_timeout: float = 0.75,
        planning_speed_timeout: float = 0.5,
        vehicle_speed_timeout: float = 0.5,
        green_release_delay: float = 0.3,
        treat_yellow_as_red: bool = True,
        obstacle_timeout: float = 0.5,
        emergency_min_distance: float = 4.0,
        emergency_reaction_time: float = 0.5,
        emergency_deceleration: float = 5.0,
        emergency_release_margin: float = 2.0,
        merge_trigger_distance: float = 10.0,
        merge_release_margin: float = 2.0,
        merge_target_speed: float = 2.0,
    ) -> None:
        self.controlled_route_states = {
            state.strip().lower() for state in controlled_route_states if state.strip()
        }
        self.traffic_light_waypoint_ids = {
            int(waypoint_id) for waypoint_id in traffic_light_waypoint_ids
        }
        self.stopped_speed_threshold = max(0.0, float(stopped_speed_threshold))
        self.traffic_light_timeout = max(0.0, float(traffic_light_timeout))
        self.planning_speed_timeout = max(0.0, float(planning_speed_timeout))
        self.vehicle_speed_timeout = max(0.0, float(vehicle_speed_timeout))
        self.green_release_delay = max(0.0, float(green_release_delay))
        self.treat_yellow_as_red = bool(treat_yellow_as_red)
        self.obstacle_timeout = max(0.0, float(obstacle_timeout))
        self.emergency_min_distance = max(0.0, float(emergency_min_distance))
        self.emergency_reaction_time = max(0.0, float(emergency_reaction_time))
        self.emergency_deceleration = max(1e-3, float(emergency_deceleration))
        self.emergency_release_margin = max(0.0, float(emergency_release_margin))
        self.merge_trigger_distance = max(0.0, float(merge_trigger_distance))
        self.merge_release_margin = max(0.0, float(merge_release_margin))
        self.merge_target_speed = max(0.0, float(merge_target_speed))

        self.state = DecisionState.CRUISE
        self.route_state = ""
        self.waypoint_id: Optional[int] = None
        self.planning_speed: Optional[float] = None
        self.planning_speed_time: Optional[float] = None
        self.vehicle_speed: Optional[float] = None
        self.vehicle_speed_time: Optional[float] = None
        self.traffic_light = TrafficLight.UNKNOWN
        self.traffic_light_time: Optional[float] = None
        self.green_since: Optional[float] = None
        self.latched_waypoint_id: Optional[int] = None
        self.front_obstacle_distance = math.inf
        self.merge_obstacle_distance = math.inf
        self.obstacle_time: Optional[float] = None
        self.emergency_latched = False
        self.merge_latched = False

    def update_route(self, route_state: str, waypoint_id: Optional[int] = None) -> None:
        self.route_state = (route_state or "").strip().lower()
        self.waypoint_id = waypoint_id

    def update_planning_speed(self, speed: float, now: float) -> None:
        if math.isfinite(speed):
            self.planning_speed = max(0.0, float(speed))
            self.planning_speed_time = float(now)

    def update_vehicle_speed(self, speed: float, now: float) -> None:
        if math.isfinite(speed):
            self.vehicle_speed = abs(float(speed))
            self.vehicle_speed_time = float(now)

    def update_traffic_light(self, value: int, now: float) -> None:
        try:
            self.traffic_light = TrafficLight(value)
        except ValueError:
            self.traffic_light = TrafficLight.UNKNOWN
        self.traffic_light_time = float(now)

    def update_obstacle_distances(
        self, front_distance: float, merge_distance: float, now: float
    ) -> None:
        self.front_obstacle_distance = self._sanitize_distance(front_distance)
        self.merge_obstacle_distance = self._sanitize_distance(merge_distance)
        self.obstacle_time = float(now)

    def evaluate(self, now: float) -> DecisionOutput:
        now = float(now)
        planning_fresh = self._is_fresh(
            self.planning_speed_time, now, self.planning_speed_timeout
        )
        requested_speed = self.planning_speed if planning_fresh else 0.0
        light_fresh = self._is_fresh(
            self.traffic_light_time, now, self.traffic_light_timeout
        )
        speed_fresh = self._is_fresh(
            self.vehicle_speed_time, now, self.vehicle_speed_timeout
        )
        controlled_zone = self._is_controlled_zone()
        obstacle_fresh = self._is_fresh(
            self.obstacle_time, now, self.obstacle_timeout
        )

        stop_signal = light_fresh and (
            self.traffic_light == TrafficLight.RED
            or (self.treat_yellow_as_red and self.traffic_light == TrafficLight.YELLOW)
        )
        green_signal = light_fresh and self.traffic_light == TrafficLight.GREEN

        if self.state == DecisionState.CRUISE:
            if controlled_zone:
                self.state = DecisionState.APPROACHING_INTERSECTION

        if self.state == DecisionState.APPROACHING_INTERSECTION:
            if stop_signal:
                self.state = DecisionState.STOPPING_FOR_LIGHT
                self.latched_waypoint_id = self.waypoint_id
                self.green_since = None
            elif not controlled_zone:
                self.state = DecisionState.CRUISE

        elif self.state == DecisionState.STOPPING_FOR_LIGHT:
            if speed_fresh and self.vehicle_speed <= self.stopped_speed_threshold:
                self.state = DecisionState.WAITING_FOR_GREEN

        elif self.state == DecisionState.WAITING_FOR_GREEN:
            if green_signal:
                if self.green_since is None:
                    self.green_since = now
                if now - self.green_since >= self.green_release_delay:
                    self.state = DecisionState.RELEASED
                    self.latched_waypoint_id = None
            else:
                self.green_since = None

        elif self.state == DecisionState.RELEASED:
            if stop_signal and controlled_zone:
                self.state = DecisionState.STOPPING_FOR_LIGHT
                self.latched_waypoint_id = self.waypoint_id
                self.green_since = None
            elif not controlled_zone:
                self.state = DecisionState.CRUISE

        current_speed = self.vehicle_speed if speed_fresh else 0.0
        emergency_distance = max(
            self.emergency_min_distance,
            current_speed * self.emergency_reaction_time
            + current_speed * current_speed / (2.0 * self.emergency_deceleration),
        )
        if obstacle_fresh:
            if self.front_obstacle_distance <= emergency_distance:
                self.emergency_latched = True
            elif self.front_obstacle_distance >= (
                emergency_distance + self.emergency_release_margin
            ):
                self.emergency_latched = False

            if self.merge_obstacle_distance <= self.merge_trigger_distance:
                self.merge_latched = True
            elif self.merge_obstacle_distance >= (
                self.merge_trigger_distance + self.merge_release_margin
            ):
                self.merge_latched = False
        else:
            # A stale merge observation must not limit the car forever. Emergency
            # braking stays latched on stale data and therefore fails safe.
            self.merge_latched = False

        traffic_stop_required = self.state in (
            DecisionState.STOPPING_FOR_LIGHT,
            DecisionState.WAITING_FOR_GREEN,
        )
        if self.emergency_latched:
            target_speed = 0.0
            output_state = DecisionState.EMERGENCY_BRAKING
            stop_required = True
            reason = "front_obstacle_emergency"
        elif traffic_stop_required:
            target_speed = 0.0
            output_state = self.state
            stop_required = True
            reason = "traffic_light_stop_latched"
        elif self.merge_latched and planning_fresh:
            target_speed = min(float(requested_speed), self.merge_target_speed)
            output_state = DecisionState.SLOWING_FOR_MERGE
            stop_required = False
            reason = "merge_vehicle_slowdown"
        elif not planning_fresh:
            target_speed = 0.0
            output_state = self.state
            stop_required = False
            reason = "planning_speed_timeout"
        else:
            target_speed = float(requested_speed)
            output_state = self.state
            stop_required = False
            reason = "planning_speed"

        return DecisionOutput(target_speed, output_state, stop_required, reason)

    def _is_controlled_zone(self) -> bool:
        if self.route_state not in self.controlled_route_states:
            return False
        if not self.traffic_light_waypoint_ids:
            return True
        return self.waypoint_id in self.traffic_light_waypoint_ids

    @staticmethod
    def _is_fresh(timestamp: Optional[float], now: float, timeout: float) -> bool:
        return timestamp is not None and 0.0 <= now - timestamp <= timeout

    @staticmethod
    def _sanitize_distance(distance: float) -> float:
        value = float(distance)
        if math.isnan(value) or value < 0.0:
            return math.inf
        return value
