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
    FOLLOWING_VEHICLE = 7


@dataclass(frozen=True)
class DecisionOutput:
    target_speed: float
    state: DecisionState
    stop_required: bool
    reason: str


class TrafficLightStateMachine:
    """
    Select a safe target speed from route, planning and traffic-light inputs.

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
        strict_green_waypoint_ids: Sequence[int] = (),
        traffic_light_score_timeout: float = 0.75,
        strict_green_over_red_ratio: float = 1.05,
        strict_green_max_red_score: int = 15,
        strict_green_min_green_score: int = 0,
        strict_green_release_delay: float = 0.0,
        treat_yellow_as_red: bool = True,
        require_green_in_controlled_zone: bool = True,
        waypoint_distance_timeout: float = 0.5,
        unknown_light_stop_distance: float = 6.0,
        unknown_light_approach_speed: float = 1.0,
        traffic_light_approach_deceleration: float = 0.0,
        traffic_light_stop_distance: float = 4.0,
        stop_distance_override_waypoint_ids: Sequence[int] = (),
        stop_distance_overrides: Sequence[float] = (),
        traffic_light_activation_distance: float = 20.0,
        obstacle_timeout: float = 0.5,
        emergency_min_distance: float = 4.0,
        emergency_standstill_distance: float = 0.0,
        emergency_reaction_time: float = 0.5,
        emergency_deceleration: float = 5.0,
        emergency_release_margin: float = 2.0,
        emergency_min_hold_time: float = 0.0,
        emergency_trigger_max_distance: float = 0.0,
        emergency_follow_release_min_distance: float = 0.0,
        emergency_follow_release_min_range_rate: float = 0.0,
        emergency_follow_release_min_raw_range_rate: float = math.inf,
        emergency_follow_release_max_speed: float = 0.0,
        emergency_follow_release_confirmation: float = 0.0,
        emergency_follow_recovery_hold_time: float = 0.0,
        emergency_follow_recovery_acceleration: float = 1.0,
        merge_trigger_distance: float = 10.0,
        merge_release_margin: float = 2.0,
        merge_target_speed: float = 2.0,
        merge_stale_timeout: float = 3.0,
        following_enabled: bool = True,
        following_activation_distance: float = 30.0,
        following_time_headway: float = 1.5,
        following_distance_gain: float = 0.35,
        following_range_rate_alpha: float = 0.5,
        following_range_rate_timeout: float = 1.5,
        following_distance_deadband: float = 0.75,
        following_range_rate_deadband: float = 0.2,
        following_min_rolling_speed: float = 0.0,
        following_min_rolling_clearance: float = 0.0,
        following_emergency_closing_speed_margin: float = math.inf,
        following_target_acceleration: float = 1.0,
        following_target_deceleration: float = 2.0,
        following_max_abs_range_rate: float = 6.0,
        smooth_following_waypoint_ids: Sequence[int] = (),
        smooth_following_target_deceleration: float = 2.0,
        smooth_following_release_margin: float = 0.0,
        smooth_following_dropout_hold_time: float = 0.0,
        vehicle_behavior_waypoint_ids: Sequence[int] = (),
        merge_behavior_waypoint_ids: Sequence[int] = (),
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
        self.strict_green_waypoint_ids = {
            int(waypoint_id) for waypoint_id in strict_green_waypoint_ids
        }
        self.traffic_light_score_timeout = max(
            0.0, float(traffic_light_score_timeout)
        )
        self.strict_green_over_red_ratio = max(
            0.0, float(strict_green_over_red_ratio)
        )
        self.strict_green_max_red_score = max(0, int(strict_green_max_red_score))
        self.strict_green_min_green_score = max(
            0, int(strict_green_min_green_score)
        )
        self.strict_green_release_delay = max(
            self.green_release_delay, float(strict_green_release_delay)
        )
        self.treat_yellow_as_red = bool(treat_yellow_as_red)
        self.require_green_in_controlled_zone = bool(
            require_green_in_controlled_zone
        )
        self.waypoint_distance_timeout = max(0.0, float(waypoint_distance_timeout))
        self.unknown_light_stop_distance = max(
            0.0, float(unknown_light_stop_distance)
        )
        self.unknown_light_approach_speed = max(
            0.0, float(unknown_light_approach_speed)
        )
        self.traffic_light_approach_deceleration = max(
            0.0, float(traffic_light_approach_deceleration)
        )
        self.traffic_light_stop_distance = max(
            0.0, float(traffic_light_stop_distance)
        )
        if len(stop_distance_override_waypoint_ids) != len(
            stop_distance_overrides
        ):
            raise ValueError(
                'stop-distance waypoint IDs and overrides must have equal length'
            )
        self.stop_distance_overrides = {
            int(waypoint_id): max(0.0, float(distance))
            for waypoint_id, distance in zip(
                stop_distance_override_waypoint_ids,
                stop_distance_overrides,
            )
        }
        self.active_stop_distance = self.unknown_light_stop_distance
        self.traffic_light_activation_distance = max(
            self.unknown_light_stop_distance,
            self.traffic_light_stop_distance,
            max(self.stop_distance_overrides.values(), default=0.0),
            float(traffic_light_activation_distance),
        )
        self.obstacle_timeout = max(0.0, float(obstacle_timeout))
        self.emergency_min_distance = max(0.0, float(emergency_min_distance))
        self.emergency_standstill_distance = max(
            0.0, float(emergency_standstill_distance)
        )
        self.emergency_reaction_time = max(0.0, float(emergency_reaction_time))
        self.emergency_deceleration = max(1e-3, float(emergency_deceleration))
        self.emergency_release_margin = max(0.0, float(emergency_release_margin))
        self.emergency_min_hold_time = max(
            0.0, float(emergency_min_hold_time)
        )
        self.emergency_trigger_max_distance = max(
            0.0, float(emergency_trigger_max_distance)
        )
        self.emergency_follow_release_min_distance = max(
            0.0, float(emergency_follow_release_min_distance)
        )
        self.emergency_follow_release_min_range_rate = float(
            emergency_follow_release_min_range_rate
        )
        self.emergency_follow_release_min_raw_range_rate = float(
            emergency_follow_release_min_raw_range_rate
        )
        self.emergency_follow_release_max_speed = max(
            0.0, float(emergency_follow_release_max_speed)
        )
        self.emergency_follow_release_confirmation = max(
            0.0, float(emergency_follow_release_confirmation)
        )
        self.emergency_follow_recovery_hold_time = max(
            0.0, float(emergency_follow_recovery_hold_time)
        )
        self.emergency_follow_recovery_acceleration = max(
            0.0, float(emergency_follow_recovery_acceleration)
        )
        self.merge_trigger_distance = max(0.0, float(merge_trigger_distance))
        self.merge_release_margin = max(0.0, float(merge_release_margin))
        self.merge_target_speed = max(0.0, float(merge_target_speed))
        self.merge_stale_timeout = max(
            self.obstacle_timeout, float(merge_stale_timeout)
        )
        self.following_enabled = bool(following_enabled)
        self.following_activation_distance = max(
            0.0, float(following_activation_distance)
        )
        self.following_time_headway = max(
            0.0, float(following_time_headway)
        )
        self.following_distance_gain = max(
            0.0, float(following_distance_gain)
        )
        self.following_range_rate_alpha = min(
            1.0, max(0.0, float(following_range_rate_alpha))
        )
        self.following_range_rate_timeout = max(
            0.0, float(following_range_rate_timeout)
        )
        self.following_distance_deadband = max(
            0.0, float(following_distance_deadband)
        )
        self.following_range_rate_deadband = max(
            0.0, float(following_range_rate_deadband)
        )
        self.following_min_rolling_speed = max(
            0.0, float(following_min_rolling_speed)
        )
        self.following_min_rolling_clearance = max(
            0.0, float(following_min_rolling_clearance)
        )
        self.following_emergency_closing_speed_margin = max(
            0.0, float(following_emergency_closing_speed_margin)
        )
        self.following_target_acceleration = max(
            0.0, float(following_target_acceleration)
        )
        self.following_target_deceleration = max(
            0.0, float(following_target_deceleration)
        )
        self.following_max_abs_range_rate = max(
            0.0, float(following_max_abs_range_rate)
        )
        self.smooth_following_waypoint_ids = {
            int(waypoint_id) for waypoint_id in smooth_following_waypoint_ids
        }
        self.smooth_following_target_deceleration = max(
            0.0, float(smooth_following_target_deceleration)
        )
        self.smooth_following_release_margin = max(
            0.0, float(smooth_following_release_margin)
        )
        self.smooth_following_dropout_hold_time = max(
            0.0, float(smooth_following_dropout_hold_time)
        )
        self.vehicle_behavior_waypoint_ids = {
            int(waypoint_id) for waypoint_id in vehicle_behavior_waypoint_ids
        }
        self.merge_behavior_waypoint_ids = {
            int(waypoint_id) for waypoint_id in merge_behavior_waypoint_ids
        }

        self.state = DecisionState.CRUISE
        self.route_state = ''
        self.waypoint_id: Optional[int] = None
        self.waypoint_distance: Optional[float] = None
        self.waypoint_distance_time: Optional[float] = None
        self.planning_speed: Optional[float] = None
        self.planning_speed_time: Optional[float] = None
        self.vehicle_speed: Optional[float] = None
        self.vehicle_speed_time: Optional[float] = None
        self.traffic_light = TrafficLight.UNKNOWN
        self.traffic_light_time: Optional[float] = None
        self.traffic_light_red_score = 0
        self.traffic_light_green_score = 0
        self.traffic_light_score_time: Optional[float] = None
        self.strict_green_required = False
        self.strict_green_evidence = False
        self.green_since: Optional[float] = None
        self.latched_waypoint_id: Optional[int] = None
        self.signal_stop_latched = False
        self.signal_stop_pending = False
        self.front_obstacle_distance = math.inf
        self.merge_obstacle_distance = math.inf
        self.front_obstacle_time: Optional[float] = None
        self.merge_obstacle_time: Optional[float] = None
        self.front_obstacle_range_rate = 0.0
        self.front_obstacle_raw_range_rate = 0.0
        self.front_obstacle_raw_range_rate_time: Optional[float] = None
        self.effective_front_obstacle_range_rate = 0.0
        self.front_obstacle_range_rate_time: Optional[float] = None
        self._front_distance_sample = math.inf
        self._front_distance_sample_time: Optional[float] = None
        self.following_distance_target = self.emergency_standstill_distance
        self.following_raw_target_speed = 0.0
        self.following_target_speed = 0.0
        self._following_command_speed: Optional[float] = None
        self._following_command_time: Optional[float] = None
        self._following_last_observation_time: Optional[float] = None
        self._following_last_distance = math.inf
        self.emergency_distance = self.emergency_min_distance
        self.emergency_closing_speed = 0.0
        self.emergency_latched = False
        self.emergency_latched_time: Optional[float] = None
        self.emergency_follow_release_since: Optional[float] = None
        self.emergency_follow_recovery_until: Optional[float] = None
        self.merge_latched = False

    def update_route(self, route_state: str, waypoint_id: Optional[int] = None) -> None:
        self.route_state = (route_state or '').strip().lower()
        self.waypoint_id = waypoint_id

    def update_planning_speed(self, speed: float, now: float) -> None:
        if math.isfinite(speed):
            self.planning_speed = max(0.0, float(speed))
            self.planning_speed_time = float(now)

    def update_waypoint_distance(self, distance: float, now: float) -> None:
        if math.isfinite(distance) and distance >= 0.0:
            self.waypoint_distance = float(distance)
            self.waypoint_distance_time = float(now)

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

    def update_traffic_light_scores(
        self, red_score: int, green_score: int, now: float
    ) -> None:
        self.traffic_light_red_score = max(0, int(red_score))
        self.traffic_light_green_score = max(0, int(green_score))
        self.traffic_light_score_time = float(now)

    def update_obstacle_distances(
        self, front_distance: float, merge_distance: float, now: float
    ) -> None:
        self.update_front_obstacle_distance(front_distance, now)
        self.update_merge_obstacle_distance(merge_distance, now)

    def update_front_obstacle_distance(
        self, front_distance: float, now: float
    ) -> None:
        sanitized_front = self._sanitize_distance(front_distance)
        sample_changed = (
            math.isfinite(sanitized_front)
            and (
                not math.isfinite(self._front_distance_sample)
                or abs(sanitized_front - self._front_distance_sample) >= 0.02
            )
        )
        if sample_changed:
            if (
                math.isfinite(self._front_distance_sample)
                and self._front_distance_sample_time is not None
            ):
                sample_dt = float(now) - self._front_distance_sample_time
                if 0.15 <= sample_dt <= 2.5:
                    raw_range_rate = (
                        sanitized_front - self._front_distance_sample
                    ) / sample_dt
                    raw_range_rate = max(
                        -self.following_max_abs_range_rate,
                        min(
                            self.following_max_abs_range_rate,
                            raw_range_rate,
                        ),
                    )
                    self.front_obstacle_raw_range_rate = raw_range_rate
                    self.front_obstacle_raw_range_rate_time = float(now)
                    alpha = self.following_range_rate_alpha
                    if self.front_obstacle_range_rate_time is None:
                        self.front_obstacle_range_rate = raw_range_rate
                    else:
                        self.front_obstacle_range_rate = (
                            alpha * raw_range_rate
                            + (1.0 - alpha)
                            * self.front_obstacle_range_rate
                        )
                    self.front_obstacle_range_rate_time = float(now)
            self._front_distance_sample = sanitized_front
            self._front_distance_sample_time = float(now)
        elif not math.isfinite(sanitized_front):
            self._front_distance_sample = math.inf
            self._front_distance_sample_time = None
            self.front_obstacle_range_rate = 0.0
            self.front_obstacle_range_rate_time = None
            self.front_obstacle_raw_range_rate = 0.0
            self.front_obstacle_raw_range_rate_time = None

        self.front_obstacle_distance = sanitized_front
        self.front_obstacle_time = float(now)

    def update_merge_obstacle_distance(
        self, merge_distance: float, now: float
    ) -> None:
        self.merge_obstacle_distance = self._sanitize_distance(merge_distance)
        self.merge_obstacle_time = float(now)

    def evaluate(self, now: float) -> DecisionOutput:
        now = float(now)
        planning_fresh = self._is_fresh(
            self.planning_speed_time, now, self.planning_speed_timeout
        )
        requested_speed = self.planning_speed if planning_fresh else 0.0
        light_fresh = self._is_fresh(
            self.traffic_light_time, now, self.traffic_light_timeout
        )
        light_scores_fresh = self._is_fresh(
            self.traffic_light_score_time,
            now,
            self.traffic_light_score_timeout,
        )
        speed_fresh = self._is_fresh(
            self.vehicle_speed_time, now, self.vehicle_speed_timeout
        )
        controlled_zone = self._is_controlled_zone()
        waypoint_distance_fresh = self._is_fresh(
            self.waypoint_distance_time, now, self.waypoint_distance_timeout
        )
        front_obstacle_fresh = self._is_fresh(
            self.front_obstacle_time, now, self.obstacle_timeout
        )
        merge_obstacle_fresh = self._is_fresh(
            self.merge_obstacle_time, now, self.obstacle_timeout
        )

        raw_stop_signal = light_fresh and (
            self.traffic_light == TrafficLight.RED
            or (self.treat_yellow_as_red and self.traffic_light == TrafficLight.YELLOW)
        )
        associated_waypoint_id = (
            self.latched_waypoint_id
            if self.latched_waypoint_id is not None
            else self.waypoint_id
        )
        stop_distance_override = self.stop_distance_overrides.get(
            associated_waypoint_id
        )
        signal_stop_distance = (
            stop_distance_override
            if stop_distance_override is not None
            else self.traffic_light_stop_distance
        )
        unknown_stop_distance = (
            stop_distance_override
            if stop_distance_override is not None
            else self.unknown_light_stop_distance
        )
        self.active_stop_distance = (
            signal_stop_distance
            if self.signal_stop_pending
            else unknown_stop_distance
        )
        strict_green_required = (
            associated_waypoint_id in self.strict_green_waypoint_ids
        )
        strict_green_evidence = (
            light_scores_fresh
            and self.traffic_light_red_score <= self.strict_green_max_red_score
            and self.traffic_light_green_score >= self.strict_green_min_green_score
            and self.traffic_light_green_score
            >= self.strict_green_over_red_ratio
            * max(1, self.traffic_light_red_score)
        )
        self.strict_green_required = strict_green_required
        self.strict_green_evidence = strict_green_evidence
        raw_green_signal = (
            light_fresh
            and self.traffic_light == TrafficLight.GREEN
            and (not strict_green_required or strict_green_evidence)
        )
        light_action_allowed = (
            not waypoint_distance_fresh
            or self.waypoint_distance <= self.traffic_light_activation_distance
        )
        stop_signal = raw_stop_signal and light_action_allowed
        green_signal = raw_green_signal and light_action_allowed
        if (
            controlled_zone
            and self.state != DecisionState.RELEASED
            and stop_signal
        ):
            # Remember an associated red/yellow even if it disappears from the
            # camera before the vehicle reaches the stop boundary.
            self.signal_stop_pending = True
            self.latched_waypoint_id = self.waypoint_id
        signal_light_approach = (
            controlled_zone
            and self.signal_stop_pending
            and waypoint_distance_fresh
            and self.waypoint_distance > signal_stop_distance
        )
        unknown_light_approach = (
            controlled_zone
            and self.require_green_in_controlled_zone
            and not self.signal_stop_pending
            and not stop_signal
            and not green_signal
            and waypoint_distance_fresh
            and self.waypoint_distance > unknown_stop_distance
        )
        # Unknown/stale light data may only creep toward a controlled waypoint
        # while a fresh distance proves that the hard stop boundary is ahead.
        # Missing/stale distance therefore fails safe to a stop.
        light_requires_stop = (
            self.signal_stop_pending and not signal_light_approach
        ) or (
            controlled_zone
            and self.require_green_in_controlled_zone
            and not self.signal_stop_pending
            and not green_signal
            and not unknown_light_approach
        )

        if self.state == DecisionState.CRUISE:
            if controlled_zone:
                self.state = DecisionState.APPROACHING_INTERSECTION

        if self.state == DecisionState.APPROACHING_INTERSECTION:
            if green_signal and not self.signal_stop_pending:
                if self.green_since is None:
                    self.green_since = now
                if now - self.green_since >= self.green_release_delay:
                    self.state = DecisionState.RELEASED
                    self.latched_waypoint_id = None
                    self.signal_stop_latched = False
            elif light_requires_stop:
                self.state = DecisionState.STOPPING_FOR_LIGHT
                self.latched_waypoint_id = self.waypoint_id
                self.signal_stop_latched = self.signal_stop_pending
                self.green_since = None
            elif not controlled_zone:
                self.state = DecisionState.CRUISE
                self.green_since = None
            else:
                self.green_since = None

        elif self.state == DecisionState.STOPPING_FOR_LIGHT:
            if stop_signal:
                self.signal_stop_latched = True
            if green_signal:
                # Perception already debounces green. Once it is confirmed,
                # release any existing red/yellow brake latch immediately so
                # the vehicle does not wait for a needless full stop.
                self.state = DecisionState.RELEASED
                self.latched_waypoint_id = None
                self.signal_stop_latched = False
                self.signal_stop_pending = False
                self.green_since = None
            elif not self.signal_stop_latched and unknown_light_approach:
                # Route topics are independent ROS messages. If a fresh
                # distance shows the boundary is still ahead, cancel a
                # transient UNKNOWN stop caused by an old/missing sample.
                self.state = DecisionState.APPROACHING_INTERSECTION
                self.latched_waypoint_id = None
            elif speed_fresh and self.vehicle_speed <= self.stopped_speed_threshold:
                self.state = DecisionState.WAITING_FOR_GREEN

        elif self.state == DecisionState.WAITING_FOR_GREEN:
            if green_signal:
                if self.green_since is None:
                    self.green_since = now
                release_delay = (
                    self.strict_green_release_delay
                    if strict_green_required
                    else self.green_release_delay
                )
                if now - self.green_since >= release_delay:
                    self.state = DecisionState.RELEASED
                    self.latched_waypoint_id = None
                    self.signal_stop_latched = False
                    self.signal_stop_pending = False
            else:
                self.green_since = None

        elif self.state == DecisionState.RELEASED:
            # A confirmed green grants one passage through the current
            # intersection. The light normally leaves the camera field of view
            # before route_manager reaches the controlled waypoint; re-latching
            # on UNKNOWN here would stop the car inside the intersection.
            if not controlled_zone:
                self.state = DecisionState.CRUISE
                self.signal_stop_latched = False
                self.signal_stop_pending = False

        current_speed = self.vehicle_speed if speed_fresh else 0.0
        range_rate_fresh = self._is_fresh(
            self.front_obstacle_range_rate_time,
            now,
            self.following_range_rate_timeout,
        )
        range_rate = (
            self.front_obstacle_range_rate if range_rate_fresh else 0.0
        )
        raw_range_rate_fresh = self._is_fresh(
            self.front_obstacle_raw_range_rate_time,
            now,
            self.following_range_rate_timeout,
        )
        raw_opening_evidence = (
            raw_range_rate_fresh
            and self.front_obstacle_raw_range_rate
            >= self.emergency_follow_release_min_raw_range_rate
        )
        following_range_rate = (
            max(range_rate, self.front_obstacle_raw_range_rate)
            if raw_opening_evidence
            else range_rate
        )
        self.effective_front_obstacle_range_rate = range_rate
        closing_speed = max(0.0, -range_rate)
        rolling_clearance_available = (
            self.following_enabled
            and front_obstacle_fresh
            and speed_fresh
            and current_speed > self.stopped_speed_threshold
            and self.front_obstacle_distance
            >= (
                self.emergency_standstill_distance
                + self.following_min_rolling_clearance
            )
        )
        if rolling_clearance_available:
            # A same-direction lead vehicle cannot normally close the gap much
            # faster than ego is moving. Bound isolated RGB-D range jumps while
            # a safe surplus gap remains; below that boundary the unbounded
            # measured closing rate immediately regains authority.
            closing_speed = min(
                closing_speed,
                current_speed + self.following_emergency_closing_speed_margin,
            )
        self.emergency_closing_speed = closing_speed
        emergency_distance = max(
            self.emergency_min_distance,
            self.emergency_min_distance
            + closing_speed * self.emergency_reaction_time
            + closing_speed * closing_speed
            / (2.0 * self.emergency_deceleration),
        )
        if self.emergency_trigger_max_distance > 0.0:
            emergency_distance = min(
                emergency_distance,
                self.emergency_trigger_max_distance,
            )
        self.emergency_distance = emergency_distance
        vehicle_behavior_allowed = (
            not self.vehicle_behavior_waypoint_ids
            or self.waypoint_id in self.vehicle_behavior_waypoint_ids
        )
        merge_behavior_allowed = (
            not self.merge_behavior_waypoint_ids
            or self.waypoint_id in self.merge_behavior_waypoint_ids
        )
        merge_obstacle_stale_stop = False
        emergency_follow_handoff = False
        if front_obstacle_fresh:
            if (
                vehicle_behavior_allowed
                and self.front_obstacle_distance <= emergency_distance
            ):
                if not self.emergency_latched:
                    self.emergency_latched_time = now
                self.emergency_latched = True
                self.emergency_follow_recovery_until = None
            if self.emergency_latched:
                hold_complete = (
                    self.emergency_latched_time is None
                    or now - self.emergency_latched_time
                    >= self.emergency_min_hold_time
                )
                vehicle_stopped = (
                    speed_fresh
                    and self.vehicle_speed <= self.stopped_speed_threshold
                )
                release_distance = max(
                    self.emergency_min_distance,
                    self.emergency_standstill_distance,
                ) + self.emergency_release_margin
                obstacle_clear = (
                    not math.isfinite(self.front_obstacle_distance)
                    or self.front_obstacle_distance >= release_distance
                )
                follow_release_candidate = (
                    range_rate_fresh
                    and speed_fresh
                    and self.emergency_follow_release_max_speed > 0.0
                    and current_speed
                    <= self.emergency_follow_release_max_speed
                    and self.front_obstacle_distance
                    >= self.emergency_follow_release_min_distance
                    and (
                        range_rate
                        >= self.emergency_follow_release_min_range_rate
                        or raw_opening_evidence
                    )
                )
                if follow_release_candidate:
                    if self.emergency_follow_release_since is None:
                        self.emergency_follow_release_since = now
                else:
                    self.emergency_follow_release_since = None
                follow_release_confirmed = (
                    self.emergency_follow_release_since is not None
                    and now - self.emergency_follow_release_since
                    >= self.emergency_follow_release_confirmation
                )
                # Do not release merely because braking reduced speed and made
                # the dynamic stopping threshold shrink. That caused Event2 to
                # alternate full brake, throttle, then full brake again while
                # the ego vehicle was still closing. A confirmed non-closing
                # lead vehicle may, however, hand back to adaptive following
                # without forcing ego all the way to zero.
                if hold_complete and (
                    (vehicle_stopped and obstacle_clear)
                    or follow_release_confirmed
                ):
                    emergency_follow_handoff = True
                    self.emergency_follow_recovery_until = (
                        now + self.emergency_follow_recovery_hold_time
                    )
                    self.emergency_latched = False
                    self.emergency_latched_time = None
                    self.emergency_follow_release_since = None

        else:
            self.emergency_follow_release_since = None

        if merge_obstacle_fresh:
            if (
                merge_behavior_allowed
                and self.merge_obstacle_distance <= self.merge_trigger_distance
            ):
                self.merge_latched = True
            elif (
                not merge_behavior_allowed
                or self.merge_obstacle_distance
                >= self.merge_trigger_distance + self.merge_release_margin
            ):
                self.merge_latched = False
        else:
            obstacle_age = (
                now - self.merge_obstacle_time
                if self.merge_obstacle_time is not None
                else math.inf
            )
            merge_obstacle_stale_stop = (
                self.merge_latched
                and not 0.0 <= obstacle_age <= self.merge_stale_timeout
            )

        traffic_stop_required = self.state in (
            DecisionState.STOPPING_FOR_LIGHT,
            DecisionState.WAITING_FOR_GREEN,
        )
        permitted_speed = float(requested_speed)
        if (unknown_light_approach or signal_light_approach) and planning_fresh:
            approach_speed_limit = self.unknown_light_approach_speed
            if self.traffic_light_approach_deceleration > 0.0:
                stop_distance = (
                    signal_stop_distance
                    if signal_light_approach
                    else unknown_stop_distance
                )
                remaining_braking_distance = max(
                    0.0, self.waypoint_distance - stop_distance
                )
                braking_envelope_speed = math.sqrt(
                    2.0
                    * self.traffic_light_approach_deceleration
                    * remaining_braking_distance
                )
                approach_speed_limit = min(
                    approach_speed_limit, braking_envelope_speed
                )
            permitted_speed = min(
                permitted_speed, approach_speed_limit
            )
        following_candidate = (
            self.following_enabled
            and vehicle_behavior_allowed
            and front_obstacle_fresh
            and math.isfinite(self.front_obstacle_distance)
            and planning_fresh
        )
        smooth_following = (
            self.waypoint_id in self.smooth_following_waypoint_ids
        )
        following_observed = (
            following_candidate
            and self.front_obstacle_distance
            <= self.following_activation_distance
        )
        following_release_observed = (
            following_candidate
            and smooth_following
            and self._following_command_speed is not None
            and self.front_obstacle_distance
            <= (
                self.following_activation_distance
                + self.smooth_following_release_margin
            )
        )
        if following_observed or following_release_observed:
            self._following_last_observation_time = now
            self._following_last_distance = self.front_obstacle_distance
        following_dropout_held = (
            not following_observed
            and not following_release_observed
            and smooth_following
            and vehicle_behavior_allowed
            and planning_fresh
            and self._following_command_speed is not None
            and self._following_last_observation_time is not None
            and 0.0
            <= now - self._following_last_observation_time
            <= self.smooth_following_dropout_hold_time
        )
        following_active = (
            following_observed
            or following_release_observed
            or following_dropout_held
        )
        if following_active:
            following_distance = (
                self._following_last_distance
                if following_dropout_held
                else self.front_obstacle_distance
            )
            following_deceleration = (
                self.smooth_following_target_deceleration
                if smooth_following
                else self.following_target_deceleration
            )
            self.following_distance_target = (
                self.emergency_standstill_distance
                + self.following_time_headway * current_speed
            )
            distance_error = (
                following_distance
                - self.following_distance_target
            )
            if (
                abs(distance_error) <= self.following_distance_deadband
                and abs(following_range_rate)
                <= self.following_range_rate_deadband
            ):
                # Preserve speed when both gap and relative motion are already
                # inside the hold band.
                raw_following_target = current_speed
            else:
                estimated_lead_speed = max(
                    0.0, current_speed + following_range_rate
                )
                raw_following_target = (
                    estimated_lead_speed
                    + self.following_distance_gain * distance_error
                )
                if (
                    following_range_rate
                    <= self.following_range_rate_deadband
                ):
                    # Do not accelerate toward a lead vehicle while the gap is
                    # stable or closing. Acceleration is permitted again once
                    # the measured gap is clearly opening.
                    raw_following_target = min(
                        raw_following_target, current_speed
                    )
            self.following_raw_target_speed = max(
                0.0, min(permitted_speed, raw_following_target)
            )
            rolling_speed_floor_available = (
                not self.emergency_latched
                and current_speed > self.stopped_speed_threshold
                and following_distance
                >= (
                    self.emergency_standstill_distance
                    + self.following_min_rolling_clearance
                )
            )
            if (
                rolling_speed_floor_available
                and self.following_min_rolling_speed > 0.0
            ):
                # Event2's lead vehicle brakes hard and then moves again. Keep
                # a small crawl command while a safe surplus gap remains so
                # adaptive following does not create an unnecessary standstill
                # before the near-field emergency guard is reached. Never
                # accelerate to reach this floor while the gap is closing.
                rolling_target = min(
                    permitted_speed,
                    current_speed,
                    self.following_min_rolling_speed,
                )
                self.following_raw_target_speed = max(
                    self.following_raw_target_speed,
                    rolling_target,
                )
            if emergency_follow_handoff:
                # Do not inherit the stale low target computed during full
                # braking. The release guards have confirmed safe recovery, so
                # immediately accept an opening-gap target when it is higher.
                following_target = max(
                    current_speed, self.following_raw_target_speed
                )
                self._following_command_speed = following_target
                self._following_command_time = now
            elif (
                self.emergency_follow_recovery_until is not None
                and now <= self.emergency_follow_recovery_until
            ):
                recovery_target = max(
                    current_speed, self.following_raw_target_speed
                )
                following_target = self._limit_following_target(
                    recovery_target,
                    current_speed,
                    now,
                    acceleration=self.emergency_follow_recovery_acceleration,
                    deceleration=following_deceleration,
                    maximum_target=permitted_speed,
                    seed_from_current=smooth_following,
                )
            else:
                self.emergency_follow_recovery_until = None
                following_target = self._limit_following_target(
                    self.following_raw_target_speed,
                    current_speed,
                    now,
                    deceleration=following_deceleration,
                    maximum_target=permitted_speed,
                    seed_from_current=smooth_following,
                )
            self.following_target_speed = following_target
        else:
            self._following_command_speed = None
            self._following_command_time = None
            self._following_last_observation_time = None
            self._following_last_distance = math.inf
            self.following_raw_target_speed = permitted_speed
            self.following_target_speed = permitted_speed

        if self.emergency_latched:
            target_speed = 0.0
            output_state = DecisionState.EMERGENCY_BRAKING
            stop_required = True
            reason = 'front_obstacle_emergency'
        elif traffic_stop_required:
            target_speed = 0.0
            output_state = self.state
            stop_required = True
            reason = (
                'strict_green_evidence_rejected'
                if (
                    light_fresh
                    and self.traffic_light == TrafficLight.GREEN
                    and strict_green_required
                    and not strict_green_evidence
                )
                else 'traffic_light_stop_latched'
            )
        elif merge_obstacle_stale_stop:
            target_speed = 0.0
            output_state = DecisionState.SLOWING_FOR_MERGE
            stop_required = True
            reason = 'merge_obstacle_stale_stop'
        elif following_active:
            target_speed = following_target
            output_state = DecisionState.FOLLOWING_VEHICLE
            stop_required = False
            reason = (
                'adaptive_vehicle_following_hold'
                if following_dropout_held
                else 'adaptive_vehicle_following'
            )
        elif self.merge_latched and planning_fresh:
            target_speed = min(permitted_speed, self.merge_target_speed)
            output_state = DecisionState.SLOWING_FOR_MERGE
            stop_required = False
            reason = 'merge_vehicle_slowdown'
        elif not planning_fresh:
            target_speed = 0.0
            output_state = self.state
            stop_required = False
            reason = 'planning_speed_timeout'
        else:
            target_speed = permitted_speed
            output_state = self.state
            stop_required = False
            reason = (
                'traffic_light_controlled_approach'
                if signal_light_approach
                else 'unknown_light_controlled_approach'
                if unknown_light_approach
                else 'planning_speed'
            )

        return DecisionOutput(target_speed, output_state, stop_required, reason)

    def _limit_following_target(
        self,
        raw_target: float,
        current_speed: float,
        now: float,
        acceleration: Optional[float] = None,
        deceleration: Optional[float] = None,
        maximum_target: float = math.inf,
        seed_from_current: bool = False,
    ) -> float:
        acceleration_limit = (
            self.following_target_acceleration
            if acceleration is None
            else max(0.0, float(acceleration))
        )
        deceleration_limit = (
            self.following_target_deceleration
            if deceleration is None
            else max(0.0, float(deceleration))
        )
        if (
            self._following_command_speed is None
            or self._following_command_time is None
        ):
            if seed_from_current:
                # In smooth-following route windows, a newly associated,
                # still-far lead vehicle must not cause an instantaneous
                # target-speed step. Near-field risk bypasses this path through
                # the independent emergency latch above.
                command = max(0.0, float(current_speed))
            else:
                command = min(
                    float(raw_target), max(0.0, float(current_speed))
                )
        else:
            dt = max(0.0, float(now) - self._following_command_time)
            lower = (
                self._following_command_speed
                - deceleration_limit * dt
            )
            upper = (
                self._following_command_speed
                + acceleration_limit * dt
            )
            command = max(lower, min(upper, float(raw_target)))
        self._following_command_speed = max(
            0.0, min(float(maximum_target), command)
        )
        self._following_command_time = float(now)
        return self._following_command_speed

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
