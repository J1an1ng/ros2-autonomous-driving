"""ROS 2 adapter for the traffic-light decision state machine."""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, Int32, Int32MultiArray, String, UInt8

from decision_making.traffic_light_state_machine import TrafficLightStateMachine


class TrafficLightStateMachineNode(Node):
    def __init__(self) -> None:
        super().__init__("traffic_light_state_machine_node")

        self.declare_parameter(
            "controlled_route_states",
            ["intersection_approach", "stop_zone", "intersection"],
        )
        self.declare_parameter(
            "traffic_light_waypoint_ids", Parameter.Type.INTEGER_ARRAY
        )
        self.declare_parameter("stopped_speed_threshold", 0.15)
        self.declare_parameter("traffic_light_timeout", 0.75)
        self.declare_parameter("planning_speed_timeout", 0.5)
        self.declare_parameter("vehicle_speed_timeout", 0.5)
        self.declare_parameter("green_release_delay", 0.3)
        self.declare_parameter(
            "strict_green_waypoint_ids", Parameter.Type.INTEGER_ARRAY
        )
        self.declare_parameter("traffic_light_score_timeout", 0.75)
        self.declare_parameter("strict_green_over_red_ratio", 1.05)
        self.declare_parameter("strict_green_max_red_score", 15)
        self.declare_parameter("strict_green_min_green_score", 0)
        self.declare_parameter("strict_green_release_delay", 0.0)
        self.declare_parameter("treat_yellow_as_red", True)
        self.declare_parameter("require_green_in_controlled_zone", True)
        self.declare_parameter("waypoint_distance_timeout", 0.5)
        self.declare_parameter("unknown_light_stop_distance", 6.0)
        self.declare_parameter("unknown_light_approach_speed", 1.0)
        self.declare_parameter("traffic_light_approach_deceleration", 0.0)
        self.declare_parameter("traffic_light_stop_distance", 4.0)
        self.declare_parameter(
            "stop_distance_override_waypoint_ids",
            Parameter.Type.INTEGER_ARRAY,
        )
        self.declare_parameter(
            "stop_distance_overrides",
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter("traffic_light_activation_distance", 20.0)
        self.declare_parameter("obstacle_timeout", 0.5)
        self.declare_parameter("emergency_min_distance", 4.0)
        self.declare_parameter("emergency_standstill_distance", 0.0)
        self.declare_parameter("emergency_reaction_time", 0.5)
        self.declare_parameter("emergency_deceleration", 5.0)
        self.declare_parameter("emergency_release_margin", 2.0)
        self.declare_parameter("emergency_min_hold_time", 2.5)
        self.declare_parameter("emergency_trigger_max_distance", 0.0)
        self.declare_parameter("emergency_follow_release_min_distance", 0.0)
        self.declare_parameter("emergency_follow_release_min_range_rate", 0.0)
        self.declare_parameter(
            "emergency_follow_release_min_raw_range_rate", math.inf
        )
        self.declare_parameter("emergency_follow_release_max_speed", 0.0)
        self.declare_parameter("emergency_follow_release_confirmation", 0.0)
        self.declare_parameter("emergency_follow_recovery_hold_time", 0.0)
        self.declare_parameter("emergency_follow_recovery_acceleration", 1.0)
        self.declare_parameter("merge_trigger_distance", 12.0)
        self.declare_parameter("merge_release_margin", 2.0)
        self.declare_parameter("merge_target_speed", 1.5)
        self.declare_parameter("merge_stale_timeout", 3.0)
        self.declare_parameter("following_enabled", True)
        self.declare_parameter("following_activation_distance", 30.0)
        self.declare_parameter("following_time_headway", 1.5)
        self.declare_parameter("following_distance_gain", 0.35)
        self.declare_parameter("following_range_rate_alpha", 0.5)
        self.declare_parameter("following_range_rate_timeout", 1.5)
        self.declare_parameter("following_distance_deadband", 0.75)
        self.declare_parameter("following_range_rate_deadband", 0.2)
        self.declare_parameter("following_min_rolling_speed", 0.0)
        self.declare_parameter("following_min_rolling_clearance", 0.0)
        self.declare_parameter(
            "following_emergency_closing_speed_margin", math.inf
        )
        self.declare_parameter("following_target_acceleration", 1.0)
        self.declare_parameter("following_target_deceleration", 2.0)
        self.declare_parameter("following_max_abs_range_rate", 6.0)
        self.declare_parameter(
            "smooth_following_waypoint_ids", Parameter.Type.INTEGER_ARRAY
        )
        self.declare_parameter("smooth_following_target_deceleration", 2.0)
        self.declare_parameter("smooth_following_release_margin", 0.0)
        self.declare_parameter("smooth_following_dropout_hold_time", 0.0)
        self.declare_parameter(
            "vehicle_behavior_waypoint_ids", Parameter.Type.INTEGER_ARRAY
        )
        self.declare_parameter("vehicle_behavior_global", False)
        self.declare_parameter(
            "merge_behavior_waypoint_ids", Parameter.Type.INTEGER_ARRAY
        )
        self.declare_parameter("update_rate", 20.0)

        controlled_states = self.get_parameter("controlled_route_states").value
        waypoint_ids = self.get_parameter_or(
            "traffic_light_waypoint_ids",
            Parameter(
                "traffic_light_waypoint_ids", Parameter.Type.INTEGER_ARRAY, []
            ),
        ).value
        strict_green_waypoint_ids = self.get_parameter_or(
            "strict_green_waypoint_ids",
            Parameter(
                "strict_green_waypoint_ids", Parameter.Type.INTEGER_ARRAY, []
            ),
        ).value
        vehicle_behavior_waypoint_ids = self.get_parameter_or(
            "vehicle_behavior_waypoint_ids",
            Parameter(
                "vehicle_behavior_waypoint_ids",
                Parameter.Type.INTEGER_ARRAY,
                [],
            ),
        ).value
        if bool(self.get_parameter("vehicle_behavior_global").value):
            vehicle_behavior_waypoint_ids = []
        merge_behavior_waypoint_ids = self.get_parameter_or(
            "merge_behavior_waypoint_ids",
            Parameter(
                "merge_behavior_waypoint_ids",
                Parameter.Type.INTEGER_ARRAY,
                [],
            ),
        ).value
        smooth_following_waypoint_ids = self.get_parameter_or(
            "smooth_following_waypoint_ids",
            Parameter(
                "smooth_following_waypoint_ids",
                Parameter.Type.INTEGER_ARRAY,
                [],
            ),
        ).value
        stop_distance_override_waypoint_ids = self.get_parameter_or(
            "stop_distance_override_waypoint_ids",
            Parameter(
                "stop_distance_override_waypoint_ids",
                Parameter.Type.INTEGER_ARRAY,
                [],
            ),
        ).value
        stop_distance_overrides = self.get_parameter_or(
            "stop_distance_overrides",
            Parameter(
                "stop_distance_overrides",
                Parameter.Type.DOUBLE_ARRAY,
                [],
            ),
        ).value
        self.machine = TrafficLightStateMachine(
            controlled_route_states=controlled_states,
            traffic_light_waypoint_ids=waypoint_ids,
            stopped_speed_threshold=self.get_parameter("stopped_speed_threshold").value,
            traffic_light_timeout=self.get_parameter("traffic_light_timeout").value,
            planning_speed_timeout=self.get_parameter("planning_speed_timeout").value,
            vehicle_speed_timeout=self.get_parameter("vehicle_speed_timeout").value,
            green_release_delay=self.get_parameter("green_release_delay").value,
            strict_green_waypoint_ids=strict_green_waypoint_ids,
            traffic_light_score_timeout=self.get_parameter(
                "traffic_light_score_timeout"
            ).value,
            strict_green_over_red_ratio=self.get_parameter(
                "strict_green_over_red_ratio"
            ).value,
            strict_green_max_red_score=self.get_parameter(
                "strict_green_max_red_score"
            ).value,
            strict_green_min_green_score=self.get_parameter(
                "strict_green_min_green_score"
            ).value,
            strict_green_release_delay=self.get_parameter(
                "strict_green_release_delay"
            ).value,
            treat_yellow_as_red=self.get_parameter("treat_yellow_as_red").value,
            require_green_in_controlled_zone=self.get_parameter(
                "require_green_in_controlled_zone"
            ).value,
            waypoint_distance_timeout=self.get_parameter(
                "waypoint_distance_timeout"
            ).value,
            unknown_light_stop_distance=self.get_parameter(
                "unknown_light_stop_distance"
            ).value,
            unknown_light_approach_speed=self.get_parameter(
                "unknown_light_approach_speed"
            ).value,
            traffic_light_approach_deceleration=self.get_parameter(
                "traffic_light_approach_deceleration"
            ).value,
            traffic_light_stop_distance=self.get_parameter(
                "traffic_light_stop_distance"
            ).value,
            stop_distance_override_waypoint_ids=(
                stop_distance_override_waypoint_ids
            ),
            stop_distance_overrides=stop_distance_overrides,
            traffic_light_activation_distance=self.get_parameter(
                "traffic_light_activation_distance"
            ).value,
            obstacle_timeout=self.get_parameter("obstacle_timeout").value,
            emergency_min_distance=self.get_parameter("emergency_min_distance").value,
            emergency_standstill_distance=self.get_parameter(
                "emergency_standstill_distance"
            ).value,
            emergency_reaction_time=self.get_parameter("emergency_reaction_time").value,
            emergency_deceleration=self.get_parameter("emergency_deceleration").value,
            emergency_release_margin=self.get_parameter("emergency_release_margin").value,
            emergency_min_hold_time=self.get_parameter(
                "emergency_min_hold_time"
            ).value,
            emergency_trigger_max_distance=self.get_parameter(
                "emergency_trigger_max_distance"
            ).value,
            emergency_follow_release_min_distance=self.get_parameter(
                "emergency_follow_release_min_distance"
            ).value,
            emergency_follow_release_min_range_rate=self.get_parameter(
                "emergency_follow_release_min_range_rate"
            ).value,
            emergency_follow_release_min_raw_range_rate=self.get_parameter(
                "emergency_follow_release_min_raw_range_rate"
            ).value,
            emergency_follow_release_max_speed=self.get_parameter(
                "emergency_follow_release_max_speed"
            ).value,
            emergency_follow_release_confirmation=self.get_parameter(
                "emergency_follow_release_confirmation"
            ).value,
            emergency_follow_recovery_hold_time=self.get_parameter(
                "emergency_follow_recovery_hold_time"
            ).value,
            emergency_follow_recovery_acceleration=self.get_parameter(
                "emergency_follow_recovery_acceleration"
            ).value,
            merge_trigger_distance=self.get_parameter("merge_trigger_distance").value,
            merge_release_margin=self.get_parameter("merge_release_margin").value,
            merge_target_speed=self.get_parameter("merge_target_speed").value,
            merge_stale_timeout=self.get_parameter("merge_stale_timeout").value,
            following_enabled=self.get_parameter("following_enabled").value,
            following_activation_distance=self.get_parameter(
                "following_activation_distance"
            ).value,
            following_time_headway=self.get_parameter(
                "following_time_headway"
            ).value,
            following_distance_gain=self.get_parameter(
                "following_distance_gain"
            ).value,
            following_range_rate_alpha=self.get_parameter(
                "following_range_rate_alpha"
            ).value,
            following_range_rate_timeout=self.get_parameter(
                "following_range_rate_timeout"
            ).value,
            following_distance_deadband=self.get_parameter(
                "following_distance_deadband"
            ).value,
            following_range_rate_deadband=self.get_parameter(
                "following_range_rate_deadband"
            ).value,
            following_min_rolling_speed=self.get_parameter(
                "following_min_rolling_speed"
            ).value,
            following_min_rolling_clearance=self.get_parameter(
                "following_min_rolling_clearance"
            ).value,
            following_emergency_closing_speed_margin=self.get_parameter(
                "following_emergency_closing_speed_margin"
            ).value,
            following_target_acceleration=self.get_parameter(
                "following_target_acceleration"
            ).value,
            following_target_deceleration=self.get_parameter(
                "following_target_deceleration"
            ).value,
            following_max_abs_range_rate=self.get_parameter(
                "following_max_abs_range_rate"
            ).value,
            smooth_following_waypoint_ids=smooth_following_waypoint_ids,
            smooth_following_target_deceleration=self.get_parameter(
                "smooth_following_target_deceleration"
            ).value,
            smooth_following_release_margin=self.get_parameter(
                "smooth_following_release_margin"
            ).value,
            smooth_following_dropout_hold_time=self.get_parameter(
                "smooth_following_dropout_hold_time"
            ).value,
            vehicle_behavior_waypoint_ids=vehicle_behavior_waypoint_ids,
            merge_behavior_waypoint_ids=merge_behavior_waypoint_ids,
        )

        self.route_state = ""
        self.waypoint_id = None
        self.front_obstacle_distance = math.inf
        self.merge_obstacle_distance = math.inf

        self.create_subscription(
            UInt8,
            "/perception/traffic_light_state",
            self.traffic_light_callback,
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            "/perception/traffic_light_scores",
            self.traffic_light_scores_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/planning/target_speed",
            self.planning_speed_callback,
            10,
        )
        self.create_subscription(
            String,
            "/route/route_state",
            self.route_state_callback,
            10,
        )
        self.create_subscription(
            Int32,
            "/route/current_waypoint_id",
            self.waypoint_id_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/route/current_waypoint_distance",
            self.waypoint_distance_callback,
            10,
        )
        self.create_subscription(
            TwistStamped,
            "/OurCar/INS/twist",
            self.twist_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/decision/front_obstacle_distance",
            self.front_obstacle_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/decision/merge_obstacle_distance",
            self.merge_obstacle_callback,
            10,
        )

        self.target_speed_pub = self.create_publisher(
            Float32, "/decision/target_speed", 10
        )
        self.state_pub = self.create_publisher(String, "/decision/state", 10)
        self.stop_required_pub = self.create_publisher(
            Bool, "/decision/stop_required", 10
        )
        self.debug_pub = self.create_publisher(String, "/decision/debug", 10)

        update_rate = max(float(self.get_parameter("update_rate").value), 1e-3)
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)
        self.last_reported_state = None
        self.get_logger().info("Traffic-light decision state machine started")

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def traffic_light_callback(self, msg: UInt8) -> None:
        self.machine.update_traffic_light(msg.data, self.now_seconds())

    def traffic_light_scores_callback(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warning(
                "Ignoring traffic-light score message with fewer than two values"
            )
            return
        self.machine.update_traffic_light_scores(
            msg.data[0], msg.data[1], self.now_seconds()
        )

    def planning_speed_callback(self, msg: Float32) -> None:
        self.machine.update_planning_speed(msg.data, self.now_seconds())

    def route_state_callback(self, msg: String) -> None:
        self.route_state = msg.data
        self.machine.update_route(self.route_state, self.waypoint_id)

    def waypoint_id_callback(self, msg: Int32) -> None:
        self.waypoint_id = msg.data
        self.machine.update_route(self.route_state, self.waypoint_id)

    def waypoint_distance_callback(self, msg: Float32) -> None:
        self.machine.update_waypoint_distance(msg.data, self.now_seconds())

    def twist_callback(self, msg: TwistStamped) -> None:
        velocity = msg.twist.linear
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        self.machine.update_vehicle_speed(speed, self.now_seconds())

    def front_obstacle_callback(self, msg: Float32) -> None:
        self.front_obstacle_distance = msg.data
        self.machine.update_front_obstacle_distance(
            self.front_obstacle_distance, self.now_seconds()
        )

    def merge_obstacle_callback(self, msg: Float32) -> None:
        self.merge_obstacle_distance = msg.data
        self.machine.update_merge_obstacle_distance(
            self.merge_obstacle_distance, self.now_seconds()
        )

    def timer_callback(self) -> None:
        now = self.now_seconds()
        output = self.machine.evaluate(now)
        front_obstacle_age = (
            now - self.machine.front_obstacle_time
            if self.machine.front_obstacle_time is not None
            else math.inf
        )
        merge_obstacle_age = (
            now - self.machine.merge_obstacle_time
            if self.machine.merge_obstacle_time is not None
            else math.inf
        )

        speed_msg = Float32()
        speed_msg.data = output.target_speed
        self.target_speed_pub.publish(speed_msg)

        state_msg = String()
        state_msg.data = output.state.name
        self.state_pub.publish(state_msg)

        stop_msg = Bool()
        stop_msg.data = output.stop_required
        self.stop_required_pub.publish(stop_msg)

        ego_speed = (
            self.machine.vehicle_speed
            if self.machine.vehicle_speed is not None
            else math.nan
        )
        debug_msg = String()
        debug_msg.data = (
            f"state={output.state.name} reason={output.reason} "
            f"route_state={self.machine.route_state} waypoint_id={self.machine.waypoint_id} "
            f"waypoint_distance={self.machine.waypoint_distance} "
            f"active_stop_distance={self.machine.active_stop_distance:.3f} "
            f"light={self.machine.traffic_light.name} "
            f"light_scores={self.machine.traffic_light_red_score}/"
            f"{self.machine.traffic_light_green_score} "
            f"strict_green={self.machine.strict_green_required}/"
            f"{self.machine.strict_green_evidence} "
            f"emergency_threshold={self.machine.emergency_distance:.3f} "
            f"emergency_closing_speed="
            f"{self.machine.emergency_closing_speed:.3f} "
            f"ego_speed={ego_speed:.3f} "
            f"range_rate="
            f"{self.machine.effective_front_obstacle_range_rate:.3f} "
            f"raw_range_rate="
            f"{self.machine.front_obstacle_raw_range_rate:.3f} "
            f"following_gap={self.machine.following_distance_target:.3f} "
            f"following_raw_target="
            f"{self.machine.following_raw_target_speed:.3f} "
            f"front_distance={self.machine.front_obstacle_distance:.3f} "
            f"merge_distance={self.machine.merge_obstacle_distance:.3f} "
            f"front_obstacle_age={front_obstacle_age:.3f} "
            f"merge_obstacle_age={merge_obstacle_age:.3f} "
            f"emergency_latched={self.machine.emergency_latched} "
            f"merge_latched={self.machine.merge_latched} "
            f"target_speed={output.target_speed:.3f}"
        )
        self.debug_pub.publish(debug_msg)

        if output.state != self.last_reported_state:
            self.get_logger().info(debug_msg.data)
            self.last_reported_state = output.state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficLightStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
