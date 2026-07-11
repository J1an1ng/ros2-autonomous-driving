"""ROS 2 adapter for the traffic-light decision state machine."""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, Int32, String, UInt8

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
        self.declare_parameter("treat_yellow_as_red", True)
        self.declare_parameter("obstacle_timeout", 0.5)
        self.declare_parameter("emergency_min_distance", 4.0)
        self.declare_parameter("emergency_reaction_time", 0.5)
        self.declare_parameter("emergency_deceleration", 5.0)
        self.declare_parameter("emergency_release_margin", 2.0)
        self.declare_parameter("merge_trigger_distance", 10.0)
        self.declare_parameter("merge_release_margin", 2.0)
        self.declare_parameter("merge_target_speed", 2.0)
        self.declare_parameter("update_rate", 20.0)

        controlled_states = self.get_parameter("controlled_route_states").value
        waypoint_ids = self.get_parameter_or(
            "traffic_light_waypoint_ids",
            Parameter(
                "traffic_light_waypoint_ids", Parameter.Type.INTEGER_ARRAY, []
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
            treat_yellow_as_red=self.get_parameter("treat_yellow_as_red").value,
            obstacle_timeout=self.get_parameter("obstacle_timeout").value,
            emergency_min_distance=self.get_parameter("emergency_min_distance").value,
            emergency_reaction_time=self.get_parameter("emergency_reaction_time").value,
            emergency_deceleration=self.get_parameter("emergency_deceleration").value,
            emergency_release_margin=self.get_parameter("emergency_release_margin").value,
            merge_trigger_distance=self.get_parameter("merge_trigger_distance").value,
            merge_release_margin=self.get_parameter("merge_release_margin").value,
            merge_target_speed=self.get_parameter("merge_target_speed").value,
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

    def planning_speed_callback(self, msg: Float32) -> None:
        self.machine.update_planning_speed(msg.data, self.now_seconds())

    def route_state_callback(self, msg: String) -> None:
        self.route_state = msg.data
        self.machine.update_route(self.route_state, self.waypoint_id)

    def waypoint_id_callback(self, msg: Int32) -> None:
        self.waypoint_id = msg.data
        self.machine.update_route(self.route_state, self.waypoint_id)

    def twist_callback(self, msg: TwistStamped) -> None:
        velocity = msg.twist.linear
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        self.machine.update_vehicle_speed(speed, self.now_seconds())

    def front_obstacle_callback(self, msg: Float32) -> None:
        self.front_obstacle_distance = msg.data
        self.machine.update_obstacle_distances(
            self.front_obstacle_distance,
            self.merge_obstacle_distance,
            self.now_seconds(),
        )

    def merge_obstacle_callback(self, msg: Float32) -> None:
        self.merge_obstacle_distance = msg.data
        self.machine.update_obstacle_distances(
            self.front_obstacle_distance,
            self.merge_obstacle_distance,
            self.now_seconds(),
        )

    def timer_callback(self) -> None:
        output = self.machine.evaluate(self.now_seconds())

        speed_msg = Float32()
        speed_msg.data = output.target_speed
        self.target_speed_pub.publish(speed_msg)

        state_msg = String()
        state_msg.data = output.state.name
        self.state_pub.publish(state_msg)

        stop_msg = Bool()
        stop_msg.data = output.stop_required
        self.stop_required_pub.publish(stop_msg)

        debug_msg = String()
        debug_msg.data = (
            f"state={output.state.name} reason={output.reason} "
            f"route_state={self.machine.route_state} waypoint_id={self.machine.waypoint_id} "
            f"light={self.machine.traffic_light.name} "
            f"front_distance={self.machine.front_obstacle_distance:.3f} "
            f"merge_distance={self.machine.merge_obstacle_distance:.3f} "
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
    finally:
        node.destroy_node()
        rclpy.shutdown()
