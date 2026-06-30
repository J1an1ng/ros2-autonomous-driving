#!/usr/bin/env python3

import math
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32, String
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_deg_to_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def distance_2d(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)


class RouteManagerNode(Node):
    def __init__(self):
        super().__init__("route_manager_node")

        self.declare_parameter("route_file", "")
        self.declare_parameter("map_frame", "world")
        self.declare_parameter("vehicle_frame", "OurCar/INS")
        self.declare_parameter("waypoint_reach_distance", 3.0)
        self.declare_parameter("lookahead_index", 1)

        self.route_file = self.get_parameter("route_file").value
        self.map_frame = self.get_parameter("map_frame").value
        self.vehicle_frame = self.get_parameter("vehicle_frame").value
        self.reach_distance = float(self.get_parameter("waypoint_reach_distance").value)
        self.lookahead_index = int(self.get_parameter("lookahead_index").value)

        self.waypoints = self.load_route_file(self.route_file)
        self.current_index = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pose_pub = self.create_publisher(PoseStamped, "/route/target_pose", 10)
        self.speed_limit_pub = self.create_publisher(Float32, "/route/speed_limit", 10)
        self.current_wp_pub = self.create_publisher(Int32, "/route/current_waypoint_id", 10)
        self.route_state_pub = self.create_publisher(String, "/route/route_state", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Route Manager started.")
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")
        self.get_logger().info(f"map_frame: {self.map_frame}")
        self.get_logger().info(f"vehicle_frame: {self.vehicle_frame}")

    def load_route_file(self, route_file):
        if not route_file:
            raise RuntimeError("Parameter route_file is empty.")

        path = Path(route_file)
        if not path.exists():
            raise RuntimeError(f"Route file does not exist: {route_file}")

        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        waypoints = data.get("waypoints", [])
        if not waypoints:
            raise RuntimeError("No waypoints found in route file.")

        return waypoints

    def get_vehicle_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.vehicle_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"TF not available: {ex}",
                throttle_duration_sec=2.0
            )
            return None

        t = transform.transform.translation
        return float(t.x), float(t.y)

    def update_current_waypoint(self, ego_x, ego_y):
        if self.current_index >= len(self.waypoints):
            self.current_index = len(self.waypoints) - 1
            return

        current_wp = self.waypoints[self.current_index]
        dist = distance_2d(
            ego_x,
            ego_y,
            float(current_wp["x"]),
            float(current_wp["y"])
        )

        if dist < self.reach_distance and self.current_index < len(self.waypoints) - 1:
            self.current_index += 1
            self.get_logger().info(
                f"Reached waypoint {current_wp['id']}, switching to waypoint {self.current_index}"
            )

    def get_target_waypoint(self):
        target_index = min(
            self.current_index + self.lookahead_index,
            len(self.waypoints) - 1
        )
        return self.waypoints[target_index], target_index

    def publish_target(self, waypoint, target_index):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame

        pose_msg.pose.position.x = float(waypoint["x"])
        pose_msg.pose.position.y = float(waypoint["y"])
        pose_msg.pose.position.z = float(waypoint.get("z", 0.0))

        yaw_deg = float(waypoint.get("yaw_deg", 0.0))
        pose_msg.pose.orientation = yaw_deg_to_quaternion(yaw_deg)

        speed_msg = Float32()
        speed_msg.data = float(waypoint.get("speed_limit", 2.0))

        id_msg = Int32()
        id_msg.data = int(waypoint.get("id", target_index))

        state_msg = String()
        state_msg.data = str(waypoint.get("type", "normal"))

        self.target_pose_pub.publish(pose_msg)
        self.speed_limit_pub.publish(speed_msg)
        self.current_wp_pub.publish(id_msg)
        self.route_state_pub.publish(state_msg)

    def timer_callback(self):
        ego_pose = self.get_vehicle_pose()
        if ego_pose is None:
            return

        ego_x, ego_y = ego_pose

        self.update_current_waypoint(ego_x, ego_y)

        target_wp, target_index = self.get_target_waypoint()
        self.publish_target(target_wp, target_index)


def main(args=None):
    rclpy.init(args=args)

    node = RouteManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Route Manager stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
