#!/usr/bin/env python3

import math
from pathlib import Path as FilePath

import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Int32, Float32, String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path as NavPath
from tf2_ros import Buffer, TransformListener, TransformException

from route_manager.route_progress import distance_2d, waypoint_reached_or_passed


def yaw_deg_to_quaternion(yaw_deg):
    yaw = math.radians(float(yaw_deg))

    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)

    return q


class RouteManagerNode(Node):

    def __init__(self):
        super().__init__("route_manager_node")

        # Parameters
        self.declare_parameter("route_file", "")
        self.declare_parameter("map_frame", "world")
        self.declare_parameter("vehicle_frame", "OurCar/INS")
        self.declare_parameter("waypoint_reach_distance", 3.0)
        self.declare_parameter("lookahead_index", 1)
        self.declare_parameter("local_path_size", 5)
        self.declare_parameter("initial_waypoint_index", 0)

        self.route_file = self.get_parameter("route_file").value
        self.map_frame = self.get_parameter("map_frame").value
        self.vehicle_frame = self.get_parameter("vehicle_frame").value

        self.reach_distance = float(
            self.get_parameter("waypoint_reach_distance").value
        )

        self.lookahead_index = int(
            self.get_parameter("lookahead_index").value
        )

        self.local_path_size = int(
            self.get_parameter("local_path_size").value
        )

        if self.local_path_size <= 0:
            raise RuntimeError(
                "Parameter local_path_size must be greater than 0."
            )

        # Load route
        self.waypoints = self.load_route_file(self.route_file)
        self.current_index = int(
            self.get_parameter("initial_waypoint_index").value
        )
        if not 0 <= self.current_index < len(self.waypoints):
            raise RuntimeError(
                "Parameter initial_waypoint_index is outside the route."
            )
        self.route_completed = False

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Publishers
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            "/route/target_pose",
            10
        )

        self.speed_limit_pub = self.create_publisher(
            Float32,
            "/route/speed_limit",
            10
        )

        self.current_wp_pub = self.create_publisher(
            Int32,
            "/route/current_waypoint_id",
            10
        )

        self.waypoint_distance_pub = self.create_publisher(
            Float32,
            "/route/current_waypoint_distance",
            10
        )

        self.route_state_pub = self.create_publisher(
            String,
            "/route/route_state",
            10
        )

        self.local_path_pub = self.create_publisher(
            NavPath,
            "/route/local_path",
            10
        )

        # Timer: 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info("Route Manager started.")
        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints."
        )
        self.get_logger().info(
            f"map_frame: {self.map_frame}"
        )
        self.get_logger().info(
            f"vehicle_frame: {self.vehicle_frame}"
        )
        self.get_logger().info(
            f"local_path_size: {self.local_path_size}"
        )
        self.get_logger().info(
            f"initial_waypoint_index: {self.current_index}"
        )

    def load_route_file(self, route_file):
        if not route_file:
            raise RuntimeError(
                "Parameter route_file is empty."
            )

        path = FilePath(route_file)

        if not path.exists():
            raise RuntimeError(
                f"Route file does not exist: {route_file}"
            )

        if not path.is_file():
            raise RuntimeError(
                f"Route path is not a file: {route_file}"
            )

        with open(path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)

        if not isinstance(data, dict):
            raise RuntimeError(
                "Route file does not contain a valid YAML dictionary."
            )

        waypoints = data.get("waypoints", [])

        if not isinstance(waypoints, list) or not waypoints:
            raise RuntimeError(
                "No valid waypoints found in route file."
            )

        required_fields = ["x", "y"]

        for index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                raise RuntimeError(
                    f"Waypoint {index} is not a valid dictionary."
                )

            missing_fields = [
                field
                for field in required_fields
                if field not in waypoint
            ]

            if missing_fields:
                raise RuntimeError(
                    f"Waypoint {index} is missing fields: "
                    f"{missing_fields}"
                )

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

        translation = transform.transform.translation

        ego_x = float(translation.x)
        ego_y = float(translation.y)

        return ego_x, ego_y

    def update_current_waypoint(self, ego_x, ego_y):
        if not self.waypoints or self.route_completed:
            return

        if self.current_index >= len(self.waypoints):
            self.current_index = len(self.waypoints) - 1
            return

        while not self.route_completed:
            progress = waypoint_reached_or_passed(
                self.waypoints,
                self.current_index,
                ego_x,
                ego_y,
                self.reach_distance,
            )
            if not progress:
                break

            waypoint = self.waypoints[self.current_index]
            waypoint_id = int(waypoint.get("id", self.current_index))
            if self.current_index == len(self.waypoints) - 1:
                self.route_completed = True
                self.get_logger().info(
                    f"Route completed at waypoint {waypoint_id}"
                )
                break

            self.current_index += 1
            self.get_logger().info(
                f"Waypoint {waypoint_id} {progress}; "
                f"switching to waypoint index {self.current_index}"
            )

    def get_target_waypoint(self):
        target_index = min(
            self.current_index + self.lookahead_index,
            len(self.waypoints) - 1
        )

        return self.waypoints[target_index], target_index

    def publish_target(self, waypoint, target_index, ego_x, ego_y):
        timestamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.map_frame

        pose_msg.pose.position.x = float(
            waypoint["x"]
        )
        pose_msg.pose.position.y = float(
            waypoint["y"]
        )
        pose_msg.pose.position.z = float(
            waypoint.get("z", 0.0)
        )

        yaw_deg = float(
            waypoint.get("yaw_deg", 0.0)
        )

        pose_msg.pose.orientation = (
            yaw_deg_to_quaternion(yaw_deg)
        )

        speed_msg = Float32()
        speed_msg.data = 0.0 if self.route_completed else float(
            waypoint.get("speed_limit", 2.0)
        )

        id_msg = Int32()
        id_msg.data = int(
            waypoint.get("id", target_index)
        )

        state_msg = String()
        state_msg.data = (
            "route_complete"
            if self.route_completed
            else str(waypoint.get("type", "normal"))
        )

        distance_msg = Float32()
        distance_msg.data = distance_2d(
            ego_x,
            ego_y,
            float(waypoint["x"]),
            float(waypoint["y"]),
        )

        self.target_pose_pub.publish(pose_msg)
        self.speed_limit_pub.publish(speed_msg)
        self.current_wp_pub.publish(id_msg)
        self.waypoint_distance_pub.publish(distance_msg)
        self.route_state_pub.publish(state_msg)

    def publish_local_path(self):
        if not self.waypoints:
            self.get_logger().warn(
                "Cannot publish local path: no waypoints loaded."
            )
            return

        if (
            self.current_index < 0
            or self.current_index >= len(self.waypoints)
        ):
            self.get_logger().warn(
                f"Invalid current waypoint index: "
                f"{self.current_index}"
            )
            return

        # Retain the previous waypoint so the trajectory generator can project
        # the ego pose onto the active route segment instead of falling back to
        # a straight line toward a potentially distant target waypoint.
        start_index = max(0, self.current_index - 1)

        end_index = min(
            start_index + self.local_path_size,
            len(self.waypoints)
        )

        local_waypoints = self.waypoints[
            start_index:end_index
        ]

        path_msg = NavPath()
        path_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )
        path_msg.header.frame_id = self.map_frame

        for waypoint in local_waypoints:
            pose_msg = PoseStamped()

            pose_msg.header.stamp = (
                path_msg.header.stamp
            )
            pose_msg.header.frame_id = self.map_frame

            pose_msg.pose.position.x = float(
                waypoint["x"]
            )
            pose_msg.pose.position.y = float(
                waypoint["y"]
            )
            pose_msg.pose.position.z = float(
                waypoint.get("z", 0.0)
            )

            yaw_deg = float(
                waypoint.get("yaw_deg", 0.0)
            )

            pose_msg.pose.orientation = (
                yaw_deg_to_quaternion(yaw_deg)
            )

            path_msg.poses.append(pose_msg)

        self.local_path_pub.publish(path_msg)

    def timer_callback(self):
        ego_pose = self.get_vehicle_pose()

        if ego_pose is None:
            return

        ego_x, ego_y = ego_pose

        self.update_current_waypoint(
            ego_x,
            ego_y
        )

        target_wp, target_index = (
            self.get_target_waypoint()
        )

        self.publish_target(
            target_wp,
            target_index,
            ego_x,
            ego_y,
        )

        self.publish_local_path()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = RouteManagerNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info(
                "Route Manager stopped."
            )

    except ExternalShutdownException:
        pass

    except Exception as ex:
        # During launch shutdown a timer callback can race with context
        # invalidation.  Suppress that teardown-only failure, but retain normal
        # exception reporting while ROS is active.
        if not rclpy.ok():
            return
        if node is not None:
            node.get_logger().error(
                f"Route Manager failed: {ex}"
            )
        else:
            print(f"Route Manager failed: {ex}")

        raise

    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass

        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
