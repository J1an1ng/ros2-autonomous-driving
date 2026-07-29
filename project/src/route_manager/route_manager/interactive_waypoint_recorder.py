#!/usr/bin/env python3

import math
import threading
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


DEFAULT_WAYPOINT_SPEED = 2.0

TYPE_CONFIG = {
    "": ("normal", DEFAULT_WAYPOINT_SPEED),
    "n": ("normal", DEFAULT_WAYPOINT_SPEED),

    "ca": ("curve_approach", 1.5),
    "ce": ("curve_entry", 1.2),
    "c": ("curve_mid", 1.0),
    "cx": ("curve_exit", 1.3),

    "ia": ("intersection_approach", DEFAULT_WAYPOINT_SPEED),
    "s": ("stop_zone", 0.0),
    "i": ("intersection", DEFAULT_WAYPOINT_SPEED),
    "ix": ("intersection_exit", DEFAULT_WAYPOINT_SPEED),

    "slow": ("slow_zone", 1.0),
}


def quaternion_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw_rad)


class InteractiveWaypointRecorder(Node):
    def __init__(self):
        super().__init__("interactive_waypoint_recorder")

        self.declare_parameter("map_frame", "world")
        self.declare_parameter("vehicle_frame", "OurCar/INS")
        self.declare_parameter("output_file", "route_map.yaml")
        self.declare_parameter("segment_id", 0)

        self.map_frame = self.get_parameter("map_frame").value
        self.vehicle_frame = self.get_parameter("vehicle_frame").value
        self.output_file = self.get_parameter("output_file").value
        self.segment_id = int(self.get_parameter("segment_id").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.waypoints = []

        self.get_logger().info("Interactive waypoint recorder started.")
        self.print_help()

        self.input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.input_thread.start()

    def print_help(self):
        print("\nCommands:")
        print("  ENTER / n   -> normal, speed 2.0")
        print("  ca          -> curve_approach, speed 1.5")
        print("  ce          -> curve_entry, speed 1.2")
        print("  c           -> curve_mid, speed 1.0")
        print("  cx          -> curve_exit, speed 1.3")
        print("  ia          -> intersection_approach, normal speed 2.0")
        print("  s           -> stop_zone, speed 0.0")
        print("  i           -> intersection, normal speed 2.0")
        print("  ix          -> intersection_exit, normal speed 2.0")
        print("  slow        -> slow_zone, speed 1.0")
        print("  seg <id>    -> change segment id, e.g. seg 2")
        print("  undo        -> remove last waypoint")
        print("  list        -> show last 5 waypoints")
        print("  q           -> quit")
        print("")

    def get_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.vehicle_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF not available: {ex}")
            return None

        t = transform.transform.translation
        r = transform.transform.rotation

        return {
            "x": round(float(t.x), 3),
            "y": round(float(t.y), 3),
            "z": round(float(t.z), 3),
            "yaw_deg": round(quaternion_to_yaw_deg(r.x, r.y, r.z, r.w), 3),
        }

    def record_waypoint(self, waypoint_type, speed_limit):
        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn("Could not record waypoint because TF is not available.")
            return

        waypoint_id = len(self.waypoints)

        waypoint = {
            "id": waypoint_id,
            "x": pose["x"],
            "y": pose["y"],
            "z": pose["z"],
            "yaw_deg": pose["yaw_deg"],
            "speed_limit": float(speed_limit),
            "segment_id": self.segment_id,
            "type": waypoint_type,
        }

        self.waypoints.append(waypoint)
        self.save_yaml()

        self.get_logger().info(
            f"Recorded {waypoint_type} waypoint {waypoint_id}: "
            f"x={pose['x']}, y={pose['y']}, yaw={pose['yaw_deg']} deg, "
            f"speed={speed_limit}, segment={self.segment_id}"
        )

    def undo_last(self):
        if not self.waypoints:
            self.get_logger().warn("No waypoint to remove.")
            return

        removed = self.waypoints.pop()
        self.reassign_ids()
        self.save_yaml()
        self.get_logger().info(f"Removed waypoint {removed['id']}.")

    def reassign_ids(self):
        for idx, waypoint in enumerate(self.waypoints):
            waypoint["id"] = idx

    def show_last_waypoints(self):
        if not self.waypoints:
            print("No waypoints recorded yet.")
            return

        print("\nLast waypoints:")
        for wp in self.waypoints[-5:]:
            print(
                f"  id={wp['id']}, x={wp['x']}, y={wp['y']}, "
                f"yaw={wp['yaw_deg']}, type={wp['type']}, "
                f"speed={wp['speed_limit']}, segment={wp['segment_id']}"
            )
        print("")

    def build_segments(self):
        if not self.waypoints:
            return []

        segments = []
        segment_ids = sorted(set(wp["segment_id"] for wp in self.waypoints))

        for sid in segment_ids:
            ids = [wp["id"] for wp in self.waypoints if wp["segment_id"] == sid]
            if not ids:
                continue

            segment_types = [
                wp["type"] for wp in self.waypoints
                if wp["segment_id"] == sid
            ]

            if any("intersection" in t for t in segment_types):
                segment_type = "intersection"
            elif any("curve" in t for t in segment_types):
                segment_type = "curve"
            elif any("slow" in t for t in segment_types):
                segment_type = "slow_zone"
            else:
                segment_type = "normal"

            segment_speed = min(
                wp["speed_limit"] for wp in self.waypoints
                if wp["segment_id"] == sid
            )

            segments.append({
                "id": sid,
                "name": f"segment_{sid}",
                "waypoint_start": min(ids),
                "waypoint_end": max(ids),
                "type": segment_type,
                "speed_limit": float(segment_speed),
            })

        return segments

    def build_zones(self):
        zones = []

        for wp in self.waypoints:
            if wp["type"] == "stop_zone":
                zones.append({
                    "id": f"stop_zone_{wp['id']}",
                    "type": "stop_zone",
                    "related_waypoint": wp["id"],
                    "radius": 3.0,
                    "speed_limit": 0.0,
                })

            elif wp["type"] == "intersection_approach":
                zones.append({
                    "id": f"intersection_approach_{wp['id']}",
                    "type": "intersection_approach",
                    "related_waypoint": wp["id"],
                    "radius": 5.0,
                    "speed_limit": wp["speed_limit"],
                })

            elif wp["type"] in ["curve_approach", "curve_entry", "curve_mid", "curve_exit"]:
                zones.append({
                    "id": f"curve_zone_{wp['id']}",
                    "type": "curve",
                    "related_waypoint": wp["id"],
                    "radius": 3.0,
                    "speed_limit": wp["speed_limit"],
                })

        return zones

    def save_yaml(self):
        route_map = {
            "map_frame": self.map_frame,
            "vehicle_frame": self.vehicle_frame,
            "waypoints": self.waypoints,
            "segments": self.build_segments(),
            "zones": self.build_zones(),
        }

        output_path = Path(self.output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, "w", encoding="utf-8") as f:
            yaml.dump(route_map, f, sort_keys=False, allow_unicode=True)

    def keyboard_loop(self):
        while rclpy.ok():
            command = input("record> ").strip().lower()

            if command == "q":
                self.get_logger().info("Quit requested.")
                self.save_yaml()
                rclpy.shutdown()
                break

            if command == "help":
                self.print_help()
                continue

            if command == "undo":
                self.undo_last()
                continue

            if command == "list":
                self.show_last_waypoints()
                continue

            if command.startswith("seg "):
                parts = command.split()
                if len(parts) == 2 and parts[1].isdigit():
                    self.segment_id = int(parts[1])
                    self.get_logger().info(f"Changed segment_id to {self.segment_id}.")
                else:
                    self.get_logger().warn("Use: seg <id>, for example: seg 2")
                continue

            if command not in TYPE_CONFIG:
                self.get_logger().warn(
                    f"Unknown command '{command}'. Type 'help' for available commands."
                )
                continue

            waypoint_type, speed_limit = TYPE_CONFIG[command]
            self.record_waypoint(waypoint_type, speed_limit)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveWaypointRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
    finally:
        try:
            node.save_yaml()
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
