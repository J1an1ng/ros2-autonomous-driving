"""Extract front and right-front obstacle distances from a PointCloud2 stream."""

import math
import struct

import rclpy
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float32, Int32
from tf2_ros import Buffer, TransformException, TransformListener

from decision_making.obstacle_fusion import (
    VehicleObstacleFusion,
    fuse_vehicle_obstacle_distances,
    vision_distance_is_plausible,
)
from decision_making.path_corridor import (
    in_vehicle_safety_corridor,
    matches_fixed_structure_signature,
    prepare_path,
    project_to_path,
    robust_nearest_distance,
)


class ObstacleMonitorNode(Node):
    """Publish nearest-point distances in two vehicle-frame regions of interest."""

    def __init__(self) -> None:
        super().__init__("obstacle_monitor_node")
        self.declare_parameter(
            "pointcloud_topic", "/perception/pcl/points_preprocessed"
        )
        self.declare_parameter(
            "safety_pointcloud_topic", "/perception/pcl/points"
        )
        self.declare_parameter("safety_pointcloud_timeout", 0.5)
        self.declare_parameter("vehicle_frame", "OurCar/INS")
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("planning_path_topic", "/planning/path")
        self.declare_parameter("planning_path_timeout", 0.75)
        self.declare_parameter("pointcloud_match_timeout", 0.75)
        self.declare_parameter("front_min_x", 0.5)
        self.declare_parameter("front_max_x", 30.0)
        self.declare_parameter("front_half_width", 1.2)
        self.declare_parameter("safety_corridor_enabled", True)
        self.declare_parameter("safety_min_x", 0.5)
        self.declare_parameter("safety_max_x", 3.5)
        self.declare_parameter("safety_half_width", 1.1)
        self.declare_parameter("safety_min_points", 5)
        self.declare_parameter(
            "fixed_structure_suppression_waypoint_ids", [48]
        )
        self.declare_parameter("fixed_structure_max_path_distance", 9.5)
        self.declare_parameter("fixed_structure_max_safety_distance", 4.0)
        self.declare_parameter("fixed_structure_max_merge_distance", 9.5)
        self.declare_parameter("merge_min_x", 0.5)
        self.declare_parameter("merge_max_x", 15.0)
        self.declare_parameter("merge_right_min_y", -2.5)
        self.declare_parameter("merge_right_max_y", -0.8)
        self.declare_parameter("min_z", -0.5)
        self.declare_parameter("max_z", 2.0)
        self.declare_parameter(
            "vision_vehicle_detected_topic",
            "/perception/front_vehicle_detected",
        )
        self.declare_parameter(
            "vision_vehicle_distance_topic",
            "/perception/front_vehicle_distance",
        )
        self.declare_parameter("vision_vehicle_timeout", 1.0)
        self.declare_parameter("vision_vehicle_max_abs_range_rate", 6.0)
        self.declare_parameter("vision_vehicle_jump_tolerance", 1.5)
        self.declare_parameter("vehicle_distance_match_tolerance", 2.5)
        self.declare_parameter(
            "vehicle_distance_match_relative_tolerance", 0.2
        )
        self.declare_parameter(
            "vision_direct_control_waypoint_ids",
            [43, 44, 45, 46, 47, 48],
        )
        self.declare_parameter("unconfirmed_emergency_distance", 0.8)

        self.vehicle_frame = self.get_parameter("vehicle_frame").value
        self.safety_pointcloud_topic = self.get_parameter(
            "safety_pointcloud_topic"
        ).value
        self.safety_pointcloud_timeout = max(
            0.0,
            float(self.get_parameter("safety_pointcloud_timeout").value),
        )
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.front_min_x = float(self.get_parameter("front_min_x").value)
        self.front_max_x = float(self.get_parameter("front_max_x").value)
        self.front_half_width = float(self.get_parameter("front_half_width").value)
        self.safety_corridor_enabled = bool(
            self.get_parameter("safety_corridor_enabled").value
        )
        self.safety_min_x = float(self.get_parameter("safety_min_x").value)
        self.safety_max_x = float(self.get_parameter("safety_max_x").value)
        self.safety_half_width = float(
            self.get_parameter("safety_half_width").value
        )
        self.safety_min_points = max(
            1, int(self.get_parameter("safety_min_points").value)
        )
        self.fixed_structure_suppression_waypoint_ids = {
            int(value)
            for value in self.get_parameter(
                "fixed_structure_suppression_waypoint_ids"
            ).value
        }
        self.fixed_structure_max_path_distance = float(
            self.get_parameter("fixed_structure_max_path_distance").value
        )
        self.fixed_structure_max_safety_distance = float(
            self.get_parameter("fixed_structure_max_safety_distance").value
        )
        self.fixed_structure_max_merge_distance = float(
            self.get_parameter("fixed_structure_max_merge_distance").value
        )
        self.merge_min_x = float(self.get_parameter("merge_min_x").value)
        self.merge_max_x = float(self.get_parameter("merge_max_x").value)
        self.merge_right_min_y = float(
            self.get_parameter("merge_right_min_y").value
        )
        self.merge_right_max_y = float(
            self.get_parameter("merge_right_max_y").value
        )
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.vision_vehicle_timeout = max(
            0.0,
            float(self.get_parameter("vision_vehicle_timeout").value),
        )
        self.vision_vehicle_max_abs_range_rate = max(
            0.0,
            float(
                self.get_parameter(
                    "vision_vehicle_max_abs_range_rate"
                ).value
            ),
        )
        self.vision_vehicle_jump_tolerance = max(
            0.0,
            float(
                self.get_parameter(
                    "vision_vehicle_jump_tolerance"
                ).value
            ),
        )
        self.vehicle_distance_match_tolerance = max(
            0.0,
            float(
                self.get_parameter(
                    "vehicle_distance_match_tolerance"
                ).value
            ),
        )
        self.vehicle_distance_match_relative_tolerance = max(
            0.0,
            float(
                self.get_parameter(
                    "vehicle_distance_match_relative_tolerance"
                ).value
            ),
        )
        self.vision_direct_control_waypoint_ids = {
            int(value)
            for value in self.get_parameter(
                "vision_direct_control_waypoint_ids"
            ).value
        }
        self.unconfirmed_emergency_distance = max(
            0.0,
            float(
                self.get_parameter("unconfirmed_emergency_distance").value
            ),
        )
        self.planning_path_timeout = max(
            0.0, float(self.get_parameter("planning_path_timeout").value)
        )
        self.pointcloud_match_timeout = max(
            0.0,
            float(self.get_parameter("pointcloud_match_timeout").value),
        )
        self.planning_path = None
        self.planning_path_receive_time = None
        self.pointcloud_receive_time = None
        self.path_front_distance = math.inf
        self.merge_distance = math.inf
        self.preprocessed_safety_front_distance = math.inf
        self.raw_safety_front_distance = math.inf
        self.raw_safety_receive_time = None
        self.waypoint_id = None
        self.fixed_structure_suppressed = False
        self.vision_vehicle_detected = False
        self.vision_vehicle_distance = math.inf
        self.vision_vehicle_distance_accepted_time = None
        self.vision_vehicle_detected_receive_time = None
        self.vision_vehicle_distance_receive_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.front_pub = self.create_publisher(
            Float32, "/decision/front_obstacle_distance", 10
        )
        self.merge_pub = self.create_publisher(
            Float32, "/decision/merge_obstacle_distance", 10
        )
        pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            5,
        )
        if self.safety_pointcloud_topic != pointcloud_topic:
            self.create_subscription(
                PointCloud2,
                self.safety_pointcloud_topic,
                self.safety_pointcloud_callback,
                5,
            )
        self.create_subscription(
            Path,
            self.get_parameter("planning_path_topic").value,
            self.planning_path_callback,
            5,
        )
        self.create_subscription(
            Int32,
            "/route/current_waypoint_id",
            self.waypoint_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("vision_vehicle_detected_topic").value,
            self.vision_vehicle_detected_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self.get_parameter("vision_vehicle_distance_topic").value,
            self.vision_vehicle_distance_callback,
            10,
        )
        self.get_logger().info(
            f"Obstacle monitor started; path cloud={pointcloud_topic} "
            f"safety cloud={self.safety_pointcloud_topic} "
            f"vehicle frame={self.vehicle_frame}"
        )

    def vision_vehicle_detected_callback(self, msg: Bool) -> None:
        self.vision_vehicle_detected = bool(msg.data)
        self.vision_vehicle_detected_receive_time = self.get_clock().now()

    def vision_vehicle_distance_callback(self, msg: Float32) -> None:
        now = self.get_clock().now()
        distance = float(msg.data)
        candidate = (
            distance
            if math.isfinite(distance) and distance > 0.0
            else math.inf
        )
        if not self.vision_vehicle_detected or not math.isfinite(candidate):
            self.vision_vehicle_distance = math.inf
            self.vision_vehicle_distance_accepted_time = None
        else:
            elapsed = math.inf
            if self.vision_vehicle_distance_accepted_time is not None:
                elapsed = (
                    now - self.vision_vehicle_distance_accepted_time
                ).nanoseconds / 1e9
            if vision_distance_is_plausible(
                self.vision_vehicle_distance,
                candidate,
                elapsed,
                self.vision_vehicle_max_abs_range_rate,
                self.vision_vehicle_jump_tolerance,
            ):
                self.vision_vehicle_distance = candidate
                self.vision_vehicle_distance_accepted_time = now
        self.vision_vehicle_distance_receive_time = now
        self.publish_distances()

    def planning_path_callback(self, msg: Path) -> None:
        self.planning_path = msg
        self.planning_path_receive_time = self.get_clock().now()

    def waypoint_callback(self, msg: Int32) -> None:
        waypoint_id = int(msg.data)
        if waypoint_id != self.waypoint_id:
            self.fixed_structure_suppressed = False
        self.waypoint_id = waypoint_id

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        try:
            lookup_time = (
                rclpy.time.Time()
                if self.use_latest_tf
                else rclpy.time.Time.from_msg(msg.header.stamp)
            )
            transform = self.tf_buffer.lookup_transform(
                self.vehicle_frame,
                msg.header.frame_id,
                lookup_time,
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Point-cloud transform unavailable: {ex}",
                throttle_duration_sec=2.0,
            )
            return

        vehicle_path = self.get_vehicle_path(msg.header.frame_id, transform.transform)
        path_front_distance = math.inf
        merge_distance = math.inf
        safety_distances = []
        for point in self.iter_xyz(msg):
            x, y, z = self.transform_point(point, transform.transform)
            if z < self.min_z or z > self.max_z:
                continue
            if self.safety_corridor_enabled and in_vehicle_safety_corridor(
                x,
                y,
                self.safety_min_x,
                self.safety_max_x,
                self.safety_half_width,
            ):
                safety_distances.append(math.hypot(x, y))
            if vehicle_path:
                projection = project_to_path(x, y, vehicle_path)
                if projection is None:
                    continue
                if (
                    self.front_min_x <= projection.along <= self.front_max_x
                    and projection.distance <= self.front_half_width
                ):
                    path_front_distance = min(
                        path_front_distance, projection.along
                    )
                if (
                    self.merge_min_x <= projection.along <= self.merge_max_x
                    and self.merge_right_min_y
                    <= projection.lateral
                    <= self.merge_right_max_y
                ):
                    merge_distance = min(merge_distance, projection.along)
            else:
                # A missing/stale planning path must not disable collision
                # protection. Retain the conservative vehicle-frame fallback.
                if (
                    self.front_min_x <= x <= self.front_max_x
                    and abs(y) <= self.front_half_width
                ):
                    path_front_distance = min(
                        path_front_distance, math.hypot(x, y)
                    )
                if (
                    self.merge_min_x <= x <= self.merge_max_x
                    and self.merge_right_min_y <= y <= self.merge_right_max_y
                ):
                    merge_distance = min(merge_distance, math.hypot(x, y))

        self.path_front_distance = path_front_distance
        self.merge_distance = merge_distance
        self.preprocessed_safety_front_distance = robust_nearest_distance(
            safety_distances, self.safety_min_points
        )
        self.pointcloud_receive_time = self.get_clock().now()
        self.publish_distances(publish_merge=True)

    def safety_pointcloud_callback(self, msg: PointCloud2) -> None:
        try:
            lookup_time = (
                rclpy.time.Time()
                if self.use_latest_tf
                else rclpy.time.Time.from_msg(msg.header.stamp)
            )
            transform = self.tf_buffer.lookup_transform(
                self.vehicle_frame,
                msg.header.frame_id,
                lookup_time,
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Raw safety point-cloud transform unavailable: {ex}",
                throttle_duration_sec=2.0,
            )
            return

        safety_distances = []
        for point in self.iter_xyz(msg):
            x, y, z = self.transform_point(point, transform.transform)
            if z < self.min_z or z > self.max_z:
                continue
            if self.safety_corridor_enabled and in_vehicle_safety_corridor(
                x,
                y,
                self.safety_min_x,
                self.safety_max_x,
                self.safety_half_width,
            ):
                safety_distances.append(math.hypot(x, y))
        self.raw_safety_front_distance = robust_nearest_distance(
            safety_distances, self.safety_min_points
        )
        self.raw_safety_receive_time = self.get_clock().now()
        self.publish_distances()

    def publish_distances(self, publish_merge: bool = False) -> None:
        now = self.get_clock().now()
        pointcloud_fresh = False
        if self.pointcloud_receive_time is not None:
            pointcloud_age = (
                now - self.pointcloud_receive_time
            ).nanoseconds / 1e9
            pointcloud_fresh = (
                0.0 <= pointcloud_age <= self.pointcloud_match_timeout
            )
        raw_safety_distance = math.inf
        if self.raw_safety_receive_time is not None:
            raw_age = (
                now - self.raw_safety_receive_time
            ).nanoseconds / 1e9
            if 0.0 <= raw_age <= self.safety_pointcloud_timeout:
                raw_safety_distance = self.raw_safety_front_distance
        path_front_distance = (
            self.path_front_distance if pointcloud_fresh else math.inf
        )
        merge_distance = (
            self.merge_distance if pointcloud_fresh else math.inf
        )
        preprocessed_safety_distance = (
            self.preprocessed_safety_front_distance
            if pointcloud_fresh
            else math.inf
        )
        safety_front_distance = min(
            preprocessed_safety_distance,
            raw_safety_distance,
        )
        if not self.fixed_structure_suppressed:
            self.fixed_structure_suppressed = matches_fixed_structure_signature(
                self.waypoint_id,
                self.fixed_structure_suppression_waypoint_ids,
                path_front_distance,
                safety_front_distance,
                merge_distance,
                self.fixed_structure_max_path_distance,
                self.fixed_structure_max_safety_distance,
                self.fixed_structure_max_merge_distance,
            )
        pointcloud_front_distance = (
            math.inf
            if self.fixed_structure_suppressed
            else min(path_front_distance, safety_front_distance)
        )
        vision_fresh = self.vision_vehicle_is_fresh()
        fusion = fuse_vehicle_obstacle_distances(
            pointcloud_front_distance,
            merge_distance,
            vision_fresh and self.vision_vehicle_detected,
            self.vision_vehicle_distance,
            self.vehicle_distance_match_tolerance,
            self.vehicle_distance_match_relative_tolerance,
            self.unconfirmed_emergency_distance,
            vision_direct_control=(
                self.waypoint_id
                in self.vision_direct_control_waypoint_ids
            ),
            pointcloud_match_front_distance=path_front_distance,
        )

        self._publish_fusion(fusion, publish_merge=publish_merge)

    def _publish_fusion(
        self, fusion: VehicleObstacleFusion, publish_merge: bool
    ) -> None:
        front_msg = Float32()
        front_msg.data = float(fusion.front_distance)
        self.front_pub.publish(front_msg)
        if publish_merge:
            merge_msg = Float32()
            merge_msg.data = float(fusion.merge_distance)
            self.merge_pub.publish(merge_msg)

    def vision_vehicle_is_fresh(self) -> bool:
        if (
            self.vision_vehicle_detected_receive_time is None
            or self.vision_vehicle_distance_receive_time is None
        ):
            return False
        now = self.get_clock().now()
        detected_age = (
            now - self.vision_vehicle_detected_receive_time
        ).nanoseconds / 1e9
        distance_age = (
            now - self.vision_vehicle_distance_receive_time
        ).nanoseconds / 1e9
        return (
            0.0 <= detected_age <= self.vision_vehicle_timeout
            and 0.0 <= distance_age <= self.vision_vehicle_timeout
        )

    def get_vehicle_path(self, cloud_frame, cloud_transform):
        if self.planning_path is None or self.planning_path_receive_time is None:
            return []
        age = (self.get_clock().now() - self.planning_path_receive_time).nanoseconds / 1e9
        if age < 0.0 or age > self.planning_path_timeout:
            return []

        path_frame = self.planning_path.header.frame_id
        if not path_frame:
            return []
        try:
            if path_frame == cloud_frame:
                path_transform = cloud_transform
            else:
                path_transform = self.tf_buffer.lookup_transform(
                    self.vehicle_frame,
                    path_frame,
                    rclpy.time.Time(),
                ).transform
        except TransformException as ex:
            self.get_logger().warn(
                f"Planning-path transform unavailable: {ex}",
                throttle_duration_sec=2.0,
            )
            return []

        points = []
        for pose in self.planning_path.poses:
            position = pose.pose.position
            x, y, _ = self.transform_point(
                (position.x, position.y, position.z), path_transform
            )
            points.append((x, y))
        path = prepare_path(points)
        return path if len(path) >= 2 else []

    @staticmethod
    def iter_xyz(msg: PointCloud2):
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            return
        endian = ">" if msg.is_bigendian else "<"
        readers = []
        for name in ("x", "y", "z"):
            field = fields[name]
            if field.datatype == PointField.FLOAT32:
                readers.append((field.offset, struct.Struct(endian + "f")))
            elif field.datatype == PointField.FLOAT64:
                readers.append((field.offset, struct.Struct(endian + "d")))
            else:
                return

        for row in range(msg.height):
            row_offset = row * msg.row_step
            for column in range(msg.width):
                point_offset = row_offset + column * msg.point_step
                xyz = tuple(
                    reader.unpack_from(msg.data, point_offset + offset)[0]
                    for offset, reader in readers
                )
                if all(math.isfinite(value) for value in xyz):
                    yield xyz

    @staticmethod
    def transform_point(point, transform):
        x, y, z = point
        q = transform.rotation
        tx = 2.0 * (q.y * z - q.z * y)
        ty = 2.0 * (q.z * x - q.x * z)
        tz = 2.0 * (q.x * y - q.y * x)
        rotated_x = x + q.w * tx + (q.y * tz - q.z * ty)
        rotated_y = y + q.w * ty + (q.z * tx - q.x * tz)
        rotated_z = z + q.w * tz + (q.x * ty - q.y * tx)
        return (
            rotated_x + transform.translation.x,
            rotated_y + transform.translation.y,
            rotated_z + transform.translation.z,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMonitorNode()
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
