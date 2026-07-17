"""Extract front and right-front obstacle distances from a PointCloud2 stream."""

import math
import struct

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from decision_making.path_corridor import prepare_path, project_to_path


class ObstacleMonitorNode(Node):
    """Publish nearest-point distances in two vehicle-frame regions of interest."""

    def __init__(self) -> None:
        super().__init__("obstacle_monitor_node")
        self.declare_parameter(
            "pointcloud_topic", "/perception/pcl/points_preprocessed"
        )
        self.declare_parameter("vehicle_frame", "OurCar/INS")
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("planning_path_topic", "/planning/path")
        self.declare_parameter("planning_path_timeout", 0.75)
        self.declare_parameter("front_min_x", 0.5)
        self.declare_parameter("front_max_x", 30.0)
        self.declare_parameter("front_half_width", 1.2)
        self.declare_parameter("merge_min_x", 0.5)
        self.declare_parameter("merge_max_x", 15.0)
        self.declare_parameter("merge_right_min_y", -2.5)
        self.declare_parameter("merge_right_max_y", -0.8)
        self.declare_parameter("min_z", -0.5)
        self.declare_parameter("max_z", 2.0)

        self.vehicle_frame = self.get_parameter("vehicle_frame").value
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.front_min_x = float(self.get_parameter("front_min_x").value)
        self.front_max_x = float(self.get_parameter("front_max_x").value)
        self.front_half_width = float(self.get_parameter("front_half_width").value)
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
        self.planning_path_timeout = max(
            0.0, float(self.get_parameter("planning_path_timeout").value)
        )
        self.planning_path = None
        self.planning_path_receive_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.front_pub = self.create_publisher(
            Float32, "/decision/front_obstacle_distance", 10
        )
        self.merge_pub = self.create_publisher(
            Float32, "/decision/merge_obstacle_distance", 10
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("pointcloud_topic").value,
            self.pointcloud_callback,
            5,
        )
        self.create_subscription(
            Path,
            self.get_parameter("planning_path_topic").value,
            self.planning_path_callback,
            5,
        )
        self.get_logger().info(
            f"Obstacle monitor started; transforming point cloud into {self.vehicle_frame}"
        )

    def planning_path_callback(self, msg: Path) -> None:
        self.planning_path = msg
        self.planning_path_receive_time = self.get_clock().now()

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
        front_distance = math.inf
        merge_distance = math.inf
        for point in self.iter_xyz(msg):
            x, y, z = self.transform_point(point, transform.transform)
            if z < self.min_z or z > self.max_z:
                continue
            if vehicle_path:
                projection = project_to_path(x, y, vehicle_path)
                if projection is None:
                    continue
                if (
                    self.front_min_x <= projection.along <= self.front_max_x
                    and projection.distance <= self.front_half_width
                ):
                    front_distance = min(front_distance, projection.along)
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
                    front_distance = min(front_distance, math.hypot(x, y))
                if (
                    self.merge_min_x <= x <= self.merge_max_x
                    and self.merge_right_min_y <= y <= self.merge_right_max_y
                ):
                    merge_distance = min(merge_distance, math.hypot(x, y))

        front_msg = Float32()
        front_msg.data = float(front_distance)
        self.front_pub.publish(front_msg)
        merge_msg = Float32()
        merge_msg.data = float(merge_distance)
        self.merge_pub.publish(merge_msg)

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
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
