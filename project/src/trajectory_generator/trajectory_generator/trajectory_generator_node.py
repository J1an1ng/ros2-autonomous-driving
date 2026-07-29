#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Int32, String
from tf2_ros import Buffer, TransformException, TransformListener

from trajectory_generator.geometry_utils import (
    Pose2D,
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from trajectory_generator.trajectory_generator import (
    TrajectoryGenerator,
    TrajectoryGeneratorConfig,
)


class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__("trajectory_generator_node")

        self.declare_parameter("map_frame", "world")
        self.declare_parameter("vehicle_frame", "OurCar/INS")
        self.declare_parameter("update_rate", 10.0)
        self.declare_parameter("path_resolution", 1.0)
        self.declare_parameter("min_target_distance", 0.5)
        self.declare_parameter("smoothing_strength", 0.25)
        self.declare_parameter("max_curvature", 2.0)
        self.declare_parameter("max_yaw_step", 1.1)
        self.declare_parameter("max_start_distance", 5.0)
        self.declare_parameter("max_spline_deviation", 2.5)
        self.declare_parameter("use_waypoint_yaw", True)
        self.declare_parameter("continuity_blend_distance", 5.0)
        self.declare_parameter("continuity_candidate_weight", 0.5)
        self.declare_parameter("max_continuity_deviation", 2.0)
        self.declare_parameter("default_speed", 2.0)
        self.declare_parameter("max_speed", 14.0)
        self.declare_parameter("stop_speed", 0.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.vehicle_frame = self.get_parameter("vehicle_frame").value
        update_rate = float(self.get_parameter("update_rate").value)

        config = TrajectoryGeneratorConfig(
            path_resolution=float(self.get_parameter("path_resolution").value),
            min_target_distance=float(self.get_parameter("min_target_distance").value),
            smoothing_strength=float(
                self.get_parameter("smoothing_strength").value
            ),
            max_curvature=float(self.get_parameter("max_curvature").value),
            max_yaw_step=float(self.get_parameter("max_yaw_step").value),
            max_start_distance=float(
                self.get_parameter("max_start_distance").value
            ),
            max_spline_deviation=float(
                self.get_parameter("max_spline_deviation").value
            ),
            use_waypoint_yaw=bool(
                self.get_parameter("use_waypoint_yaw").value
            ),
            continuity_blend_distance=float(
                self.get_parameter("continuity_blend_distance").value
            ),
            continuity_candidate_weight=float(
                self.get_parameter("continuity_candidate_weight").value
            ),
            max_continuity_deviation=float(
                self.get_parameter("max_continuity_deviation").value
            ),
            default_speed=float(self.get_parameter("default_speed").value),
            max_speed=float(self.get_parameter("max_speed").value),
            stop_speed=float(self.get_parameter("stop_speed").value),
        )
        self.trajectory_generator = TrajectoryGenerator(config)

        self.target_pose_msg = None
        self.local_path = []
        self.speed_limit = None
        self.route_state = ""
        self.current_waypoint_id = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseStamped,
            "/route/target_pose",
            self.target_pose_callback,
            10,
        )
        self.create_subscription(
            Path,
            "/route/local_path",
            self.local_path_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/route/speed_limit",
            self.speed_limit_callback,
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
            self.current_waypoint_id_callback,
            10,
        )

        self.path_pub = self.create_publisher(Path, "/planning/path", 10)
        self.target_speed_pub = self.create_publisher(
            Float32,
            "/planning/target_speed",
            10,
        )

        timer_period = 1.0 / max(update_rate, 1e-3)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Trajectory Generator started.")
        self.get_logger().info(f"map_frame: {self.map_frame}")
        self.get_logger().info(f"vehicle_frame: {self.vehicle_frame}")

    def target_pose_callback(self, msg):
        self.target_pose_msg = msg

    def local_path_callback(self, msg):
        self.local_path = [
            self.pose_stamped_to_pose2d(pose_msg) for pose_msg in msg.poses
        ]

    def speed_limit_callback(self, msg):
        self.speed_limit = float(msg.data)

    def route_state_callback(self, msg):
        self.route_state = str(msg.data)

    def current_waypoint_id_callback(self, msg):
        self.current_waypoint_id = int(msg.data)

    def get_ego_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.vehicle_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"TF not available: {ex}",
                throttle_duration_sec=2.0,
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)

        return Pose2D(
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
            yaw=float(yaw),
        )

    def pose_stamped_to_pose2d(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        return Pose2D(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=float(yaw),
        )

    def path_points_to_msg(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame

        for point in path_points:
            pose_msg = PoseStamped()
            pose_msg.header = path_msg.header
            pose_msg.pose.position.x = float(point.x)
            pose_msg.pose.position.y = float(point.y)
            pose_msg.pose.position.z = float(point.z)

            qx, qy, qz, qw = yaw_to_quaternion(point.yaw)
            pose_msg.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            path_msg.poses.append(pose_msg)

        return path_msg

    def timer_callback(self):
        ego_pose = self.get_ego_pose()
        if ego_pose is None:
            return

        target_pose = None
        if self.target_pose_msg is not None:
            target_pose = self.pose_stamped_to_pose2d(self.target_pose_msg)
        trajectory = self.trajectory_generator.generate(
            ego_pose=ego_pose,
            target_pose=target_pose,
            local_path=self.local_path,
            speed_limit=self.speed_limit,
            route_state=self.route_state,
        )
        if trajectory.path_status:
            self.get_logger().warn(
                trajectory.path_status,
                throttle_duration_sec=2.0,
            )

        path_msg = self.path_points_to_msg(trajectory.path_points)
        speed_msg = Float32()
        speed_msg.data = float(trajectory.target_speed)

        self.path_pub.publish(path_msg)
        self.target_speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Trajectory Generator stopped.")
    except ExternalShutdownException:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
