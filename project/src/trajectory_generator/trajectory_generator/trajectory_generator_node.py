#!/usr/bin/env python3

import math

from autonomy_interfaces.msg import PlannedTrajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped, Quaternion
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
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
        super().__init__('trajectory_generator_node')

        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('vehicle_frame', 'OurCar/INS')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('path_resolution', 1.0)
        self.declare_parameter('min_target_distance', 0.5)
        self.declare_parameter('smoothing_strength', 0.25)
        self.declare_parameter('max_curvature', 2.0)
        self.declare_parameter('max_yaw_step', 1.1)
        self.declare_parameter('max_start_distance', 5.0)
        self.declare_parameter('max_spline_deviation', 2.5)
        self.declare_parameter('use_waypoint_yaw', True)
        self.declare_parameter('continuity_blend_distance', 5.0)
        self.declare_parameter('continuity_candidate_weight', 0.5)
        self.declare_parameter('max_continuity_deviation', 2.0)
        self.declare_parameter('default_speed', 2.0)
        self.declare_parameter('max_speed', 14.0)
        self.declare_parameter('stop_speed', 0.0)
        self.declare_parameter('max_lateral_acceleration', 3.0)
        self.declare_parameter('max_longitudinal_acceleration', 2.0)
        self.declare_parameter('max_longitudinal_deceleration', 3.0)
        self.declare_parameter('target_speed_lookahead_distance', 4.0)
        self.declare_parameter('velocity_timeout', 1.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.vehicle_frame = self.get_parameter('vehicle_frame').value
        update_rate = float(self.get_parameter('update_rate').value)

        config = TrajectoryGeneratorConfig(
            path_resolution=float(self.get_parameter('path_resolution').value),
            min_target_distance=float(self.get_parameter('min_target_distance').value),
            smoothing_strength=float(
                self.get_parameter('smoothing_strength').value
            ),
            max_curvature=float(self.get_parameter('max_curvature').value),
            max_yaw_step=float(self.get_parameter('max_yaw_step').value),
            max_start_distance=float(
                self.get_parameter('max_start_distance').value
            ),
            max_spline_deviation=float(
                self.get_parameter('max_spline_deviation').value
            ),
            use_waypoint_yaw=bool(
                self.get_parameter('use_waypoint_yaw').value
            ),
            continuity_blend_distance=float(
                self.get_parameter('continuity_blend_distance').value
            ),
            continuity_candidate_weight=float(
                self.get_parameter('continuity_candidate_weight').value
            ),
            max_continuity_deviation=float(
                self.get_parameter('max_continuity_deviation').value
            ),
            default_speed=float(self.get_parameter('default_speed').value),
            max_speed=float(self.get_parameter('max_speed').value),
            stop_speed=float(self.get_parameter('stop_speed').value),
            max_lateral_acceleration=float(
                self.get_parameter('max_lateral_acceleration').value
            ),
            max_longitudinal_acceleration=float(
                self.get_parameter('max_longitudinal_acceleration').value
            ),
            max_longitudinal_deceleration=float(
                self.get_parameter('max_longitudinal_deceleration').value
            ),
            target_speed_lookahead_distance=float(
                self.get_parameter('target_speed_lookahead_distance').value
            ),
        )
        self.trajectory_generator = TrajectoryGenerator(config)
        self.velocity_timeout = max(
            float(self.get_parameter('velocity_timeout').value),
            0.1,
        )

        self.target_pose_msg = None
        self.local_path = []
        self.speed_limit = None
        self.route_state = ''
        self.current_waypoint_id = None
        self.current_speed = None
        self.last_velocity_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseStamped,
            '/route/target_pose',
            self.target_pose_callback,
            10,
        )
        self.create_subscription(
            Path,
            '/route/local_path',
            self.local_path_callback,
            10,
        )
        self.create_subscription(
            Float32,
            '/route/speed_limit',
            self.speed_limit_callback,
            10,
        )
        self.create_subscription(
            String,
            '/route/route_state',
            self.route_state_callback,
            10,
        )
        self.create_subscription(
            Int32,
            '/route/current_waypoint_id',
            self.current_waypoint_id_callback,
            10,
        )
        self.create_subscription(
            TwistStamped,
            '/OurCar/INS/twist',
            self.twist_callback,
            10,
        )

        self.path_pub = self.create_publisher(Path, '/planning/path', 10)
        self.trajectory_pub = self.create_publisher(
            PlannedTrajectory,
            '/planning/trajectory',
            10,
        )
        self.target_speed_pub = self.create_publisher(
            Float32,
            '/planning/target_speed',
            10,
        )

        timer_period = 1.0 / max(update_rate, 1e-3)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Trajectory Generator started.')
        self.get_logger().info(f'map_frame: {self.map_frame}')
        self.get_logger().info(f'vehicle_frame: {self.vehicle_frame}')

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

    def twist_callback(self, msg):
        velocity = msg.twist.linear
        speed = math.sqrt(
            float(velocity.x) ** 2
            + float(velocity.y) ** 2
            + float(velocity.z) ** 2
        )
        if not math.isfinite(speed):
            self.current_speed = None
            self.last_velocity_time = None
            return
        self.current_speed = speed
        self.last_velocity_time = self.get_clock().now()

    def fresh_current_speed(self):
        if self.current_speed is None or self.last_velocity_time is None:
            return None
        age = (
            self.get_clock().now() - self.last_velocity_time
        ).nanoseconds / 1e9
        if age < 0.0 or age > self.velocity_timeout:
            return None
        return self.current_speed

    def get_ego_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.vehicle_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'TF not available: {ex}',
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

    def path_points_to_msg(self, path_points, stamp):
        path_msg = Path()
        path_msg.header.stamp = stamp
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

    def trajectory_samples_to_msg(self, samples, stamp):
        trajectory_msg = PlannedTrajectory()
        trajectory_msg.header.stamp = stamp
        trajectory_msg.header.frame_id = self.map_frame

        for sample in samples:
            point = sample.path_point
            point_msg = TrajectoryPoint()
            point_msg.pose.position.x = float(point.x)
            point_msg.pose.position.y = float(point.y)
            point_msg.pose.position.z = float(point.z)
            qx, qy, qz, qw = yaw_to_quaternion(point.yaw)
            point_msg.pose.orientation = Quaternion(
                x=qx,
                y=qy,
                z=qz,
                w=qw,
            )
            point_msg.curvature = float(point.curvature)
            point_msg.target_speed = float(sample.target_speed)
            duration = max(float(sample.time_from_start), 0.0)
            point_msg.time_from_start.sec = int(duration)
            point_msg.time_from_start.nanosec = int(
                round((duration - int(duration)) * 1e9)
            )
            if point_msg.time_from_start.nanosec >= 1_000_000_000:
                point_msg.time_from_start.sec += 1
                point_msg.time_from_start.nanosec -= 1_000_000_000
            trajectory_msg.points.append(point_msg)

        return trajectory_msg

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
            current_speed=self.fresh_current_speed(),
        )
        if trajectory.path_status:
            self.get_logger().warn(
                trajectory.path_status,
                throttle_duration_sec=2.0,
            )

        stamp = self.get_clock().now().to_msg()
        path_msg = self.path_points_to_msg(trajectory.path_points, stamp)
        trajectory_msg = self.trajectory_samples_to_msg(
            trajectory.trajectory_samples,
            stamp,
        )
        speed_msg = Float32()
        speed_msg.data = float(trajectory.target_speed)

        self.path_pub.publish(path_msg)
        self.trajectory_pub.publish(trajectory_msg)
        self.target_speed_pub.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Trajectory Generator stopped.')
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


if __name__ == '__main__':
    main()
