#!/usr/bin/env python3
"""Publish dependency-free RViz markers for live vehicle telemetry charts."""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Point, TwistStamped
from rclpy.node import Node
from simulation.msg import VehicleControl
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class LiveVisualization(Node):
    WINDOW_SECONDS = 60.0
    GRAPH_WIDTH = 20.0

    def __init__(self):
        super().__init__("live_visualization")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_time = self.now_seconds()
        self.speed = 0.0
        self.throttle = 0.0
        self.steering = 0.0
        self.heading = 0.0
        self.samples = deque()
        self.trace = deque(maxlen=5000)
        self.charts_pub = self.create_publisher(
            MarkerArray, "/visualization/telemetry_charts", 10
        )
        self.trace_pub = self.create_publisher(
            Marker, "/visualization/vehicle_trace", 10
        )
        self.create_subscription(
            TwistStamped, "/OurCar/INS/twist", self.twist_callback, 10
        )
        self.create_subscription(
            VehicleControl, "/car_command", self.command_callback, 10
        )
        self.create_timer(0.1, self.update)
        self.get_logger().info(
            "Live charts: speed, throttle, steering wheel and heading"
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def twist_callback(self, msg):
        velocity = msg.twist.linear
        self.speed = math.sqrt(
            velocity.x * velocity.x
            + velocity.y * velocity.y
            + velocity.z * velocity.z
        )

    def command_callback(self, msg):
        self.throttle = float(msg.throttle)
        self.steering = float(msg.steering)

    @staticmethod
    def color(marker, red, green, blue, alpha=1.0):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    def make_line(self, marker_id, points, color, width=0.055):
        marker = Marker()
        marker.header.frame_id = "telemetry"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "charts"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        self.color(marker, *color)
        marker.points = points
        return marker

    def make_text(self, marker_id, x, y, text, color):
        marker = Marker()
        marker.header.frame_id = "telemetry"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.55
        self.color(marker, *color)
        marker.text = text
        return marker

    def update_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", "OurCar/INS", rclpy.time.Time()
            )
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        self.heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        point = Point()
        point.x = translation.x
        point.y = translation.y
        point.z = 0.25
        if not self.trace or math.hypot(
            point.x - self.trace[-1].x, point.y - self.trace[-1].y
        ) >= 0.1:
            self.trace.append(point)

    def publish_trace(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vehicle_trace"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.22
        self.color(marker, 0.1, 0.9, 1.0)
        marker.points = list(self.trace)
        self.trace_pub.publish(marker)

    def publish_charts(self, now):
        rows = [
            ("Speed", "m/s", 6.0, 0.0, 15.0, (0.1, 0.85, 1.0), self.speed),
            ("Throttle", "%", 2.0, 0.0, 1.0, (0.2, 1.0, 0.3), self.throttle),
            ("Steering wheel", "%", -2.0, -1.0, 1.0, (1.0, 0.75, 0.1), self.steering),
            ("Heading", "deg", -6.0, -180.0, 180.0, (1.0, 0.25, 0.7), self.heading),
        ]
        array = MarkerArray()
        for index, (name, unit, center, low, high, color, value) in enumerate(rows):
            left = Point(x=0.0, y=center - 1.25, z=0.0)
            right = Point(x=self.GRAPH_WIDTH, y=center - 1.25, z=0.0)
            array.markers.append(
                self.make_line(100 + index, [left, right], (0.35, 0.35, 0.38), 0.025)
            )
            points = []
            for sample in self.samples:
                age = now - sample[0]
                x = self.GRAPH_WIDTH * (1.0 - age / self.WINDOW_SECONDS)
                normalized = (sample[index + 1] - low) / (high - low)
                normalized = max(0.0, min(1.0, normalized))
                points.append(Point(x=x, y=center - 1.25 + 2.5 * normalized, z=0.05))
            array.markers.append(self.make_line(index, points, color))
            shown = value * 100.0 if unit == "%" else value
            array.markers.append(
                self.make_text(
                    200 + index,
                    3.2,
                    center + 1.55,
                    f"{name}: {shown:6.2f} {unit}",
                    color,
                )
            )
        array.markers.append(
            self.make_text(300, 17.0, 8.0, "60 s live history", (0.85, 0.85, 0.85))
        )
        self.charts_pub.publish(array)

    def update(self):
        self.update_pose()
        now = self.now_seconds()
        self.samples.append(
            (now, self.speed, self.throttle, self.steering, self.heading)
        )
        while self.samples and now - self.samples[0][0] > self.WINDOW_SECONDS:
            self.samples.popleft()
        self.publish_trace()
        self.publish_charts(now)


def main(args=None):
    rclpy.init(args=args)
    node = LiveVisualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
