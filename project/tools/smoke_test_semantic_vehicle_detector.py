#!/usr/bin/env python3
"""Synthetic ROS smoke test for semantic vehicle/depth fusion."""

from __future__ import annotations

import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


WIDTH = 40
HEIGHT = 30


def make_semantic(
    node: Node,
    color: tuple[int, int, int],
    x_min: int = 10,
    x_max: int = 30,
) -> Image:
    message = Image()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'test_semantic_camera'
    message.width = WIDTH
    message.height = HEIGHT
    message.encoding = 'rgb8'
    message.is_bigendian = False
    message.step = WIDTH * 3
    data = bytearray(message.step * HEIGHT)
    red, green, blue = color
    for y in range(9, 25):
        for x in range(x_min, x_max):
            offset = y * message.step + x * 3
            data[offset:offset + 3] = bytes((red, green, blue))
    message.data = data
    return message


def make_depth(node: Node, distance_mm: int) -> Image:
    message = Image()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'test_depth_camera'
    message.width = WIDTH
    message.height = HEIGHT
    message.encoding = '16UC1'
    message.is_bigendian = False
    message.step = WIDTH * 2
    low = distance_mm & 0xFF
    high = (distance_mm >> 8) & 0xFF
    message.data = [low, high] * (WIDTH * HEIGHT)
    return message


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('semantic_vehicle_detector_smoke_probe')
        self.detected = None
        self.distance = math.nan
        self.semantic_pub = self.create_publisher(
            Image, '/test/vehicle_semantic', 10
        )
        self.depth_pub = self.create_publisher(
            Image, '/test/vehicle_depth', 10
        )
        self.create_subscription(
            Bool,
            '/test/vehicle_detected',
            lambda msg: setattr(self, 'detected', bool(msg.data)),
            10,
        )
        self.create_subscription(
            Float32,
            '/test/vehicle_distance',
            lambda msg: setattr(self, 'distance', float(msg.data)),
            10,
        )

    def wait_for(
        self,
        color: tuple[int, int, int],
        predicate,
        timeout_s: float = 5.0,
        x_min: int = 10,
        x_max: int = 30,
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            depth = make_depth(self, 12000)
            semantic = make_semantic(self, color, x_min, x_max)
            semantic.header.stamp = depth.header.stamp
            self.depth_pub.publish(depth)
            self.semantic_pub.publish(semantic)
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False


def main() -> None:
    environment = os.environ.copy()
    environment.setdefault('ROS_LOG_DIR', '/tmp/ros_logs_vehicle_smoke')
    command = [
        'ros2', 'run', 'perception', 'semantic_vehicle_detector_node',
        '--ros-args',
        '-p', 'semantic_topic:=/test/vehicle_semantic',
        '-p', 'depth_topic:=/test/vehicle_depth',
        '-p', 'detected_topic:=/test/vehicle_detected',
        '-p', 'distance_topic:=/test/vehicle_distance',
        '-p', 'debug_topic:=/test/vehicle_debug',
        '-p', 'roi_x_min:=0.0',
        '-p', 'roi_y_min:=0.0',
        '-p', 'roi_x_max:=1.0',
        '-p', 'roi_y_max:=1.0',
        '-p', 'min_component_pixels:=80',
        '-p', 'min_valid_depth_pixels:=20',
        '-p', 'depth_sample_step:=1',
    ]
    detector = subprocess.Popen(
        command,
        env=environment,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    rclpy.init()
    probe = Probe()
    try:
        red_passed = probe.wait_for(
            (255, 0, 0),
            lambda: probe.detected is True
            and abs(probe.distance - 12.0) < 0.01,
        )
        if not red_passed:
            raise RuntimeError(
                'red vehicle was not detected at the expected 12.0 m'
            )

        off_lane_passed = probe.wait_for(
            (255, 0, 0),
            lambda: probe.detected is False
            and math.isinf(probe.distance),
            x_min=32,
            x_max=40,
        )
        if not off_lane_passed:
            raise RuntimeError(
                'off-lane red component was incorrectly detected as a front vehicle'
            )

        yellow_passed = probe.wait_for(
            (255, 255, 0),
            lambda: probe.detected is False
            and math.isinf(probe.distance),
        )
        if not yellow_passed:
            raise RuntimeError(
                'yellow traffic-light class was incorrectly detected as a vehicle'
            )
        print(
            'PASS: centred red vehicle=true/12.00 m; '
            'off-lane red=false/inf; yellow traffic light=false/inf'
        )
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
        detector.terminate()
        try:
            detector.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            detector.kill()
            detector.wait(timeout=3.0)


if __name__ == '__main__':
    main()
