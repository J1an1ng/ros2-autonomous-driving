#!/usr/bin/env python3
"""Save every Nth ROS RGB image for YOLO annotation."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--topic',
        default='/OurCar/Sensors/RGBCameraLeft/image_raw',
    )
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path('datasets/team20_yolo/unlabelled'),
    )
    parser.add_argument('--every-n', type=int, default=10)
    parser.add_argument('--max-images', type=int, default=0)
    parser.add_argument(
        '--start-waypoint',
        type=int,
        default=None,
        help='Save only after /route/current_waypoint_id reaches this value.',
    )
    parser.add_argument(
        '--waypoint-topic',
        default='/route/current_waypoint_id',
    )
    return parser.parse_args()


def to_bgr(message: Image):
    channels = {
        'rgb8': 3,
        'bgr8': 3,
        'rgba8': 4,
        'bgra8': 4,
    }.get(message.encoding)
    if channels is None or message.height == 0 or message.width == 0:
        return None
    required = int(message.step) * int(message.height)
    if len(message.data) < required:
        return None
    rows = np.frombuffer(message.data, dtype=np.uint8, count=required).reshape(
        int(message.height), int(message.step)
    )
    image = rows[:, : int(message.width) * channels].reshape(
        int(message.height), int(message.width), channels
    )
    if message.encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if message.encoding == 'rgba8':
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if message.encoding == 'bgra8':
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


class FrameCollector(Node):
    def __init__(self, args):
        super().__init__('yolo_frame_collector')
        self.args = args
        self.args.output_dir.mkdir(parents=True, exist_ok=True)
        self.received = 0
        self.saved = 0
        self.current_waypoint = None
        self.create_subscription(
            Image, args.topic, self.on_image, qos_profile_sensor_data
        )
        if args.start_waypoint is not None:
            self.create_subscription(
                Int32, args.waypoint_topic, self.on_waypoint, 5
            )
        self.get_logger().info(
            f'Collecting {args.topic} into {args.output_dir.resolve()}'
        )

    def on_waypoint(self, message):
        self.current_waypoint = int(message.data)

    def on_image(self, message):
        if (
            self.args.start_waypoint is not None
            and (
                self.current_waypoint is None
                or self.current_waypoint < self.args.start_waypoint
            )
        ):
            return
        self.received += 1
        if (self.received - 1) % max(1, self.args.every_n):
            return
        image = to_bgr(message)
        if image is None:
            self.get_logger().warning(
                f'Unsupported image encoding: {message.encoding}'
            )
            return
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        output = self.args.output_dir / f'unity_{stamp_ns:019d}.jpg'
        if not cv2.imwrite(str(output), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            self.get_logger().error(f'Could not write {output}')
            return
        self.saved += 1
        self.get_logger().info(f'Saved {output} ({self.saved})')
        if self.args.max_images > 0 and self.saved >= self.args.max_images:
            rclpy.shutdown()


def main():
    args = parse_args()
    rclpy.init()
    node = FrameCollector(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
