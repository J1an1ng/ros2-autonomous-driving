#!/usr/bin/env python3
"""Exercise the controller's public PlannedTrajectory subscription."""

import time
import unittest

from autonomy_interfaces.msg import PlannedTrajectory, TrajectoryPoint
from geometry_msgs.msg import TwistStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from simulation.msg import VehicleControl
from std_msgs.msg import Bool, Float32


@pytest.mark.launch_test
def generate_test_description():
    """Launch an isolated controller with a fixed vehicle pose."""
    controller = launch_ros.actions.Node(
        package='controller',
        executable='controller_node',
        name='controller_node_test',
        output='screen',
        parameters=[{
            'enable_motion': True,
            'control_rate': 30.0,
            'map_frame': 'world',
            'vehicle_frame': 'test_vehicle',
            'wheelbase': 2.63,
            'maximum_steering_angle': 0.52,
            'trajectory_timeout': 0.35,
            'decision_timeout': 0.35,
            'twist_timeout': 0.35,
        }],
    )
    static_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'world', '--child-frame-id', 'test_vehicle',
        ],
        output='screen',
    )
    return launch.LaunchDescription([
        controller,
        static_transform,
        launch_testing.actions.ReadyToTest(),
    ])


class TestControllerTrajectoryInterface(unittest.TestCase):
    """Verify accepted and rejected trajectory messages through ROS topics."""

    @classmethod
    def setUpClass(cls):
        """Start one rclpy context for the launch test."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Stop the launch-test rclpy context."""
        rclpy.shutdown()

    def setUp(self):
        """Create public-interface publishers and the command observer."""
        self.node = rclpy.create_node('controller_interface_test_client')
        self.trajectory_publisher = self.node.create_publisher(
            PlannedTrajectory, '/planning/trajectory', 10
        )
        self.speed_publisher = self.node.create_publisher(
            Float32, '/decision/target_speed', 10
        )
        self.stop_publisher = self.node.create_publisher(
            Bool, '/decision/stop_required', 10
        )
        self.twist_publisher = self.node.create_publisher(
            TwistStamped, '/OurCar/INS/twist', 10
        )
        self.commands = []
        self.node.create_subscription(
            VehicleControl, '/car_command', self.commands.append, 10
        )

    def tearDown(self):
        """Destroy the test client node."""
        self.node.destroy_node()

    @staticmethod
    def trajectory(frame='world', curvatures=None, times=None):
        """Build a short valid trajectory, with selected fields overridable."""
        curvatures = curvatures or [0.0, 0.0, 0.0]
        times = times or [0, 1, 2]
        message = PlannedTrajectory()
        message.header.frame_id = frame
        for x, curvature, seconds in zip(
                [2.0, 4.0, 8.0], curvatures, times):
            point = TrajectoryPoint()
            point.pose.position.x = x
            point.pose.orientation.w = 1.0
            point.curvature = curvature
            point.target_speed = 2.0
            point.time_from_start.sec = seconds
            message.points.append(point)
        return message

    def publish_inputs(self, trajectory):
        """Publish one coherent controller input set."""
        speed = Float32()
        speed.data = 2.0
        stop = Bool()
        stop.data = False
        twist = TwistStamped()
        twist.header.frame_id = 'test_vehicle'
        self.speed_publisher.publish(speed)
        self.stop_publisher.publish(stop)
        self.twist_publisher.publish(twist)
        self.trajectory_publisher.publish(trajectory)

    def wait_for_command(self, predicate, trajectory=None, timeout=2.0):
        """Wait for a matching command, refreshing inputs when requested."""
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if trajectory is not None and now >= next_publish:
                self.publish_inputs(trajectory)
                next_publish = now + 0.05
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if any(predicate(command) for command in self.commands):
                return True
        return False

    def assert_safe_stop_for(self, trajectory):
        """Assert that a rejected trajectory produces full-brake safe stop."""
        self.commands.clear()
        self.assertTrue(self.wait_for_command(
            lambda command: command.brake >= 0.99 and command.throttle == 0.0,
            trajectory,
        ))

    def test_valid_invalid_and_stale_trajectory_behaviour(self):
        """Accept valid input and stop for every invalid or stale variant."""
        valid = self.trajectory()
        self.assertTrue(self.wait_for_command(
            lambda command: command.throttle > 0.0 and command.brake == 0.0,
            valid,
        ))

        self.assert_safe_stop_for(PlannedTrajectory())
        self.assert_safe_stop_for(self.trajectory(frame='wrong_frame'))
        self.assert_safe_stop_for(self.trajectory(times=[0, 2, 1]))
        self.assert_safe_stop_for(
            self.trajectory(curvatures=[0.0, 0.218, 0.0])
        )

        self.commands.clear()
        self.publish_inputs(valid)
        self.assertTrue(self.wait_for_command(
            lambda command: command.brake >= 0.99 and command.throttle == 0.0,
            timeout=1.5,
        ))


@launch_testing.post_shutdown_test()
class TestControllerProcesses(unittest.TestCase):
    """Check that the launched controller did not crash."""

    def test_controller_exited_cleanly(self, proc_info):
        """Require a normal test-driven shutdown."""
        launch_testing.asserts.assertExitCodes(proc_info)
