from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    talker_node = Node(
        package="first_cpp_pkg",
        executable="talker",
        name="robot_status_talker",
        output="screen",
    )

    listener_node = Node(
        package="first_cpp_pkg",
        executable="listener",
        name="robot_status_listener",
        output="screen",
    )

    return LaunchDescription([
        talker_node,
        listener_node,
    ])